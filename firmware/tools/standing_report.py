"""Does the robot stand still?  Windowed stepping / jitter report for a USB trace of a robot standing on a flat surface.

Usage: python3 tools/standing_report.py TRACE.jsonl [--window 15] [--skip 0]   (system python3: needs numpy and scipy)
       (TRACE from tools/capture_trace_usb.py, tools/incident_snapshot.py or a `!trace dump`)

Why: a standing robot used to creep and kick back and forth (wheel dead zone, see src/policy/dead_zone_comp.h).  This
prints, per window, what the wheels did, so a firmware or `!dzcomp` setting can be judged with numbers instead of by eye:

* ``p2p mm``    peak-to-peak wheel travel in the window (from the raw wheel encoders, 1024 ticks/rev, unwrapped);
* ``steps``     direction reversals of the wheel travel that are larger than ``--step-mm`` (default 5 mm): one back-and-forth
                step counts as two;
* ``v rms``     rms ground speed in mm/s (encoder derivative, 0.2 s smoothing);
* ``hf``        rms of the common-mode wheel command above 3 Hz (the buzz you hear/feel; the policy alone gives ~1 when
                standing, a healthy driven robot 5-10);
* ``pitch sd``  standard deviation of the pitch, degrees;
* ``preload``   the dead-zone preload actually added to the command ((cmd11 - a0) - (cmd21 - a1)) / 2, median magnitude;
* ``flips/s``   how often that preload changed sign (chattering shows as more than ~8).

The tool never opens a serial port.  Wheel travel uses the rolling diameter measured on 2026-10-22 (0.173 mm per tick).
"""
import argparse
import json
from pathlib import Path
import sys

try:
    import numpy as np
    import scipy.signal  # noqa: F401  (the 3 Hz high-pass)
except ImportError as error:
    sys.exit(f'{error}: this tool needs numpy and scipy.  Run it with the system interpreter: python3 tools/standing_report.py ...')

MM_PER_TICK = 0.9 / 0.878 * 55 * np.pi / 1024     # calibrated against a hand-rolled 0.9 m (logs/wheel-scale-20261022)
RING = 1024


def unwrap(positions):
    step = np.diff(np.asarray(positions, float))
    step = (step + RING / 2) % RING - RING / 2    # shortest signed step on the encoder ring
    return np.concatenate([[0.], np.cumsum(step)])


def load(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if line.startswith('{'):
            row = json.loads(line)
            if 'ms' in row and 'pos11' in row and 'cmd11' in row:
                rows.append(row)
    return rows


def count_steps(travel_mm, threshold_mm):
    """Direction reversals of a travel curve that move at least `threshold_mm` between two turning points (zig-zag count)."""
    high = low = float(travel_mm[0])
    direction, turns = 0, 0              # +1 rising, -1 falling, 0 not decided yet
    for value in travel_mm[1:]:
        high, low = max(high, value), min(low, value)
        if direction == 0:
            if high - value >= threshold_mm:
                direction, high, low = -1, value, value
            elif value - low >= threshold_mm:
                direction, high, low = 1, value, value
        elif direction == 1 and high - value >= threshold_mm:
            turns += 1
            direction, high, low = -1, value, value
        elif direction == -1 and value - low >= threshold_mm:
            turns += 1
            direction, high, low = 1, value, value
    return turns


def hf_rms(signal_values, fs):
    if len(signal_values) < int(fs * 2):
        return 0.
    from scipy import signal
    sos = signal.butter(4, 3., 'highpass', fs=fs, output='sos')
    return float(np.sqrt(np.mean(signal.sosfilt(sos, signal_values - np.mean(signal_values))[int(fs):] ** 2)))


def analyse(rows, window=15., skip=0., step_mm=5.):
    """Per-window metrics.  `rows` as written by capture_trace_usb.py (jsonl records)."""
    t = np.array([r['ms'] for r in rows], float) / 1000.
    t -= t[0]
    ticks11, ticks21 = unwrap([r['pos11'] for r in rows]), unwrap([r['pos21'] for r in rows])
    travel = 0.5 * (ticks11 - ticks21) * MM_PER_TICK     # wheel 21 is mounted mirrored: forward = falling position
    cmd = np.array([0.5 * (r['cmd11'] - r['cmd21']) for r in rows], float)
    pre = np.array([0.5 * ((r['cmd11'] - r['a0']) - (r['cmd21'] - r['a1'])) for r in rows], float)
    pitch = np.array([r['pitch'] for r in rows], float) / 100.
    out = []
    start = skip
    while start + window <= t[-1] + 1e-9:
        m = (t >= start) & (t < start + window)
        if m.sum() < 50:
            start += window
            continue
        tt, x = t[m], travel[m]
        fs = (m.sum() - 1) / (tt[-1] - tt[0])
        smooth = max(1, int(0.2 * fs))
        v = np.gradient(np.convolve(x, np.ones(smooth) / smooth, mode='same'), tt)
        p = pre[m]
        sign = np.sign(np.where(np.abs(p) > 3., p, 0.))
        nz = sign[sign != 0]
        flips = float(np.sum(nz[1:] != nz[:-1]) / (tt[-1] - tt[0])) if len(nz) > 1 else 0.
        out.append(dict(t0=float(start), p2p_mm=float(x.max() - x.min()), steps=count_steps(x, step_mm),
                        v_rms_mm_s=float(np.sqrt(np.mean(v[smooth:-smooth] ** 2))) if len(v) > 4 * smooth else 0.,
                        hf=hf_rms(cmd[m], fs), pitch_sd_deg=float(np.std(pitch[m])),
                        preload=float(np.median(np.abs(p))), flips_per_s=flips))
        start += window
    return out


def format_table(windows):
    lines = [f"{'t0 s':>6} {'p2p mm':>7} {'steps':>6} {'v rms':>6} {'hf':>6} {'pitch sd':>8} {'preload':>8} {'flips/s':>8}"]
    for w in windows:
        lines.append(f"{w['t0']:6.0f} {w['p2p_mm']:7.1f} {w['steps']:6d} {w['v_rms_mm_s']:6.1f} {w['hf']:6.1f} {w['pitch_sd_deg']:8.2f} "
                     f"{w['preload']:8.1f} {w['flips_per_s']:8.1f}")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--window', type=float, default=15.)
    parser.add_argument('--skip', type=float, default=0., help='ignore this many seconds at the start')
    parser.add_argument('--step-mm', type=float, default=5., help='smallest back-and-forth that counts as a step')
    args = parser.parse_args()
    rows = load(args.trace)
    if len(rows) < 200:
        sys.exit(f'only {len(rows)} usable samples in {args.trace}: record at least a few seconds of the robot standing')
    windows = analyse(rows, args.window, args.skip, args.step_mm)
    seconds = (rows[-1]['ms'] - rows[0]['ms']) / 1000.
    print(f'{len(rows)} samples, {seconds:.0f} s')
    print('\n'.join(format_table(windows)))


if __name__ == '__main__':
    main()
