"""Judge a real-robot trace (``tools/capture_trace.py``, 55 Hz) by the numbers the user feels: ringing and wobble.

For every balanced stretch (|pitch| < 20 deg, no gaps) it prints

* pitch / pitch-rate standard deviation (deg, deg/s),
* ``cmd_hf_rms``: rms of the common wheel command (cmd11 - cmd21)/2 above 3 Hz -- the same definition as
  ``evaluate2.py`` (the simulator prints 0.8-9 for v6; the previous policies measured 79-317 on the robot),
* the strongest command frequency between 3 and 27 Hz and how much of the command variance sits above 3 Hz,
* the median wheel speed and the lean (mean pitch) -- a steady lean means the loop is fighting a bias.

Usage:  python tools/trace_report.py TRACE.jsonl [TRACE2.jsonl ...]      (no argument: the old floor-diagnosis runs)
"""
import json
from pathlib import Path
import sys
import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parents[1]
OLD = sorted((ROOT / 'data/real_robot_traces').glob('trace-run*.jsonl'))
FS = 1 / 0.018        # the firmware emits every third 6 ms control tick


def load(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if 'seq' in r and 'cmd11' in r:
            rows.append(r)
    return rows


def arr(rows, key):
    return np.array([r[key] for r in rows], float)


def balanced_runs(rows, limit_deg=20, minimum=100):
    seq, pitch = arr(rows, 'seq'), arr(rows, 'pitch') / 100.
    runs, start = [], None
    for i in range(len(rows)):
        bad = abs(pitch[i]) >= limit_deg or (i > 0 and seq[i] - seq[i - 1] > 1)
        if start is None:
            if not bad:
                start = i
        elif bad:
            if i - start >= minimum:
                runs.append((start, i))
            start = None if abs(pitch[i]) >= limit_deg else i
    if start is not None and len(rows) - start >= minimum:
        runs.append((start, len(rows)))
    return runs


def active(rows):
    """Samples where the wheels are being commanded (the robot is waiting, not driving, otherwise)."""
    return np.abs(arr(rows, 'cmd11')) + np.abs(arr(rows, 'cmd21')) > 0


def metrics(rows):
    u = (arr(rows, 'cmd11') - arr(rows, 'cmd21')) / 2.
    pitch = arr(rows, 'pitch') / 100.
    rate = np.degrees(arr(rows, 'pitch_rate') / 1000.)
    speed = (arr(rows, 'fsp11') + arr(rows, 'fsp21')) / 2.
    sos = signal.butter(4, 3., 'highpass', fs=FS, output='sos')
    hf = signal.sosfilt(sos, u - u.mean())
    hf = hf[len(hf) // 8:]              # drop the filter start-up
    f, p = signal.welch(u - u.mean(), fs=FS, nperseg=min(128, len(u)), noverlap=min(96, len(u) // 2))
    band = (f >= 3) & (f <= 27)
    peak = f[band][int(np.argmax(p[band]))] if band.any() else float('nan')
    share = float(np.sum(p[f >= 3]) / np.sum(p[f > 0.2])) if np.sum(p[f > 0.2]) > 0 else float('nan')
    return dict(seconds=len(rows) * 0.018, pitch_mean=float(pitch.mean()), pitch_sd=float(pitch.std()), rate_sd=float(rate.std()),
                cmd_sd=float(u.std()), cmd_hf_rms=float(np.sqrt(np.mean(hf ** 2))), peak_hz=float(peak), hf_share=share,
                speed_median=float(np.median(speed)), speed_abs_p95=float(np.percentile(np.abs(speed), 95)))


def report(path):
    rows = load(path)
    print(f'== {Path(path).name}: {len(rows)} samples, {len(rows) * 0.018:.1f} s')
    if not rows:
        return []
    gate = active(rows)
    print(f'   wheels commanded in {gate.mean() * 100:.0f}% of the samples')
    out = []
    for a, b in balanced_runs(rows):
        seg = [r for r, on in zip(rows[a:b], gate[a:b]) if on]
        if len(seg) < 100:
            continue
        m = metrics(seg)
        out.append(m)
        print(f"   run {a:5d}-{b:5d} {m['seconds']:5.1f}s  pitch {m['pitch_mean']:+5.2f}+-{m['pitch_sd']:.2f} deg  rate sd {m['rate_sd']:6.1f} deg/s  "
              f"cmd sd {m['cmd_sd']:6.1f}  cmd>3Hz rms {m['cmd_hf_rms']:6.1f}  peak {m['peak_hz']:4.1f} Hz  >3Hz share {m['hf_share'] * 100:3.0f}%  "
              f"speed med {m['speed_median']:+6.1f} p95 {m['speed_abs_p95']:6.1f} mm/s")
    if not out:
        print('   no balanced stretch with commanded wheels (>= 1.8 s)')
    return out


if __name__ == '__main__':
    files = sys.argv[1:] or [str(p) for p in OLD]
    for f in files:
        report(f)
