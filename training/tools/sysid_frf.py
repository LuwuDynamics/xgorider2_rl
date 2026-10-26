"""Extract the real robot's wheel-command -> body response from firmware trace logs.

Input: ``data/real_robot_traces/*.jsonl`` captured with ``tools/capture_trace.py`` (55 Hz BLE
trace: pitch, pitch_rate, cmd11/21, sp11/21 ...).  While the robot balances it rings; the ringing is a
narrow-band signal with coherence ~1 between command and response, so the *closed-loop* data
identify the plant at the ringing frequency without any special excitation.

For every ~4 s balanced chunk this fits  H(f*) = S_uy / S_uu  at the dominant command frequency f*
(Welch cross-spectra), for

* ``rate``:  firmware pitch rate [rad/s] per common wheel command unit (cmd11 - cmd21)/2,
* ``wheel``: mean wheel speed [rad/s] (sp/27.5mm) per command unit,

then reports medians in frequency bins -> ``models/measured_frf.json`` (consumed by
``rider_sim/plant.py``).  Re-run it after capturing new traces; the whole sim calibration follows.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / 'data/real_robot_traces'
FS = 1 / 0.018        # firmware emits every third 6 ms control tick
BINS = ((4, 5.5), (5.5, 6.5), (6.5, 7.5), (7.5, 8.5), (8.5, 9.5), (9.5, 10.5), (10.5, 12.5), (12.5, 15))


def load(path):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    return [r for r in rows if 'seq' in r and 'cmd11' in r]


def arr(rows, key):
    return np.array([r[key] for r in rows], float)


def balanced_runs(rows, limit_deg=20, minimum=150):
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


def chunk_frf(rows):
    u = (arr(rows, 'cmd11') - arr(rows, 'cmd21')) / 2.
    rate = arr(rows, 'pitch_rate') / 1000.
    wheel = (arr(rows, 'sp11') + arr(rows, 'sp21')) / 2. / 27.5
    kw = dict(fs=FS, nperseg=96, noverlap=64)
    f, Suu = signal.welch(u - u.mean(), **kw)
    _, Sur = signal.csd(u - u.mean(), rate - rate.mean(), **kw)
    _, Suw = signal.csd(u - u.mean(), wheel - wheel.mean(), **kw)
    _, Srr = signal.welch(rate - rate.mean(), **kw)
    m = (f > 3) & (f < 20)
    i = int(np.argmax(Suu[m]))
    coherence = float(np.abs(Sur[m][i]) ** 2 / (Suu[m][i] * Srr[m][i]))
    return float(f[m][i]), coherence, Sur[m][i] / Suu[m][i], Suw[m][i] / Suu[m][i]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--logs', type=Path, nargs='+', default=sorted(LOGS.glob('*.jsonl')))
    parser.add_argument('--output', type=Path, default=ROOT / 'models/measured_frf.json')
    args = parser.parse_args()
    chunks = []
    for path in args.logs:
        try:
            rows = load(path)
        except Exception:
            continue
        for a, b in balanced_runs(rows):
            for s in range(a, b - 150, 110):
                part = rows[s:min(b, s + 220)]
                if len(part) < 150:
                    continue
                f, coh, hr, hw = chunk_frf(part)
                if coh >= .9:
                    chunks.append((f, hr, hw))
    if not chunks:
        raise SystemExit('no usable chunks')
    out = dict(freqs_hz=[], rate_mag=[], rate_phase_deg=[], wheel_mag=[], wheel_phase_deg=[], counts=[])
    for lo, hi in BINS:
        sel = [(f, hr, hw) for f, hr, hw in chunks if lo <= f < hi]
        if len(sel) < 2:
            continue
        out['freqs_hz'].append(float(np.median([c[0] for c in sel])))
        out['rate_mag'].append(float(np.median([abs(c[1]) for c in sel])))
        out['rate_phase_deg'].append(float(np.median([np.degrees(np.angle(c[1])) for c in sel])))
        out['wheel_mag'].append(float(np.median([abs(c[2]) for c in sel])))
        out['wheel_phase_deg'].append(float(np.median([np.degrees(np.angle(c[2])) for c in sel])))
        out['counts'].append(len(sel))
    f = np.array(out['freqs_hz'])
    k = int(np.argmin(np.abs(f - 8.)))
    out['ref_hz'] = 8.0
    out['rate_per_unit'] = out['rate_mag'][k]
    out['wheel_to_rate'] = float(np.median(np.array(out['wheel_mag']) / np.array(out['rate_mag'])))
    out['chunks'] = len(chunks)
    out['source'] = [(str(Path(p).resolve().relative_to(ROOT.parent)) if ROOT.parent in Path(p).resolve().parents else Path(p).name) for p in args.logs]
    args.output.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
