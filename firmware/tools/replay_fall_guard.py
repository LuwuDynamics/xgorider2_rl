"""Would the free-fall guard have fired on these recordings?  Replays traces through the firmware's own detector (src/fall_guard.h).

Usage: python3 tools/replay_fall_guard.py TRACE.jsonl [TRACE.jsonl ...] [--limit-g 0.5] [--ms 60]
       python3 tools/replay_fall_guard.py --all            (every trace under logs/)

Each trace (tools/capture_trace_usb.py, tools/incident_snapshot.py) has the raw accelerometer in cm/s^2 (acc_x/acc_y/acc_z); the
total is |a| / 981 in g.  A 55 Hz sample is held for ~18 ms and fed at the 5 ms control tick, so a one-sample dip counts as 18 ms
(the real detector sees the accelerometer at ~200 Hz).  Prints per file: trips, the longest run below the limit and the minimum.
A trace of a REAL drop must show a trip; handling, pushes and standing must show none (nothing stayed below 0.6 g for two
consecutive samples in the recordings of 2026-10-22).
"""
import argparse
import glob
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def build():
    out = Path(tempfile.gettempdir()) / 'replay_fall_guard'
    sources = [ROOT / 'tests/replay_fall_guard.cpp', ROOT / 'src/fall_guard.h']
    if not out.exists() or out.stat().st_mtime < max(p.stat().st_mtime for p in sources):
        subprocess.run(['clang++', '-std=c++17', '-O1', '-I', str(ROOT / 'src'), str(sources[0]), '-o', str(out)], check=True)
    return out


def samples(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if line.startswith('{'):
            row = json.loads(line)
            if 'acc_x' in row and 'ms' in row:
                rows.append((row['ms'], math.sqrt(row['acc_x'] ** 2 + row['acc_y'] ** 2 + row['acc_z'] ** 2) / 981.0))
    return rows


def replay(binary, rows, limit_g, ms):
    text = ''.join(f'{t} {g}\n' for t, g in rows)
    out = subprocess.run([str(binary), str(limit_g), str(ms)], input=text, capture_output=True, text=True, check=True).stdout.split()
    return int(out[0]), float(out[1]), float(out[2])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('traces', nargs='*')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--limit-g', type=float, default=0.5)
    parser.add_argument('--ms', type=float, default=60., help='hold time in ms (the firmware default is 60; `!fallguard ms N` changes it on the robot)')
    args = parser.parse_args()
    files = list(args.traces)
    if args.all:
        files += sorted(glob.glob(str(ROOT / 'logs/**/*.jsonl'), recursive=True))
    files = [f for f in files if not f.endswith(('events.jsonl', 'cycles.jsonl', 'samples.jsonl'))]
    if not files:
        sys.exit('no traces given')
    binary = build()
    print(f"limit {args.limit_g} g for {args.ms:.0f} ms")
    print(f"{'trace':70s} {'samples':>8} {'seconds':>7} {'trips':>5} {'longest low ms':>14} {'min g':>6}")
    total = 0
    for f in files:
        rows = samples(f)
        if len(rows) < 100:
            continue
        trips, longest, minimum = replay(binary, rows, args.limit_g, args.ms)
        seconds = (rows[-1][0] - rows[0][0]) / 1000.
        total += trips
        print(f"{str(Path(f).relative_to(ROOT) if Path(f).is_absolute() and ROOT in Path(f).parents else f)[-70:]:70s} {len(rows):8d} {seconds:7.0f} {trips:5d} {longest:14.0f} {minimum:6.2f}")
    print('total trips:', total)


if __name__ == '__main__':
    main()
