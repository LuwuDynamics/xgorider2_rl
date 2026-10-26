"""Report on a free-fall-guard test recorded with `tools/capture_trace_usb.py --protocol drop` (or any USB trace).

Usage: python3 tools/drop_report.py OUT.jsonl [--dip 0.7]

For every protocol step (the `label` stored with each sample) it prints the shortest and longest stretch the total acceleration |a| (from the raw
accelerometer columns acc_x/y/z, cm/s^2, 1 g = 980.665) stayed below 0.5 g (the guard's free-fall limit) and below `--dip` g, the lowest |a|, and how many
samples carried the trace flags bit 6 (this sample below the free-fall limit) and bit 7 (a free fall is detected).  Then the stop/start events
(OUT.events.jsonl, `# ev` lines from the firmware: driving state, stop reason, lift guard) in time order, with the step each one fell into.
The trace only runs while the policy drives, so the samples stop at a protective stop; the events show why and when it restarted.
Pure Python (no numpy).
"""
import argparse
import json
import math
from pathlib import Path

G = 980.665          # cm/s^2
FREE_FALL_G = 0.5
FREE_FALL_MS = 80.0


def load(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def norm_g(r):
    return math.sqrt(r['acc_x'] ** 2 + r['acc_y'] ** 2 + r['acc_z'] ** 2) / G


def runs_below(records, limit):
    """[(start index, end index, duration ms)] of consecutive samples with |a| < limit (consecutive = no seq gap)."""
    out, start = [], None
    for i, r in enumerate(records):
        low = norm_g(r) < limit and (start is None or r['seq'] - records[i - 1]['seq'] == 1)
        if low and start is None:
            start = i
        elif not low and start is not None:
            out.append((start, i - 1))
            start = None
            if norm_g(r) < limit:
                start = i
    if start is not None:
        out.append((start, len(records) - 1))
    result = []
    for a, b in out:
        dt = records[min(b + 1, len(records) - 1)]['ms'] - records[a]['ms'] if b + 1 < len(records) else (records[b]['ms'] - records[a]['ms'] + 18)
        result.append((a, b, dt))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--dip', type=float, default=0.7, help='also report stretches below this many g')
    args = parser.parse_args()
    records = load(args.trace)
    print(f'{len(records)} samples, {records[0]["ms"] / 1000:.1f}-{records[-1]["ms"] / 1000:.1f} s of firmware time')
    labels = []
    for r in records:
        if not labels or labels[-1] != r['label']:
            labels.append(r['label'])
    print(f"{'step':10s} {'t from':>7s} {'t to':>7s} {'min|a| g':>9s} {'<0.5 g longest ms':>18s} {'<%.1f g longest ms' % args.dip:>18s} {'bit6':>5s} {'bit7':>5s} {'peak|a| g':>9s}")
    for label in labels:
        part = [r for r in records if r['label'] == label]
        g = [norm_g(r) for r in part]
        r5 = runs_below(part, FREE_FALL_G)
        r7 = runs_below(part, args.dip)
        b6 = sum(1 for r in part if r['flags'] & 64)
        b7 = sum(1 for r in part if r['flags'] & 128)
        print(f"{label:10s} {part[0]['t']:7.1f} {part[-1]['t']:7.1f} {min(g):9.2f} {max([d for _, _, d in r5] or [0]):18d} {max([d for _, _, d in r7] or [0]):18d} {b6:5d} {b7:5d} {max(g):9.2f}")
    print()
    print('lowest |a| stretches in the whole recording (below %.1f g):' % args.dip)
    stretches = sorted(runs_below(records, args.dip), key=lambda s: -s[2])[:8]
    for a, b, dt in sorted(stretches, key=lambda s: records[s[0]]['t']):
        part = records[a:b + 1]
        print(f"  t={records[a]['t']:6.2f} s step={records[a]['label']:9s} {len(part)} samples, {dt:3d} ms, min {min(norm_g(r) for r in part):.2f} g, "
              f"bit6 {sum(1 for r in part if r['flags'] & 64)}, bit7 {sum(1 for r in part if r['flags'] & 128)}")
    events_path = args.trace.with_suffix('.events.jsonl')
    if events_path.exists():
        print()
        print('events (firmware clock ms, driving, stop reason, last stop, lift guard, fall guard is in !status):')
        t0 = records[0]['ms']
        for e in load(events_path):
            text = e['text']
            ms = int(text.split('ms=')[1].split()[0]) if 'ms=' in text else None
            at = (ms - t0) / 1000. if ms is not None else None
            step = None
            if at is not None:
                before = [r for r in records if r['ms'] <= ms]
                step = before[-1]['label'] if before else 'before the first sample'
            print(f"  {'t=%+.2f s' % at if at is not None else 't=?':>12s} {('step ' + step) if step else '':22s} {text}")


if __name__ == '__main__':
    main()
