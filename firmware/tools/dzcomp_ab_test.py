"""A/B test of the dead-zone preload on a robot that stands by itself: does it step less with `!dzcomp on` than with `!dzcomp off`?

Usage: python3 tools/dzcomp_ab_test.py --port /dev/cu.usbserial-XXXX [--off-seconds 75] [--on-seconds 105] [--learn-seconds 40]
                                      [--cue] [--out DIR]

Put the robot on the flat surface where it steps, standing, and do not touch it (the phone remote stays neutral).  The tool
attaches without resetting the board and records two runs of the 55 Hz USB trace (tools/capture_trace_usb.py):

1. ``off``  the preload is forgotten and switched off (``!dzcomp reset``, ``!dzcomp off``) -- the old behaviour;
2. ``on``   the preload is forgotten and switched on (``!dzcomp reset``, ``!dzcomp on``): it learns the wheel dead zone while
            the robot stands, so only the time after ``--learn-seconds`` is judged; the first seconds are shown separately.

The robot must already be standing (policy driving) when you start; if it is not, lift it upright and let go as usual.  Only
settings commands are sent, nothing that moves the robot.  The result is judged with tools/standing_report.py: wheel travel
peak-to-peak, number of steps (back-and-forth moves over 5 mm), ground speed rms and the command energy above 3 Hz (the
buzz).  It leaves the preload switched ON when it finishes.  Output (``--out``, default logs/dzcomp-ab/<time>/): the two
traces, ``summary.json`` and ``summary.txt``.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys

try:
    import numpy as np
    import scipy.signal  # noqa: F401  (tools/standing_report.py needs it)
except ImportError as error:      # .venv/bin/python of this project has neither: fail before any recording starts
    sys.exit(f'{error}: this tool needs numpy and scipy.  Run it with the system interpreter: python3 tools/dzcomp_ab_test.py ...')

sys.path.insert(0, str(Path(__file__).resolve().parent))
import standing_report  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / 'tools' / 'capture_trace_usb.py'
WINDOW_S = 15.0


def speak(text):
    from capture_trace_usb import speak as say
    say(text)


def run_capture(port, output, seconds, commands, wait_start, cue):
    """One recording; returns the process exit status."""
    command = [sys.executable, str(CAPTURE), '--port', port, str(output), '--seconds', str(seconds), '--wait-start', str(wait_start)]
    for line in commands:
        command += ['--send', line]
    if cue:
        command.append('--cue')
    return subprocess.run(command).returncode


def summarise(windows):
    """Median over windows (robust against one push or one bump)."""
    if not windows:
        return None
    pick = lambda key: float(np.median([w[key] for w in windows]))
    return dict(windows=len(windows), p2p_mm=pick('p2p_mm'), steps=pick('steps'), v_rms_mm_s=pick('v_rms_mm_s'), hf=pick('hf'),
                pitch_sd_deg=pick('pitch_sd_deg'), preload=pick('preload'), flips_per_s=pick('flips_per_s'))


def verdict(off, on):
    """(text, label) comparing the two summaries.  `off` must show stepping for the comparison to mean anything."""
    if off is None or on is None:
        return '没有足够的数据（机器人是不是没有站起来？）', 'no_data'
    notes = []
    if on['hf'] > max(15.0, 2.0 * off['hf'] + 8.0) or on['flips_per_s'] > 8.0:
        notes.append('补偿开着时轮指令的高频抖动偏多（可能轮子在嗡嗡响）：先 `!dzcomp off`，把这份数据发给我')
    if off['p2p_mm'] < 15.0 and off['steps'] < 2:
        return '关闭补偿时机器人这段时间没有踱步（p2p<15 mm），没法比较；换它爱踱步的地面再测，或测久一点。' + ' '.join(notes), 'inconclusive'
    if on['p2p_mm'] <= 0.5 * off['p2p_mm'] and on['v_rms_mm_s'] <= 0.5 * off['v_rms_mm_s']:
        label = 'improved'
        text = f"补偿有效：轮子位移 {off['p2p_mm']:.0f} -> {on['p2p_mm']:.0f} mm，速度 rms {off['v_rms_mm_s']:.0f} -> {on['v_rms_mm_s']:.0f} mm/s。"
    else:
        label = 'no_clear_improvement'
        text = f"补偿后改善不明显：位移 {off['p2p_mm']:.0f} -> {on['p2p_mm']:.0f} mm，速度 rms {off['v_rms_mm_s']:.0f} -> {on['v_rms_mm_s']:.0f} mm/s；把数据发给我。"
    return text + (' ' + ' '.join(notes) if notes else ''), label


def judged(path, skip, window=WINDOW_S):
    rows = standing_report.load(path)
    return standing_report.analyse(rows, window=window, skip=skip) if len(rows) > 200 else []


def format_summary(name, summary):
    if summary is None:
        return f'{name:14s} (no data)'
    return (f"{name:14s} {summary['windows']:2d} windows  p2p {summary['p2p_mm']:6.1f} mm  steps/window {summary['steps']:4.1f}  "
            f"v rms {summary['v_rms_mm_s']:5.1f} mm/s  hf {summary['hf']:5.1f}  pitch sd {summary['pitch_sd_deg']:.2f}  "
            f"preload {summary['preload']:4.1f}  flips/s {summary['flips_per_s']:4.1f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--off-seconds', type=float, default=75.)
    parser.add_argument('--on-seconds', type=float, default=105.)
    parser.add_argument('--learn-seconds', type=float, default=40., help='part of the "on" run that is the learning phase, not judged')
    parser.add_argument('--wait-start', type=float, default=90., help='give up if the robot is not driving after this many seconds')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--cue', action='store_true')
    args = parser.parse_args()
    out = args.out or ROOT / 'logs' / 'dzcomp-ab' / datetime.now().strftime('%Y%m%d-%H%M%S')
    out.mkdir(parents=True, exist_ok=True)
    off_path, on_path = out / 'off.jsonl', out / 'on.jsonl'

    if args.cue:
        speak('机器人放在平地上站稳，手松开，不要碰它。开始记录第一段，补偿关闭')
    if run_capture(args.port, off_path, args.off_seconds, ['!dzcomp reset', '!dzcomp off'], args.wait_start, args.cue):
        sys.exit('the first recording failed')
    if args.cue:
        speak('第二段，补偿打开，继续不要碰它')
    if run_capture(args.port, on_path, args.on_seconds, ['!dzcomp reset', '!dzcomp on'], args.wait_start, args.cue):
        sys.exit('the second recording failed')
    if args.cue:
        speak('测试结束')

    off = summarise(judged(off_path, skip=5.))
    learning = summarise(judged(on_path, skip=0.)[: max(1, int(args.learn_seconds // WINDOW_S))])
    on = summarise(judged(on_path, skip=args.learn_seconds))
    text, label = verdict(off, on)
    lines = [format_summary('off', off), format_summary('on (learning)', learning), format_summary('on (learned)', on), '', text]
    (out / 'summary.txt').write_text('\n'.join(lines) + '\n')
    (out / 'summary.json').write_text(json.dumps(dict(off=off, on_learning=learning, on=on, verdict=label, text=text), indent=1) + '\n')
    print('\n'.join(lines))
    print(f'saved to {out}')


if __name__ == '__main__':
    main()
