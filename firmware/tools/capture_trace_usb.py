"""Record the 55 Hz diagnostic trace over USB (no Bluetooth, so the phone app stays usable).

Usage: python tools/capture_trace_usb.py --port /dev/cu.usbserial-XXXX OUT.jsonl [--seconds 90] [--wait-start 60]
                                           [--arm] [--reset] [--cue] [--protocol lift]

The firmware prints "T,..." text lines after ``!trace usb on`` (src/trace.h has the column list): the BLE trace
fields (pitch, pitch rate, wheel commands and speeds, ...) plus the wheel servo load, wheel positions, the raw
accelerometer, the firmware clock, how fresh each servo's feedback is, the control period, the leg servos' load and position
and the raw battery ADC.  It also prints a
"# ev ..." line whenever the driving state, the stop reason or the lift guard changes; those go to OUT.events.jsonl.  Each line becomes one JSON record with the same keys as capture_trace.py
(so tools/trace_report.py reads both), and the new columns under their own names.  The trace only runs while the
policy drives, so the clock starts at the FIRST sample: ``--seconds`` is the recording length from then on and
``--wait-start`` how long to wait for the robot to be lifted upright and start.

Attaching does not reset the board (RTS is released before DTR, so the auto-reset circuit never pulses EN); a running
robot keeps running.  ``--reset`` reboots it on purpose (it then comes back with the policy active and waits for an upright
robot).  Other serial tools that assert DTR and RTS together on open can still reset it.
``--liftguard on|off`` sends ``!liftguard on|off`` first (the guard that holds the wheel command at zero while the wheels run away in
the air; the detector always records, this only decides whether it acts).
``--send LINE`` (repeatable) sends a settings command before the stream starts; only ``!dzcomp ...``, ``!liftguard ...`` and ``!fallguard ...`` are
accepted (e.g. ``--send "!dzcomp off"``, ``--send "!fallguard ms 40"`` for the hand-drop test), nothing that moves the robot.
``--arm`` additionally sends ``!arm`` (select the policy and wait for an upright, still robot -- the phone's stand-up
button); use it after a protective stop, never on a robot that is balancing.  Nothing else that moves the robot is ever
sent, and ``!trace usb off`` ends the stream.

``--cue`` plays macOS sounds at the start and end; ``--protocol lift`` additionally speaks a schedule in Chinese
(``lift``: hold upright until it starts, let go, lift x4 with put-downs, push, phone drive; ``guard``: stand, push, phone drive, lift twice; ``drop``: hand drops to test the free-fall guard; ``release``: real releases from ~10 cm and a tip-over backwards; ``guard2``: the same with two quick lift/put-down/let-go cycles), reminds you every 12 s until the
robot has started, and labels every record with the current step -- see PROTOCOLS.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

COLUMNS = ('seq', 'ms', 'pitch', 'pitch_rate', 'roll', 'yaw_rate', 'cmd11', 'cmd21', 'sp11', 'sp21', 'fsp11', 'fsp21',
           'legA', 'legB', 'a0', 'a1', 'load11', 'load21', 'pos11', 'pos21', 'acc_x', 'acc_y', 'acc_z', 'flags',
           'age11', 'age21', 'age12', 'age22', 'dt_us', 'load12', 'load22', 'pos12', 'pos22', 'vbat_adc')

# (seconds from the start, spoken prompt, label) -- the label is stored with every record that follows it.
PROTOCOLS = {
    'lift': [
        (0, '开始，托直立不动', 'start'),
        (10, '放手，让它自己站着', 'stand'),
        (22, '拿起', 'lift1'), (28, '放下', 'stand1'),
        (36, '拿起', 'lift2'), (42, '放下', 'stand2'),
        (50, '拿起，再倾斜一下', 'lift3'), (57, '放下', 'stand3'),
        (65, '快速拿起', 'lift4'), (70, '放下', 'stand4'),
        (78, '轻轻推一下', 'push'), (85, '站稳', 'stand5'),
        (92, '用手机遥控', 'drive'),
        (112, '结束', 'end'),
    ],
}
PROTOCOLS['guard'] = [
    (0, '开始，托直立不动', 'start'),
    (10, '放在地上，手松开', 'floor'),
    (30, '轻轻推它，前后左右各一次', 'push'),
    (50, '用手机遥控，前进后退转弯', 'drive'),
    (75, '拿起来，在空中托住', 'lift1'),
    (84, '放回地上，托直立，等它起来再松手', 'down1'),
    (96, '再拿起来，倾斜一下，再放下', 'lift2'),
    (108, '站稳', 'floor2'),
    (116, '结束', 'end'),
]
PROTOCOLS['guard2'] = [      # guard + two quick pick-up/put-down/let-go cycles to measure how long the wheels stay off
    (0, '开始，托直立不动', 'start'),
    (10, '放在地上，手松开', 'floor'),
    (30, '轻轻推它，前后左右各一次', 'push'),
    (50, '用手机遥控，前进后退转弯', 'drive'),
    (70, '拿起来，在空中托住', 'lift1'),
    (78, '放回地上，托直立，等它起来再松手', 'down1'),
    (90, '拿起来，马上放下，松手', 'quick1'),
    (98, '站稳', 'floor2'),
    (104, '再拿起来，马上放下，松手', 'quick2'),
    (112, '站稳', 'floor3'),
    (118, '结束', 'end'),
]
PROTOCOLS['drop'] = [        # free-fall guard test: hand drops while the robot is held upright and balancing in the hand
    (0, '开始，托直立不动', 'start'),
    (10, '快速往下放，模拟掉落，然后托稳', 'drop1'),
    (20, '托直立不动，等它重新起来', 'recover1'),
    (30, '再快速往下放一次', 'drop2'),
    (40, '托直立不动，等它重新起来', 'recover2'),
    (50, '这次慢慢往下放，不要快', 'slow'),
    (58, '托稳', 'hold'),
    (64, '最后一次，用最快的速度往下放', 'drop3'),
    (74, '托直立不动', 'recover3'),
    (82, '结束', 'end'),
]
PROTOCOLS['release'] = [     # free-fall guard test with REAL falls: two releases from ~10 cm onto something soft, then a tip-over backwards
    (0, '开始，托直立不动', 'start'),
    (12, '托在软垫上方十厘米，三秒后松手', 'ready1'),
    (16, '松手', 'drop1'),
    (24, '拿起来，托直立，等它重新起来', 'recover1'),
    (40, '托在软垫上方十厘米，三秒后松手', 'ready2'),
    (44, '松手', 'drop2'),
    (52, '拿起来，托直立，等它重新起来', 'recover2'),
    (68, '放在软垫上，让它站稳，手松开', 'stand'),
    (78, '轻轻推它一下，让它向后倒下', 'tipback'),
    (88, '拿起来，托直立，等它重新起来', 'recover3'),
    (100, '结束', 'end'),
]
CHINESE_VOICE = 'Flo (中文（中国大陆）)'    # macOS zh_CN voice; `say -v '?'` lists the installed ones


def parse_line(line):
    """One 'T,...' text line -> record dict, or None for headers, JSON status lines and damaged lines."""
    if not line.startswith('T,'):
        return None
    parts = line[2:].strip().split(',')
    if len(parts) != len(COLUMNS):
        return None
    try:
        values = [int(p) for p in parts]
    except ValueError:
        return None
    return dict(zip(COLUMNS, values))


def speak(text):
    if sys.platform != 'darwin':
        return
    command = ['say']
    if any('\u4e00' <= ch <= '\u9fff' for ch in text):
        command += ['-v', CHINESE_VOICE]
    subprocess.Popen(command + [text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def play(sound):
    path = Path('/System/Library/Sounds') / (sound + '.aiff')
    if sys.platform == 'darwin' and path.exists():
        subprocess.Popen(['afplay', str(path)])


def run_protocol(steps, started, label_box, stop):
    for at, text, label in steps:
        while not stop.is_set() and time.monotonic() - started < at:
            time.sleep(.05)
        if stop.is_set():
            return
        label_box[0] = label
        speak(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output', type=Path)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=90., help='recording length after the first sample')
    parser.add_argument('--wait-start', type=float, default=60., help='give up if no sample arrives within this many seconds')
    parser.add_argument('--arm', action='store_true', help='send !arm after enabling the stream (after a protective stop)')
    parser.add_argument('--reset', action='store_true', help='reboot the board with an RTS pulse first')
    parser.add_argument('--liftguard', choices=('on', 'off'), help='send !liftguard on/off before the stream starts')
    parser.add_argument('--send', action='append', default=[], metavar='LINE',
                        help='settings command to send before the stream starts (repeatable); only !dzcomp ..., !liftguard ... and !fallguard ...')
    parser.add_argument('--cue', action='store_true')
    parser.add_argument('--protocol', choices=sorted(PROTOCOLS))
    args = parser.parse_args()
    for line in args.send:
        if not (line.startswith('!dzcomp') or line.startswith('!liftguard') or line.startswith('!fallguard')) or '\n' in line:
            parser.error(f'--send only accepts !dzcomp, !liftguard and !fallguard commands, not {line!r}')

    import serial
    port = serial.Serial(port=None, baudrate=115200, timeout=.1)
    port.rts = False            # RTS (EN) is released before the open and DTR only after it: attaching never resets the board
    port.port = args.port
    port.open()
    port.dtr = False
    if args.reset:
        port.rts = True
        time.sleep(.12)
        port.rts = False
    boot = b''
    deadline = time.monotonic() + (3.5 if args.reset else 1.0)
    while time.monotonic() < deadline:
        boot += port.read(4096)
    reset_seen = b'Rider2 application' in boot or b'POWERON_RESET' in boot
    print('board rebooted while opening the port' if reset_seen else 'attached to the running firmware', flush=True)

    port.reset_input_buffer()
    if args.liftguard:
        port.write(f'!liftguard {args.liftguard}\n'.encode())
    for line in args.send:
        port.write(line.encode() + b'\n')
        port.flush()
        time.sleep(.05)
    port.write(b'!trace usb on\n')
    if args.arm:
        port.write(b'!arm\n')
    port.flush()
    records, events, label_box, stop = [], [], ['wait'], threading.Event()
    header_seen = False
    begun = time.monotonic()
    first_sample_at = None
    last_reminder = -99.
    if args.cue:
        play('Ping')
    buffer = b''
    last_data = begun
    try:
        while True:
            now = time.monotonic()
            if first_sample_at is None:
                if now - begun > args.wait_start:
                    print(f'no sample within {args.wait_start:.0f} s: the robot never started', flush=True)
                    break
                if args.protocol and now - last_reminder >= 12:
                    last_reminder = now
                    speak('托直立不动')
            elif now - first_sample_at >= args.seconds:
                break
            chunk = port.read(4096)
            if chunk:
                last_data = time.monotonic()
                buffer += chunk
            elif not header_seen and time.monotonic() - last_data > 2.5:
                port.write(b'!trace usb on\n')      # lost during a boot: ask again
                port.flush()
                last_data = time.monotonic()
            while b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                text = raw.decode('utf-8', errors='replace').strip()
                if text.startswith('# trace'):
                    header_seen = True
                if text.startswith('# ev '):
                    events.append(dict(host_t=round(time.monotonic() - begun, 3), text=text[5:],
                                       t=round(time.monotonic() - first_sample_at, 3) if first_sample_at else None))
                record = parse_line(text)
                if record:
                    if first_sample_at is None:
                        first_sample_at = time.monotonic()
                        label_box[0] = 'start'
                        print('the policy is driving: recording', flush=True)
                        if args.protocol:
                            threading.Thread(target=run_protocol, args=(PROTOCOLS[args.protocol], first_sample_at, label_box, stop),
                                             daemon=True).start()
                    record['label'] = label_box[0]
                    record['host_t'] = round(time.monotonic() - begun, 3)
                    record['t'] = round(time.monotonic() - first_sample_at, 3)
                    records.append(record)
    except KeyboardInterrupt:
        print('interrupted; saving what was recorded', flush=True)
    finally:
        stop.set()
        try:
            port.write(b'!trace usb off\n')
            port.flush()
        except Exception:
            pass
        port.close()
        if args.cue:
            play('Glass')
    args.output.write_text(''.join(json.dumps(r) + '\n' for r in records))
    if events:
        events_path = args.output.with_suffix('.events.jsonl')
        events_path.write_text(''.join(json.dumps(e) + '\n' for e in events))
        print(f'{len(events)} stop/start events (driving state, stop reason, lift guard) written to {events_path}')
    gaps = sum(1 for a, b in zip(records, records[1:]) if b['seq'] - a['seq'] > 1)
    lost = sum(b['seq'] - a['seq'] - 1 for a, b in zip(records, records[1:]) if b['seq'] - a['seq'] > 1)
    seconds = (records[-1]['ms'] - records[0]['ms']) / 1000. if len(records) > 1 else 0.
    print(f'wrote {len(records)} samples ({seconds:.1f} s of firmware time, {gaps} gaps, {lost} samples lost) to {args.output}')
    if not records:
        print('no samples: the trace only runs while the policy is driving -- lift the robot upright so it starts')


if __name__ == '__main__':
    main()
