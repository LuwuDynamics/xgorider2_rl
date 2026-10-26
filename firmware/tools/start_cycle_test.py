"""Reset-then-start test: reboot the board N times and check on every start that all four servos keep answering.

Usage: python tools/start_cycle_test.py --port /dev/cu.usbserial-XXXX [--cycles 20] [--observe 2.5] [--out DIR] [--cue]

Why: the leg servo ID 12 went silent three times on 2026-10-22, each time on the very first drive tick after an ESP32
reset (the legs get their first command then).  Whether that is the servo (wiring, supply, mechanics) or the firmware
needs a failure rate per servo, which means many starts.  Hold the robot upright and still for the whole run.

Every cycle: RTS pulse (reboot) -> wait for the policy to start (needs an upright, still robot ~3-5 s after boot) ->
watch the 55 Hz USB trace for ``--observe`` seconds -> ``!stop`` -> next.  It judges each start by the servo
freshness columns (``age11/21/12/22``) and the ``# ev`` stop events:

* ok       the trace keeps flowing and every servo stays fresh (wheels < 100 ms, legs < 250 ms, the firmware's limits),
* fail     a servo went stale or the policy stopped inside the window; the silent servo is named,
* no_start no sample within ``--start-timeout`` (robot not upright?); a ``!status`` snapshot then says whether a servo
           is still silent from the previous cycle.

For ok starts it also keeps what the leg servos did (load and position of 12 and 22 -- compare them) and the raw battery
ADC around the start.  Output (``--out``, default logs/start-cycles/<time>/): ``cycles.jsonl`` (one record per cycle),
``samples.jsonl`` (every trace sample with its cycle number) and ``summary.txt``.

Commands sent: the RTS reset, ``!trace usb on/off``, ``!status`` (only when no start happened) and ``!stop``.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_trace_usb import parse_line  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SERVOS = ('11', '21', '12', '22')
STALE_MS = {'11': 100, '21': 100, '12': 250, '22': 250}      # the firmware's own limits (runtime_guard.cpp)
CHINESE_VOICE = 'Flo (中文（中国大陆）)'


def say(text):
    if sys.platform == 'darwin':
        subprocess.Popen(['say', '-v', CHINESE_VOICE, text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def tick():
    sound = Path('/System/Library/Sounds/Tink.aiff')
    if sys.platform == 'darwin' and sound.exists():
        subprocess.Popen(['afplay', str(sound)])


class Lines:
    """Line reader on top of a pyserial-like port."""

    def __init__(self, port):
        self.port = port
        self.buffer = b''

    def poll(self):
        chunk = self.port.read(4096)
        if chunk:
            self.buffer += chunk
        out = []
        while b'\n' in self.buffer:
            raw, self.buffer = self.buffer.split(b'\n', 1)
            out.append(raw.decode('utf-8', errors='replace').strip())
        return out

    def clear(self):
        self.port.reset_input_buffer()
        self.buffer = b''


def parse_event(line):
    if not line.startswith('# ev '):
        return None
    fields = dict(part.split('=', 1) for part in line[5:].split() if '=' in part)
    return fields


def classify(samples, events, expected_minimum):
    """Judge the observation window of one start."""
    ages = {s: max((r['age' + s] for r in samples), default=0) for s in SERVOS}
    stale = [s for s in SERVOS if ages[s] > STALE_MS[s] * 0.6]       # clearly on its way to the firmware's limit
    stopped = [e for e in events if e.get('drv') == '0']
    if stale or stopped or len(samples) < expected_minimum:
        silent = max(SERVOS, key=lambda s: ages[s]) if stale else None
        reason = stopped[0].get('last_stop', stopped[0].get('reason')) if stopped else None
        return 'fail', silent, reason, ages
    return 'ok', None, None, ages


def start_details(samples):
    """What the legs and the supply did around the start (only meaningful when the servos kept answering)."""
    if not samples:
        return {}
    early = [r for r in samples if r['ms'] - samples[0]['ms'] <= 1000]
    first = samples[0]
    return dict(
        load12_peak=max(abs(r['load12']) for r in early), load22_peak=max(abs(r['load22']) for r in early),
        pos12_span=max(r['pos12'] for r in early) - min(r['pos12'] for r in early),
        pos22_span=max(r['pos22'] for r in early) - min(r['pos22'] for r in early),
        vbat_adc_first=first['vbat_adc'], vbat_adc_min=min(r['vbat_adc'] for r in early),
        leg_a_commanded=(first['legA'], early[-1]['legA']), leg_b_commanded=(first['legB'], early[-1]['legB']))


def one_cycle(port, lines, args, cycle):
    port.rts = True
    time.sleep(.12)
    port.rts = False
    began = time.monotonic()
    deadline = began + args.boot_wait
    while time.monotonic() < deadline:
        lines.poll()                                    # let the boot banner go by
    lines.clear()
    port.write(b'!trace usb on\n')
    port.flush()
    samples, events = [], []
    first_at = None
    while True:
        now = time.monotonic()
        if first_at is None and now - began > args.start_timeout:
            break
        if first_at is not None and now - first_at >= args.observe:
            break
        for line in lines.poll():
            record = parse_line(line)
            if record:
                if first_at is None:
                    first_at = time.monotonic()
                record['cycle'] = cycle
                samples.append(record)
                continue
            event = parse_event(line)
            if event and first_at is not None:
                event['t'] = round(time.monotonic() - first_at, 3)
                events.append(event)
    result = dict(cycle=cycle)
    if first_at is None:
        port.write(b'!status\n')
        port.flush()
        status = None
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline and status is None:
            for line in lines.poll():
                if line.startswith('{') and '"firmware"' in line:
                    try:
                        status = json.loads(line)
                    except ValueError:
                        pass
        result.update(outcome='no_start')
        if status:
            result['status'] = dict(
                driving=status['driving'], reason=status['reason'], last_stop=status['last_stop_reason'],
                pitch=status['pitch'], roll=status['roll'],
                ages={str(f['id']): (None if f['age'] > 10 ** 6 else f['age']) for f in status['feedback']})
    else:
        expected = int(args.observe * 55 * 0.6)
        outcome, silent, reason, ages = classify(samples, events, expected)
        result.update(outcome=outcome, failed_servo=silent, stop_reason=reason, samples=len(samples),
                      start_after_reset_s=round(first_at - began, 2), max_age_ms=ages, events=events,
                      uptime_ms_at_start=samples[0]['ms'])
        result.update(start_details(samples))
        if outcome == 'fail' and not events:
            result['note'] = 'trace stopped or a servo went stale inside the window'
    port.write(b'!stop\n')
    port.flush()
    quiet = time.monotonic() + .5
    while time.monotonic() < quiet:
        lines.poll()
    port.write(b'!trace usb off\n')
    port.flush()
    return result, samples


def summarise(results):
    ok = [r for r in results if r['outcome'] == 'ok']
    fail = [r for r in results if r['outcome'] == 'fail']
    nostart = [r for r in results if r['outcome'] == 'no_start']
    lines = [f"cycles {len(results)}: ok {len(ok)}, fail {len(fail)}, no start {len(nostart)}"]
    servos = {}
    for r in fail:
        servos[r.get('failed_servo') or '?'] = servos.get(r.get('failed_servo') or '?', 0) + 1
    if fail:
        lines.append('failures by servo: ' + ', '.join(f'{k}: {v}' for k, v in sorted(servos.items())))
    lines.append('')
    lines.append('cycle  outcome   start(s)  samples  max age 11/21/12/22 (ms)   load12/22 peak  pos12/22 span  vbat adc first/min  note')
    for r in results:
        if r['outcome'] == 'no_start':
            status = r.get('status', {})
            ages = status.get('ages', {})
            lines.append(f"{r['cycle']:5d}  no_start  {'-':>8}  {'-':>7}  servo ages now {ages}   reason={status.get('reason')} last_stop={status.get('last_stop')}")
            continue
        a = r['max_age_ms']
        lines.append(f"{r['cycle']:5d}  {r['outcome']:8s}  {r['start_after_reset_s']:8.1f}  {r['samples']:7d}  {a['11']:4d}/{a['21']:4d}/{a['12']:4d}/{a['22']:4d}"
                     f"            {r.get('load12_peak', 0):4d}/{r.get('load22_peak', 0):4d}        {r.get('pos12_span', 0):3d}/{r.get('pos22_span', 0):3d}"
                     f"        {r.get('vbat_adc_first', 0)}/{r.get('vbat_adc_min', 0)}"
                     f"   {('servo ' + r['failed_servo'] + ' silent ') if r.get('failed_servo') else ''}{r.get('stop_reason') or ''}")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--cycles', type=int, default=20)
    parser.add_argument('--observe', type=float, default=2.5, help='seconds to watch each start')
    parser.add_argument('--start-timeout', type=float, default=20., help='give up on a start after this many seconds')
    parser.add_argument('--boot-wait', type=float, default=2.5, help='seconds to let the board boot after the reset')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--cue', action='store_true')
    args = parser.parse_args()
    out = args.out or ROOT / 'logs' / 'start-cycles' / datetime.now().strftime('%Y%m%d-%H%M%S')
    out.mkdir(parents=True, exist_ok=True)

    import serial
    port = serial.Serial(port=None, baudrate=115200, timeout=.1)
    port.rts = False
    port.port = args.port
    port.open()
    port.dtr = False
    lines = Lines(port)
    results = []

    def stop_now(signum, frame):            # also when started in the background, where SIGINT is ignored by default
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_now)
    (out / 'samples.jsonl').write_text('')
    if args.cue:
        say('开始测试，托直立，不要动')
    try:
        for cycle in range(1, args.cycles + 1):
            if args.cue:
                tick()
            result, samples = one_cycle(port, lines, args, cycle)
            results.append(result)
            print(f"cycle {cycle}: {result['outcome']}" + (f" (servo {result['failed_servo']} silent)" if result.get('failed_servo') else ''), flush=True)
            # everything is on disk after every cycle, so a stopped run loses nothing
            with (out / 'samples.jsonl').open('a') as handle:
                handle.write(''.join(json.dumps(r) + '\n' for r in samples))
            (out / 'cycles.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in results))
            (out / 'summary.txt').write_text('\n'.join(summarise(results)) + '\n')
    except KeyboardInterrupt:
        print('interrupted; what was recorded is saved', flush=True)
    finally:
        try:
            port.write(b'!trace usb off\n')
            port.flush()
        except Exception:
            pass
        port.close()
        if args.cue:
            say('测试结束')
    (out / 'cycles.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in results))
    text = '\n'.join(summarise(results)) + '\n'
    (out / 'summary.txt').write_text(text)
    print(text)
    print(f'saved to {out}')


if __name__ == '__main__':
    main()
