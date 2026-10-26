"""One command to keep the evidence after something odd happened (a fall, the wheels going limp, a stop).

Usage: python tools/incident_snapshot.py --port /dev/cu.usbserial-XXXX [--out DIR] [--note "what happened"]

The firmware keeps the last ~18 s of the 55 Hz trace in RAM (only while the policy drives) and the reason for the last
stop; both are lost at the next reset, and about 18 s of further driving overwrites the trace.  So run this right
after the incident.  It attaches without resetting the board (RTS is released before DTR), reads ``!status`` and
``!trace dump``, and saves them under ``logs/incidents/<time>/``:

* ``status.json`` -- driving state, stop reason and last stop reason, lift guard (enabled, trips), the free-fall guard (hold, trips, lowest |a|, longest low stretch), the IMU read timing, servo freshness,
  bus error counters, battery, the policy block;
* ``trace.jsonl`` -- the dumped samples (same keys as tools/capture_trace_usb.py, incl. ``flags``: bit0 = the lift
  guard was holding the wheels at zero, bit1 = its detector had fired, bit2 = wheel speed over the limit, bit3 = a dead-zone
  preload had been learned and was on, bit4/bit5 = its relay on the positive/negative side; cmd11 - a0 is the preload, bit6 = acceleration below the free-fall
  limit, bit7 = free fall detected);
* ``note.txt`` -- your note, if given.

It sends only these two read-only commands; it never moves the robot.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_trace_usb import parse_line  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def read_until(port, seconds, stop=None):
    deadline = time.monotonic() + seconds
    text = ''
    while time.monotonic() < deadline:
        text += port.read(8192).decode('utf-8', errors='replace')
        if stop and stop in text:
            break
    return text


def find_status(text):
    for line in text.splitlines():
        if line.startswith('{') and '"firmware"' in line:
            try:
                return json.loads(line)
            except ValueError:
                continue
    return None


def summarise_status(status):
    lift = status.get('lift_guard', {})
    servos = ', '.join(f"{f['id']}: {'no answer' if f['age'] > 10 ** 6 else str(f['age']) + ' ms'}/status {f['status']}" for f in status['feedback'])
    lines = [
        f"build: {status.get('build', '(older firmware)')}",
        f"firmware uptime {status['ms'] / 1000:.0f} s   driving={status['driving']}   reason={status['reason']}   last stop={status['last_stop_reason']}",
        f"pose now: pitch {status['pitch']:.1f} deg, roll {status['roll']:.1f} deg   battery {status['battery_voltage']} V ({status['battery_percent']}%)",
        f"lift guard: enabled={lift.get('enabled')} active={lift.get('active')} trips since the last stop={lift.get('trips')}",
        f"servos (age/status): {servos}",
        f"bus: timeouts {status['bus_timeouts']}, status errors {status['bus_status']}, length {status['bus_length_errors']}, "
        f"checksum {status['bus_checksum']}; control period max {status['control_max_us']} us",
    ]
    fall = status.get('fall_guard')
    if fall:
        lines.append(f"fall guard: enabled={fall.get('enabled')} hold {fall.get('freefall_ms')} ms, trips {fall.get('trips')} (deep, below 0.061 g: {fall.get('deep_trips', '-')}), "
                     f"lowest |a| since boot {fall.get('min_g', '-')} g, longest stretch below {fall.get('freefall_g')} g {fall.get('longest_low_ms', '-')} ms")
    imu = status.get('imu_timing')
    if imu:
        lines.append(f"imu: read interval min/mean/max {imu.get('min_us')}/{imu.get('mean_us')}/{imu.get('max_us')} us, longest read+fusion {imu.get('max_work_us')} us, "
                     f"read failures {imu.get('read_failures')}, blank {imu.get('blank')}, identical {imu.get('identical')}")
    return lines


def summarise_trace(rows):
    if not rows:
        return ['trace: empty (the policy has not driven since the last reset, or it was overwritten)']
    seconds = (rows[-1]['ms'] - rows[0]['ms']) / 1000.
    speed = max(max(abs(r['fsp11']), abs(r['fsp21'])) for r in rows)
    acting = sum(1 for r in rows if r['flags'] & 1)
    gaps = sum(1 for a, b in zip(rows, rows[1:]) if b['seq'] - a['seq'] > 1)
    return [f"trace: {len(rows)} samples, {seconds:.1f} s of firmware time, {gaps} gaps, wheel speed peak {speed}, "
            f"lift guard holding the wheels in {acting} samples ({acting * 0.018:.1f} s)"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', required=True)
    parser.add_argument('--out', type=Path, help='output folder (default logs/incidents/<time>)')
    parser.add_argument('--note', default='')
    parser.add_argument('--dump-seconds', type=float, default=25., help='how long to wait for the dump to finish')
    args = parser.parse_args()
    out = args.out or ROOT / 'logs' / 'incidents' / datetime.now().strftime('%Y%m%d-%H%M%S')
    out.mkdir(parents=True, exist_ok=True)

    import serial
    port = serial.Serial(port=None, baudrate=115200, timeout=.1)
    port.rts = False            # RTS (EN) released before the open and DTR after it: attaching never resets the board
    port.port = args.port
    port.open()
    port.dtr = False
    try:
        time.sleep(.3)
        status = None
        for _ in range(3):
            port.reset_input_buffer()
            port.write(b'!status\n')
            port.flush()
            status = find_status(read_until(port, 2.0, '}}\n'))
            if status:
                break
        port.reset_input_buffer()
        port.write(b'!trace dump\n')
        port.flush()
        text = read_until(port, args.dump_seconds, '# end')
    finally:
        port.close()

    rows = [r for r in (parse_line(line.strip()) for line in text.splitlines()) if r]
    if status:
        (out / 'status.json').write_text(json.dumps(status, indent=1))
    (out / 'trace.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    if args.note:
        (out / 'note.txt').write_text(args.note + '\n')
    print(f'saved to {out}')
    if status:
        print('\n'.join(summarise_status(status)))
    else:
        print('no status answer: is the firmware running and the right port selected?')
    print('\n'.join(summarise_trace(rows)))
    if '# end' not in text:
        print('warning: the dump did not finish in time; the trace may be cut short (raise --dump-seconds)')


if __name__ == '__main__':
    main()
