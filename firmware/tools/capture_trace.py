"""Capture the diagnostic BLE trace stream from the Rider2 RL-only firmware.

Usage: .venv-ble/bin/python tools/capture_trace.py OUT.jsonl [--wait-driving 120] [--record 20]
           [--stand-up [--arm-delay 3]] [--cue]
``--stand-up`` sends the phone's stand-up command (action 255 = ``!arm``) over BLE once the trace is on, so a robot that
was stopped can be started without opening the USB port (opening it resets this board).  The process needs macOS
Bluetooth permission: run it from Terminal/iTerm, not from an app without a Bluetooth usage description (it aborts).

The firmware exposes:
  write 0xb3 1/0 : enable/disable the 55 Hz trace stream
  notify payload : 0x54 + uint24 sequence + 1..2 x 28-byte samples
"""
import argparse
import asyncio
import json
import math
import struct
import subprocess
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

SERVICE = '0000fff0-0000-1000-8000-00805f9b34fb'
WRITE = '0000fff2-0000-1000-8000-00805f9b34fb'
NOTIFY = '0000fff1-0000-1000-8000-00805f9b34fb'
FIELDS = ('pitch', 'pitch_rate', 'roll', 'yaw_rate', 'cmd11', 'cmd21',
          'sp11', 'sp21', 'fsp11', 'fsp21', 'legA', 'legB', 'a0', 'a1')


def frame(kind, reg, payload):
    body = bytes([len(payload) + 8, kind, reg]) + payload
    return b'\x55\x00' + body + bytes([(~sum(body)) & 255, 0, 0xaa])


def cue(name, enabled):
    sound = Path('/System/Library/Sounds') / (name + '.aiff')
    if enabled and sound.exists():
        subprocess.Popen(['afplay', str(sound)])


def parse_samples(data):
    if len(data) < 4 or data[0] != 0x54:
        return []
    seq = int.from_bytes(data[1:4], 'little')
    payload = data[4:]
    rows = []
    for index in range(len(payload) // 28):
        values = struct.unpack('<14h', payload[index * 28:(index + 1) * 28])
        rows.append((seq + index,) + values)
    return rows


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--wait-driving', type=float, default=120.)
    parser.add_argument('--record', type=float, default=20.)
    parser.add_argument('--name', default='XGORider_', help='BLE name of your robot; only the first 8 characters (XGORider) are matched')
    parser.add_argument('--stand-up', action='store_true', help='send the stand-up command (action 255) once the trace is on')
    parser.add_argument('--arm-delay', type=float, default=0., help='seconds between the cue and the stand-up command')
    parser.add_argument('--cue', action='store_true', help='play macOS sounds when connected and when the recording ends')
    args = parser.parse_args()

    device = None
    for attempt in range(30):
        found = await BleakScanner.discover(timeout=5, return_adv=True, service_uuids=[SERVICE])
        matches = [(d, a) for d, a in found.values()
                   if (a.local_name or d.name or '').startswith(args.name[:8])]
        if matches:
            device = matches[0][0]
            break
        print(f'scan retry {attempt}', flush=True)
        await asyncio.sleep(2)
    if device is None:
        raise SystemExit('no Rider advertising')

    samples = []
    responses = asyncio.Queue()
    statuses = []

    async def read_status(cl):
        await cl.write_gatt_char(WRITE, frame(2, 0xb2, bytes([16])), response=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            try:
                data = await asyncio.wait_for(responses.get(), timeout=max(.05, deadline - time.monotonic()))
            except asyncio.TimeoutError:
                return None
            if len(data) >= 8 and data[0] == 0x55 and data[1] == 0 and data[3] == 2 and data[4] == 0xb2:
                p = data[5:-3]
                return dict(source=p[1], policy=p[2], last_stop=p[3],
                            calls=int.from_bytes(p[4:8], 'little'),
                            rl_ticks=int.from_bytes(p[12:16], 'little'))
        return None

    def on_notify(_, data):
        data = bytes(data)
        if data and data[0] == 0x54:
            samples.extend(parse_samples(data))
        else:
            responses.put_nowait(data)

    async with BleakClient(device, timeout=20) as client:
        await client.start_notify(NOTIFY, on_notify)
        await client.write_gatt_char(WRITE, frame(0, 0xb3, bytes([1])), response=True)
        print('connected; trace enabled', flush=True)
        cue('Ping', args.cue)
        if args.stand_up:
            # Same as the phone's stand-up button / USB !arm: select the policy and wait for an upright, still robot.
            # Over BLE no USB port has to be opened, so the board is not reset and the whole start is on the trace.
            await asyncio.sleep(args.arm_delay)
            await client.write_gatt_char(WRITE, frame(0, 0x3e, bytes([255])), response=True)
            print('stand-up (arm) sent: hold the robot upright and still', flush=True)
        began = time.monotonic()
        driving_at = None
        while time.monotonic() - began < args.wait_driving:
            status = await read_status(client)
            if status:
                statuses.append(dict(t=round(time.monotonic() - began, 2), **status))
                print('status', status, 'samples', len(samples), flush=True)
                if status['source'] == 2 or status['rl_ticks'] > 0:
                    driving_at = time.monotonic()
                    break
            await asyncio.sleep(.4)
        if driving_at:
            print('driving detected; recording', args.record, 's', flush=True)
            while time.monotonic() - driving_at < args.record:
                await asyncio.sleep(.5)
            cue('Glass', args.cue)
        else:
            # The BLE status read (0xb2) returns nothing on this firmware, so the wait simply runs its full length;
            # the trace stream itself is complete and is what gets saved.
            print('driving never detected', flush=True)
            cue('Glass', args.cue)
        await client.write_gatt_char(WRITE, frame(0, 0xb3, bytes([0])), response=True)
        final = await read_status(client)
        print('final', final, 'samples', len(samples), flush=True)
        try:
            await client.stop_notify(NOTIFY)
        except Exception:
            pass

    seen = set()
    unique = []
    for row in samples:
        if row[0] in seen:
            continue
        seen.add(row[0])
        unique.append(row)
    unique.sort(key=lambda r: r[0])
    records = [dict(seq=r[0], **{f: v for f, v in zip(FIELDS, r[1:])}) for r in unique]
    args.output.write_text('\n'.join(json.dumps(r) for r in records) + ('\n' if records else ''))
    print('wrote', len(records), 'samples to', args.output, flush=True)
    if records:
        pitch = [r['pitch'] / 100. for r in records]
        rate = [r['pitch_rate'] / 1000. for r in records]
        mean = sum(pitch) / len(pitch)
        crossings = sum(1 for a, b in zip(pitch, pitch[1:]) if (a - mean) * (b - mean) < 0)
        span = max(1, len(records) - 1)
        # Samples are ~18 ms apart because the firmware emits every third control tick.
        seconds = span * 0.018
        print('pitch mean/min/max/std deg', round(mean, 2), round(min(pitch), 2),
              round(max(pitch), 2), round(statistics_pstdev(pitch), 3), flush=True)
        print('pitch_rate std deg/s', round(statistics_pstdev(rate) * 180 / math.pi, 2), flush=True)
        print('approx zero-crossing frequency', round(crossings / 2 / seconds, 2), 'Hz over', round(seconds, 1), 's', flush=True)


def statistics_pstdev(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


if __name__ == '__main__':
    asyncio.run(main())
