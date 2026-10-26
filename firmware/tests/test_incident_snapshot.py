"""Incident snapshot tests using a fake serial module; no physical port is opened."""
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock
from contextlib import redirect_stdout

ROOT = Path(os.path.abspath(__file__)).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import incident_snapshot as snapshot

STATUS = {
    'firmware': 'Rider2-recovered-full', 'build': 'test-build', 'ms': 123456, 'driving': False, 'reason': 'manual_stop', 'last_stop_reason': 'tilt',
    'pitch': -44.5, 'roll': 0.4, 'battery_voltage': 8.1, 'battery_percent': 97.0,
    'lift_guard': {'enabled': True, 'active': False, 'acting': False, 'trips': 3},
    'feedback': [{'id': 11, 'age': 5, 'status': 0}, {'id': 21, 'age': 6, 'status': 0}, {'id': 12, 'age': 4294967295, 'status': 0}, {'id': 22, 'age': 8, 'status': 0}],
    'bus_timeouts': 12, 'bus_status': 1, 'bus_length_errors': 2, 'bus_checksum': 0, 'control_max_us': 7100,
    'policy': {'mode': 'off'},
}


class SummaryTests(unittest.TestCase):
    def test_fall_guard_and_imu_timing_lines_appear_only_when_the_firmware_reports_them(self):
        plain = snapshot.summarise_status(STATUS)
        self.assertFalse(any(line.startswith(('fall guard', 'imu:')) for line in plain))
        status = dict(STATUS, fall_guard={'enabled': True, 'trips': 1, 'deep_trips': 1, 'freefall_g': 0.5, 'freefall_ms': 60, 'min_g': 0.012, 'longest_low_ms': 143},
                      imu_timing={'count': 1000, 'min_us': 2100, 'mean_us': 2900, 'max_us': 7900, 'max_work_us': 640, 'read_failures': 0, 'blank': 0, 'identical': 3})
        text = '\n'.join(snapshot.summarise_status(status))
        self.assertIn('hold 60 ms, trips 1 (deep, below 0.061 g: 1)', text)
        self.assertIn('lowest |a| since boot 0.012 g', text)
        self.assertIn('longest stretch below 0.5 g 143 ms', text)
        self.assertIn('read interval min/mean/max 2100/2900/7900 us, longest read+fusion 640 us', text)


def sample_line(seq, speed=0, flags=0):
    fields = [seq, 1000 + seq * 18, 100, 0, 0, 0, 5, -5, speed, speed, speed, speed, 592, 420, 5, -5, 10, 10, 100, 900, 0, -50, 981, flags, 5, 5, 20, 20, 5000, 120, 130, 562, 449, 3150]
    return 'T,' + ','.join(str(f) for f in fields) + '\n'


class FakeRobot:
    def __init__(self, dump_lines=40, finish=True):
        self.dtr = self.rts = True
        self.port = None
        self.written = []
        self.out = b''
        self.state = []
        self.dump_lines = dump_lines
        self.finish = finish

    def open(self):
        self.state.append(('open', self.dtr, self.rts))

    def reset_input_buffer(self):
        self.out = b''

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, data):
        self.written.append(data)
        if data.strip() == b'!status':
            self.out += (json.dumps(STATUS) + '\n').encode()
        elif data.strip() == b'!trace dump':
            self.out += b'# trace seq,ms,...\n'
            for i in range(self.dump_lines):
                self.out += sample_line(i, speed=450 if i == 10 else 3, flags=1 if 10 <= i < 20 else 0).encode()
            if self.finish:
                self.out += f'# end {self.dump_lines}\n'.encode()

    def read(self, size):
        data, self.out = self.out[:size], self.out[size:]
        return data


class SnapshotTests(unittest.TestCase):
    def run_snapshot(self, robot, *extra):
        module = types.ModuleType('serial')
        module.Serial = lambda *a, **k: robot
        out = Path(tempfile.mkdtemp()) / 'incident'
        buffer = io.StringIO()
        with mock.patch.dict(sys.modules, {'serial': module}), \
                mock.patch.object(sys, 'argv', ['incident_snapshot.py', '--port', 'fake', '--out', str(out), '--dump-seconds', '1', *extra]), \
                redirect_stdout(buffer):
            snapshot.main()
        return out, buffer.getvalue()

    def test_saves_status_trace_and_note_and_summarises_them(self):
        robot = FakeRobot()
        out, text = self.run_snapshot(robot, '--note', 'fell over after the second lift')
        self.assertEqual(json.loads((out / 'status.json').read_text())['last_stop_reason'], 'tilt')
        rows = [json.loads(line) for line in (out / 'trace.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows), 40)
        self.assertEqual(rows[10]['fsp11'], 450)
        self.assertEqual((out / 'note.txt').read_text().strip(), 'fell over after the second lift')
        self.assertIn('last stop=tilt', text)
        self.assertIn('build: test-build', text)
        self.assertIn('trips since the last stop=3', text)
        self.assertIn('12: no answer', text)             # the silent servo is called out
        self.assertIn('wheel speed peak 450', text)
        self.assertIn('lift guard holding the wheels in 10 samples', text)

    def test_only_the_two_read_only_commands_are_sent_and_attaching_does_not_pulse_reset(self):
        robot = FakeRobot()
        self.run_snapshot(robot)
        sent = [w.strip() for w in robot.written]
        self.assertTrue(set(sent) <= {b'!status', b'!trace dump'}, sent)
        self.assertIn(b'!trace dump', sent)
        # RTS was already released when the port opened, DTR is let go only afterwards.
        self.assertEqual(robot.state[0], ('open', True, False))
        self.assertFalse(robot.dtr)
        self.assertFalse(robot.rts)

    def test_an_unfinished_dump_is_reported(self):
        out, text = self.run_snapshot(FakeRobot(finish=False))
        self.assertIn('did not finish', text)
        self.assertTrue((out / 'trace.jsonl').exists())

    def test_empty_ring_is_explained(self):
        out, text = self.run_snapshot(FakeRobot(dump_lines=0))
        self.assertIn('trace: empty', text)


if __name__ == '__main__':
    unittest.main()
