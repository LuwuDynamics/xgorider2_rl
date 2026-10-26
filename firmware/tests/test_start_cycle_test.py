"""Reset-then-start test using a fake board; no physical port is opened."""
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from contextlib import redirect_stdout
from unittest import mock

ROOT = Path(os.path.abspath(__file__)).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import capture_trace_usb as capture
import start_cycle_test as cycles


def sample_line(seq, ms, **over):
    values = dict(seq=seq, ms=ms, pitch=100, pitch_rate=0, roll=0, yaw_rate=0, cmd11=5, cmd21=-5, sp11=0, sp21=0, fsp11=0, fsp21=0,
                  legA=591, legB=420, a0=5, a1=-5, load11=1, load21=1, pos11=100, pos21=900, acc_x=0, acc_y=-50, acc_z=981, flags=0,
                  age11=5, age21=6, age12=20, age22=22, dt_us=5000, load12=60, load22=55, pos12=590, pos22=420, vbat_adc=3300)
    values.update(over)
    return 'T,' + ','.join(str(values[name]) for name in capture.COLUMNS) + '\n'


class FakeBoard:
    """Behaviour per cycle: 'ok', 'fail12' (servo 12 goes silent on the first tick) or 'nostart'."""

    def __init__(self, script):
        self.script = list(script)
        self.reboots = 0
        self.written = []
        self.dtr = False
        self._rts = False
        self.port = None
        self.out = b''
        self.trace_on = False
        self.stopped = False
        self.boot_at = None
        self.count = 0
        self.mode = None
        self.t_start = None

    def open(self):
        pass

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        if self._rts and not value:                       # RTS pulse finished: the board reboots
            self.reboots += 1
            self.mode = self.script[self.reboots - 1]
            self.boot_at = time.monotonic()
            self.trace_on = False
            self.stopped = False
            self.count = 0
            self.t_start = None
        self._rts = value

    def reset_input_buffer(self):
        self.out = b''

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, data):
        self.written.append(data)
        text = data.strip()
        if text == b'!trace usb on':
            self.trace_on = True
            self.out += b'# trace seq,ms,...\n'
        elif text == b'!trace usb off':
            self.trace_on = False
        elif text == b'!stop':
            self.stopped = True
        elif text == b'!status':
            ages = [{'id': i, 'age': 20 if i != 12 else 4294967295, 'status': 0} for i in (11, 21, 12, 22)]
            self.out += (json.dumps(dict(firmware='Rider2-recovered-full', driving=False, reason='startup', last_stop_reason='actuator_feedback_missing_or_stale',
                                         pitch=-30.0, roll=0.3, feedback=ages, policy={'mode': 'off'})) + '\n').encode()

    def read(self, size):
        time.sleep(.01)
        if self.boot_at is not None and self.trace_on and not self.stopped and self.mode in ('ok', 'fail12'):
            if self.t_start is None and time.monotonic() - self.boot_at > .08:
                self.t_start = time.monotonic()
            if self.t_start is not None:
                due = int((time.monotonic() - self.t_start) * 55)
                limit = 16 if self.mode == 'fail12' else 10 ** 6
                while self.count < due and self.count < limit:
                    k = self.count
                    over = dict(age12=14 + 15 * k) if self.mode == 'fail12' else {}
                    self.out += sample_line(k, 5000 + k * 18, load12=60 + k, pos12=590 - k, vbat_adc=3300 - (50 if k == 3 else 0), **over).encode()
                    self.count += 1
                    if self.mode == 'fail12' and self.count == limit:
                        self.out += b'# ev ms=5300 drv=0 reason=manual_stop last_stop=actuator_feedback_missing_or_stale lift=0 acting=0 trips=0 bus_to=14 bus_st=0 bus_len=0 bus_chk=0\n'
        data, self.out = self.out[:size], self.out[size:]
        return data


class CycleTests(unittest.TestCase):
    def run_cycles(self, script):
        board = FakeBoard(script)
        module = types.ModuleType('serial')
        module.Serial = lambda *a, **k: board
        out = Path(tempfile.mkdtemp()) / 'run'
        buffer = io.StringIO()
        argv = ['start_cycle_test.py', '--port', 'fake', '--cycles', str(len(script)), '--observe', '0.5', '--start-timeout', '0.7',
                '--boot-wait', '0.05', '--out', str(out)]
        with mock.patch.dict(sys.modules, {'serial': module}), mock.patch.object(sys, 'argv', argv), redirect_stdout(buffer):
            cycles.main()
        records = [json.loads(line) for line in (out / 'cycles.jsonl').read_text().splitlines()]
        return board, out, records, buffer.getvalue()

    def test_ok_fail_and_no_start_are_told_apart(self):
        board, out, records, text = self.run_cycles(['ok', 'fail12', 'nostart'])
        self.assertEqual([r['outcome'] for r in records], ['ok', 'fail', 'no_start'])
        self.assertIsNone(records[0]['failed_servo'])
        self.assertEqual(records[1]['failed_servo'], '12')
        self.assertEqual(records[1]['stop_reason'], 'actuator_feedback_missing_or_stale')
        self.assertIn('12', records[2]['status']['ages'])
        self.assertIsNone(records[2]['status']['ages']['12'])          # the silent servo shows as "no answer"
        self.assertIn('ok 1, fail 1, no start 1', (out / 'summary.txt').read_text())
        self.assertIn('failures by servo: 12: 1', (out / 'summary.txt').read_text())

    def test_ok_start_keeps_what_the_legs_and_the_supply_did(self):
        _, _, records, _ = self.run_cycles(['ok'])
        r = records[0]
        self.assertEqual(r['outcome'], 'ok')
        self.assertGreater(r['samples'], 12)
        self.assertGreaterEqual(r['load12_peak'], 60)
        self.assertGreater(r['pos12_span'], 0)
        self.assertEqual(r['vbat_adc_first'], 3300)
        self.assertEqual(r['vbat_adc_min'], 3250)                       # the dip at the 4th sample
        self.assertLess(max(r['max_age_ms'].values()), 100)

    def test_a_stopped_run_keeps_everything_recorded_so_far(self):
        class Stopped(FakeBoard):
            @property
            def rts(self):
                return self._rts

            @rts.setter
            def rts(self, value):
                if self._rts and not value and self.reboots >= 1:
                    raise KeyboardInterrupt            # the user stops the run during the second cycle
                FakeBoard.rts.fset(self, value)

        board = Stopped(['ok', 'ok', 'ok'])
        module = types.ModuleType('serial')
        module.Serial = lambda *a, **k: board
        out = Path(tempfile.mkdtemp()) / 'run'
        argv = ['start_cycle_test.py', '--port', 'fake', '--cycles', '3', '--observe', '0.4', '--start-timeout', '0.7', '--boot-wait', '0.05',
                '--out', str(out)]
        with mock.patch.dict(sys.modules, {'serial': module}), mock.patch.object(sys, 'argv', argv), redirect_stdout(io.StringIO()):
            cycles.main()
        records = [json.loads(line) for line in (out / 'cycles.jsonl').read_text().splitlines()]
        samples = [json.loads(line) for line in (out / 'samples.jsonl').read_text().splitlines()]
        self.assertEqual([r['cycle'] for r in records], [1])
        self.assertTrue(samples and {s['cycle'] for s in samples} == {1})
        self.assertIn('cycles 1: ok 1', (out / 'summary.txt').read_text())

    def test_every_cycle_reboots_once_and_only_known_commands_are_sent(self):
        board, out, _, _ = self.run_cycles(['ok', 'ok'])
        self.assertEqual(board.reboots, 2)
        sent = [w.strip() for w in board.written]
        self.assertTrue(set(sent) <= {b'!trace usb on', b'!trace usb off', b'!stop', b'!status'}, sent)
        self.assertEqual(sent.count(b'!stop'), 2)
        samples = [json.loads(line) for line in (out / 'samples.jsonl').read_text().splitlines()]
        self.assertEqual({s['cycle'] for s in samples}, {1, 2})


if __name__ == '__main__':
    unittest.main()
