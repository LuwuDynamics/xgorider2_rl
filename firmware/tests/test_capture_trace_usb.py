"""USB trace capture tests using a fake serial module; no physical port is opened."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

ROOT = Path(os.path.abspath(__file__)).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import capture_trace_usb as capture

LINE = 'T,70000,123456,725,-980,181,-2053,104,-180,5,-1,8,0,592,420,103,-180,-12,1030,1023,0,-981,5,32767,5,3,-1,32767,26,6012,140,-7,562,449,3150'


class FakeFirmware:
    """Answers '!trace usb on' with a header and 55 Hz text lines, as src/main.cpp does."""

    def __init__(self, damaged_at=20, silent=False):
        self.silent = silent
        self.dtr = self.rts = False
        self.port = None
        self.written = []
        self.out = b''
        self.started = None
        self.count = 0
        self.damaged_at = damaged_at

    def open(self):
        pass

    def reset_input_buffer(self):
        self.out = b''

    def write(self, data):
        self.written.append(data)
        if b'!trace usb on' in data:
            self.started = time.monotonic()
            self.out += b'# trace seq,ms,pitch_cdeg\n# ev ms=1 drv=1 reason=none last_stop=startup lift=0 acting=0 trips=0\n'

    def flush(self):
        pass

    def close(self):
        pass

    def read(self, size):
        time.sleep(.02)
        if self.started is not None and not self.silent:
            due = int((time.monotonic() - self.started) * 55)
            while self.count < due:
                if self.count == self.damaged_at:
                    self.out += b'T,20,1,2,3\n'
                self.out += (f'T,{self.count},{1000 + self.count * 18},{self.count},0,0,0,5,-5,0,0,0,0,592,420,5,-5,'
                             '10,-10,100,900,0,0,981,0,3,3,4,4,6000,50,60,562,449,3150\n').encode()
                self.count += 1
        data, self.out = self.out[:size], self.out[size:]
        return data


class ParseTests(unittest.TestCase):
    def test_full_line_becomes_a_named_record(self):
        record = capture.parse_line(LINE)
        self.assertEqual(len(record), len(capture.COLUMNS))
        self.assertEqual((record['seq'], record['ms'], record['pitch'], record['pitch_rate']), (70000, 123456, 725, -980))
        self.assertEqual((record['load11'], record['load21'], record['pos11'], record['pos21']), (-12, 1030, 1023, 0))
        self.assertEqual((record['acc_x'], record['acc_z'], record['flags']), (-981, 32767, 5))
        self.assertEqual((record['age11'], record['age21'], record['age12'], record['age22'], record['dt_us']), (3, -1, 32767, 26, 6012))
        self.assertEqual((record['load12'], record['load22'], record['pos12'], record['pos22'], record['vbat_adc']), (140, -7, 562, 449, 3150))

    def test_everything_else_is_ignored(self):
        for text in ('# trace seq,ms', '{"firmware":"Rider2-recovered-full"}', 'T,1,2,3', LINE.replace('725', 'x'), '', 'stop requested'):
            self.assertIsNone(capture.parse_line(text), text)

    def test_columns_match_the_ble_trace_names_used_by_the_report_tools(self):
        for name in ('pitch', 'pitch_rate', 'roll', 'yaw_rate', 'cmd11', 'cmd21', 'sp11', 'sp21', 'fsp11', 'fsp21', 'legA', 'legB', 'a0', 'a1'):
            self.assertIn(name, capture.COLUMNS)


class CaptureTests(unittest.TestCase):
    def run_capture(self, *extra):
        fake = FakeFirmware()
        self.last_output = None
        module = types.ModuleType('serial')
        module.Serial = lambda *a, **k: fake
        out = Path(tempfile.mkdtemp()) / 'trace.jsonl'
        with mock.patch.dict(sys.modules, {'serial': module}), \
                mock.patch.object(sys, 'argv', ['capture_trace_usb.py', '--port', 'fake', str(out), '--seconds', '2', *extra]):
            capture.main()
        self.last_output = out
        return fake, [json.loads(line) for line in out.read_text().splitlines()]

    def test_records_the_stream_skips_damaged_lines_and_switches_it_off(self):
        fake, rows = self.run_capture()
        self.assertTrue(95 <= len(rows) <= 112, len(rows))
        self.assertEqual([r['seq'] for r in rows[:3]], [0, 1, 2])
        self.assertTrue(all(r['label'] == 'start' and 'host_t' in r for r in rows))
        self.assertEqual(fake.written[0].strip(), b'!trace usb on')
        self.assertEqual(fake.written[-1].strip(), b'!trace usb off')

    def test_liftguard_option_is_sent_first_and_only_when_asked(self):
        fake, _ = self.run_capture('--liftguard', 'on')
        self.assertEqual([w.strip() for w in fake.written[:2]], [b'!liftguard on', b'!trace usb on'])
        plain, _ = self.run_capture()
        self.assertNotIn(b'!liftguard', b''.join(plain.written))

    def test_send_option_forwards_settings_commands_before_the_stream_and_refuses_anything_else(self):
        fake, _ = self.run_capture('--send', '!dzcomp off', '--send', '!dzcomp reset')
        self.assertEqual([w.strip() for w in fake.written[:3]], [b'!dzcomp off', b'!dzcomp reset', b'!trace usb on'])
        for bad in ('!arm', '!move 255 128', '!stop', 'dzcomp off', 'fallguard ms 40'):
            with self.assertRaises(SystemExit):
                self.run_capture('--send', bad)
        plain, _ = self.run_capture()
        self.assertNotIn(b'!dzcomp', b''.join(plain.written))
        # the free-fall guard's hold time is a setting too (hand-drop test: --send "!fallguard ms 40")
        fake, _ = self.run_capture('--send', '!fallguard ms 40')
        self.assertEqual([w.strip() for w in fake.written[:2]], [b'!fallguard ms 40', b'!trace usb on'])

    def test_stop_and_start_events_go_to_a_side_file(self):
        _, rows = self.run_capture()
        events = [json.loads(line) for line in self.last_output.with_suffix('.events.jsonl').read_text().splitlines()]
        self.assertEqual(len(events), 1)
        self.assertTrue(events[0]['text'].startswith('ms=1 drv=1 reason=none'))
        self.assertTrue(all('seq' in r for r in rows))        # event lines never leak into the sample records

    def test_arm_option_sends_arm_right_after_enabling_the_stream(self):
        fake, rows = self.run_capture('--arm')
        self.assertEqual([w.strip() for w in fake.written[:2]], [b'!trace usb on', b'!arm'])
        self.assertTrue(rows)
        plain, _ = self.run_capture()
        self.assertNotIn(b'!arm', b''.join(plain.written))

    def test_recording_time_counts_from_the_first_sample_and_gives_up_if_none_comes(self):
        silent = FakeFirmware(silent=True)
        module = types.ModuleType('serial')
        module.Serial = lambda *a, **k: silent
        out = Path(tempfile.mkdtemp()) / 'trace.jsonl'
        began = time.monotonic()
        with mock.patch.dict(sys.modules, {'serial': module}), \
                mock.patch.object(sys, 'argv', ['capture_trace_usb.py', '--port', 'fake', str(out), '--seconds', '30', '--wait-start', '1']):
            capture.main()
        self.assertLess(time.monotonic() - began, 6)          # did not wait for the 30 s recording
        self.assertEqual(out.read_text(), '')
        self.assertEqual(silent.written[-1].strip(), b'!trace usb off')

    def test_protocol_clock_starts_at_the_first_sample_and_records_are_labelled(self):
        spoken = []
        with mock.patch.object(capture, 'speak', side_effect=spoken.append), \
                mock.patch.dict(capture.PROTOCOLS, {'quick': [(0, 'go', 'one'), (1, 'next', 'two')]}):
            module_choices = capture.PROTOCOLS
            fake, rows = self.run_capture('--protocol', 'quick')
        labels = [r['label'] for r in rows]
        self.assertEqual(labels[0], 'one')
        self.assertIn('two', labels)
        self.assertIn('go', spoken)
        self.assertIsNotNone(module_choices)
        self.assertTrue(all('t' in r and r['t'] >= 0 for r in rows))

    def test_never_sends_a_motion_command(self):
        fake, _ = self.run_capture()
        sent = b''.join(fake.written)
        for forbidden in (b'!arm', b'!move', b'!policy', b'!action', b'!neutral'):
            self.assertNotIn(forbidden, sent)

    def test_reset_option_pulses_rts_only_when_asked(self):
        class Recording(FakeFirmware):
            def __init__(self):
                self.levels = []
                super().__init__()

            @property
            def rts(self):
                return self.levels[-1] if self.levels else False

            @rts.setter
            def rts(self, value):
                self.levels.append(value)

        for extra, expect_pulse in (((), False), (('--reset',), True)):
            fake = Recording()
            module = types.ModuleType('serial')
            module.Serial = lambda *a, **k: fake
            out = Path(tempfile.mkdtemp()) / 'trace.jsonl'
            with mock.patch.dict(sys.modules, {'serial': module}), \
                    mock.patch.object(sys, 'argv', ['capture_trace_usb.py', '--port', 'fake', str(out), '--seconds', '0.3', *extra]):
                capture.main()
            self.assertEqual(True in fake.levels, expect_pulse, extra)
            self.assertFalse(fake.levels[-1])   # the line is always released again


if __name__ == '__main__':
    unittest.main()
