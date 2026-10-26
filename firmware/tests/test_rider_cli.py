"""PC console tests using a fake serial module; no physical port is opened."""
import io
import json
import math
import os
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

ROOT = Path(os.path.abspath(__file__)).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import rider_cli


class VirtualClock:
    def __init__(self):
        self.now = 0.0
        self.interrupt_next_sleep = False

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        if self.interrupt_next_sleep:
            self.interrupt_next_sleep = False
            raise KeyboardInterrupt()
        self.now += seconds


class SerialDouble:
    def __init__(self, clock, responses, **kwargs):
        self.clock = clock
        self.constructor = kwargs
        self.responses = list(responses)
        self.writes = []
        self.port = kwargs['port']
        self.dtr = self.rts = True
        self.open_state = None
        self.closed = False
        self.flush_count = 0
        self.fail_write = None

    def open(self):
        self.open_state = (self.port, self.dtr, self.rts)

    def write(self, data):
        self.writes.append(data)
        if self.fail_write == data:
            self.fail_write = None
            raise OSError('Simulated USB write failure')
        return len(data)

    def flush(self):
        self.flush_count += 1

    def readline(self):
        self.clock.now += 0.01
        return self.responses.pop(0) if self.responses else b''

    def close(self):
        self.closed = True


def status_line(driving=True, firmware='Rider2-recovered-full', manual_stop=False):
    return (json.dumps(dict(firmware=firmware, driving=driving, manual_stop=manual_stop)) + '\n').encode()


class PercentMapping(unittest.TestCase):
    def test_neutral_full_scale_and_intermediate_values(self):
        for percent, expected in [(-100, 0), (-99, 1), (-50, 64), (-0.5, 127),
                                  (-0.0, 128), (0, 128), (0.5, 129), (50, 192),
                                  (99, 254), (100, 255)]:
            with self.subTest(percent=percent):
                self.assertEqual(rider_cli.percent_byte(percent), expected)

    def test_monotonic_bounded_mapping_over_the_command_range(self):
        values = [rider_cli.percent_byte(index / 10) for index in range(-1000, 1001)]
        self.assertEqual(values, sorted(values))
        self.assertTrue(all(isinstance(value, int) and 0 <= value <= 255 for value in values))
        self.assertEqual(set(values), set(range(256)))

    def test_nonfinite_and_out_of_range_rejected(self):
        for value in (-100.001, 100.001, -101, 101, math.inf, -math.inf, math.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                rider_cli.percent_byte(value)


class ConsoleProtocol(unittest.TestCase):
    def setUp(self):
        self.clock = VirtualClock()
        self.responses = [status_line(), status_line()]
        self.ports = []
        self.fail_write = None

        def fake_serial(**kwargs):
            port = SerialDouble(self.clock, self.responses, **kwargs)
            port.fail_write = self.fail_write
            self.ports.append(port)
            return port

        # Replace the import itself, so a missing/installed pyserial cannot touch hardware.
        patches = [mock.patch.dict(sys.modules, {'serial': types.SimpleNamespace(Serial=fake_serial)}),
                   mock.patch.object(rider_cli, 'time', self.clock),
                   mock.patch('sys.stdout', new_callable=io.StringIO),
                   mock.patch('sys.stderr', new_callable=io.StringIO)]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def invoke(self, arguments):
        with mock.patch.object(sys, 'argv', ['rider_cli.py', '--port', 'FAKE_PORT_NEVER_OPEN'] + arguments):
            rider_cli.main()

    def test_serial_open_disables_reset_lines_before_opening(self):
        session = rider_cli.Session('FAKE_PORT_NEVER_OPEN')
        port = self.ports[-1]
        self.assertEqual(port.constructor, dict(port=None, baudrate=115200, timeout=.2))
        # RTS is released before the open (DTR keeps the OS default until afterwards), then both lines end up released.
        self.assertEqual(port.open_state, ('FAKE_PORT_NEVER_OPEN', True, False))
        self.assertFalse(port.dtr)
        self.assertFalse(port.rts)
        session.close()
        self.assertTrue(port.closed)

    def test_send_accepts_one_explicit_command_and_rejects_line_injection(self):
        session = rider_cli.Session('FAKE_PORT_NEVER_OPEN')
        self.addCleanup(session.close)
        for text in ('status', '', '!stop\n!arm', '!stop\r', '\n!stop'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                session.send(text)
        self.assertEqual(self.ports[-1].writes, [])
        session.send('!stop')
        self.assertEqual(self.ports[-1].writes, [b'!stop\n'])
        self.assertEqual(self.ports[-1].flush_count, 1)

    def test_handshake_ignores_noise_and_other_firmware(self):
        self.responses = [b'boot output\n', b'\xff\xfe\n', b'[]\n', b'null\n',
                          status_line(firmware='Rider2-bringup'), status_line(driving=False)]
        session = rider_cli.Session('FAKE_PORT_NEVER_OPEN')
        self.addCleanup(session.close)
        self.assertFalse(session.status()['driving'])
        self.assertEqual(self.ports[-1].writes, [b'!status\n'])

    def test_unrecognized_firmware_never_receives_arm(self):
        self.responses = [status_line(firmware='Rider2-bringup')]
        with self.assertRaises(RuntimeError):
            self.invoke(['arm'])
        self.assertEqual(self.ports[-1].writes, [b'!status\n'])
        self.assertTrue(self.ports[-1].closed)

    def test_mutations_require_handshake_and_status_followup(self):
        commands = [(['arm'], b'!arm\n'), (['neutral'], b'!neutral\n'),
                    (['trim'], b'!trim\n'), (['cal', 'save'], b'!cal save\n'),
                    (['height', '0'], b'!height 0\n'), (['roll', '255'], b'!roll 255\n'),
                    (['name', 'R2'], b'!name R2\n'), (['led', '3', '0', '128', '255'], b'!rgb 3 0 128 255\n')]
        for args, wire_command in commands:
            with self.subTest(command=args):
                self.invoke(args)
                self.assertEqual(self.ports[-1].writes, [b'!status\n', wire_command, b'!status\n'])
                self.assertTrue(self.ports[-1].closed)

    def policy_line(self, mode='off', trained=False, driving=False, manual_stop=True):
        return (json.dumps(dict(firmware='Rider2-recovered-full',driving=driving,manual_stop=manual_stop,
            policy=dict(mode=mode,trained=trained,model_valid=True)))+'\n').encode()

    def test_policy_shadow_is_explicit_and_verified(self):
        self.responses=[self.policy_line(),self.policy_line('shadow')]
        self.invoke(['policy','shadow'])
        self.assertEqual(self.ports[-1].writes,[b'!status\n',b'!policy shadow\n',b'!status\n'])
        self.assertTrue(self.ports[-1].closed)

    def test_policy_active_rejects_untrained_model(self):
        self.responses=[self.policy_line()]
        with self.assertRaisesRegex(RuntimeError,'trained model'):self.invoke(['policy','active'])
        self.assertEqual(self.ports[-1].writes,[b'!status\n'])

    def test_policy_requires_firmware_support_and_explicit_stop(self):
        for response in (status_line(False,manual_stop=True),self.policy_line(driving=True),
                         self.policy_line(manual_stop=False)):
            self.responses=[response]
            with self.assertRaises(RuntimeError):self.invoke(['policy','shadow'])
            self.assertEqual(self.ports[-1].writes,[b'!status\n'])

    def test_policy_active_selection_does_not_arm(self):
        self.responses=[self.policy_line(trained=True),self.policy_line('active',trained=True)]
        self.invoke(['policy','active'])
        self.assertEqual(self.ports[-1].writes,[b'!status\n',b'!policy active\n',b'!status\n'])

    def test_policy_status_is_read_only_and_rejected_modes_fail(self):
        self.responses=[self.policy_line()];self.invoke(['policy','status'])
        self.assertEqual(self.ports[-1].writes,[b'!status\n'])
        self.responses=[self.policy_line(),self.policy_line()]
        with self.assertRaisesRegex(RuntimeError,'not accepted'):self.invoke(['policy','shadow'])

    def test_stop_precedes_handshake_even_when_status_is_lost(self):
        for responses, should_fail in [([status_line(driving=False, manual_stop=True)], False), ([], True)]:
            self.responses = responses
            if should_fail:
                with self.assertRaises(RuntimeError):
                    self.invoke(['stop'])
            else:
                self.invoke(['stop'])
            self.assertEqual(self.ports[-1].writes, [b'!stop\n', b'!status\n'])
            self.assertTrue(self.ports[-1].closed)

    def test_stop_waits_for_stale_status_to_confirm_both_latch_and_no_drive(self):
        self.responses = [status_line(), status_line(driving=False),
                          status_line(manual_stop=True), status_line(driving=False, manual_stop=True)]
        self.invoke(['stop'])
        self.assertEqual(self.ports[-1].writes, [b'!stop\n'] + [b'!status\n'] * 4)
        self.assertGreaterEqual(self.clock.now, .2)
        self.assertTrue(self.ports[-1].closed)

    def test_stop_fails_if_stopped_state_is_not_confirmed(self):
        # Correct firmware identity alone is insufficient; require real booleans.
        for driving, latch in [(True, False), (False, False), (True, True),
                               (0, True), (False, 1), (None, True), (False, None)]:
            self.responses = [status_line(driving=driving, manual_stop=latch)] * 100
            before = self.clock.now
            with self.subTest(driving=driving, manual_stop=latch), \
                    self.assertRaisesRegex(RuntimeError, 'stopped state was not confirmed'):
                self.invoke(['stop'])
            self.assertGreaterEqual(self.clock.now - before, 2)
            self.assertLess(self.clock.now - before, 2.2)
            self.assertEqual(self.ports[-1].writes[0], b'!stop\n')
            self.assertTrue(all(data == b'!status\n' for data in self.ports[-1].writes[1:]))
            self.assertTrue(self.ports[-1].closed)

    def test_move_requires_balancing_and_closes_on_refusal(self):
        self.responses = [status_line(driving=False)]
        with self.assertRaises(RuntimeError):
            self.invoke(['move', '--forward', '10'])
        self.assertEqual(self.ports[-1].writes, [b'!status\n'])
        self.assertTrue(self.ports[-1].closed)

    def test_timed_move_uses_signed_mapping_and_returns_to_neutral(self):
        self.invoke(['move', '--forward', '-100', '--turn', '100', '--seconds', '.25'])
        self.assertEqual(self.ports[-1].writes,
                         [b'!status\n', b'!move 0 255\n', b'!neutral\n', b'!status\n'])
        self.assertGreaterEqual(self.clock.now, .25)
        self.assertTrue(self.ports[-1].closed)

    def test_interrupted_move_still_requests_neutral_and_closes(self):
        self.clock.interrupt_next_sleep = True
        with self.assertRaises(KeyboardInterrupt):
            self.invoke(['move', '--forward', '50'])
        self.assertEqual(self.ports[-1].writes, [b'!status\n', b'!move 192 128\n', b'!neutral\n'])
        self.assertTrue(self.ports[-1].closed)

    def test_failed_move_write_still_attempts_neutral_and_closes(self):
        self.fail_write = b'!move 128 128\n'
        with self.assertRaises(OSError):
            self.invoke(['move'])
        self.assertEqual(self.ports[-1].writes, [b'!status\n', b'!move 128 128\n', b'!neutral\n'])
        self.assertTrue(self.ports[-1].closed)

    def test_invalid_arguments_rejected_before_constructing_serial(self):
        invalid = [['move', '--forward', '101'], ['move', '--turn', 'nan'],
                   ['move', '--seconds', '0'], ['move', '--seconds', '30.01'],
                   ['move', '--seconds', 'nan'], ['move', '--seconds', 'inf'],
                   ['height', '-1'], ['roll', '256'], ['name', ''],
                   ['name', '\u817f' * 7], ['name', 'bad\n!arm'], ['name', 'bad\x7f'],
                   ['led', '4', '0', '0', '0'], ['led', '0', '-1', '0', '0'],
                   ['led', '0', '0', '0', '256'], ['log', '--seconds', '0'],
                   ['log', '--seconds', '3601'], ['cal', 'unknown']]
        for arguments in invalid:
            with self.subTest(arguments=arguments), self.assertRaises(SystemExit) as error:
                self.invoke(arguments)
            self.assertEqual(error.exception.code, 2)
        self.assertEqual(self.ports, [])

    def test_utf8_name_boundary_uses_bytes(self):
        suffix = '\u817f' * 6 + 'AB'  # 20 bytes, although only eight characters.
        self.invoke(['name', suffix])
        self.assertEqual(self.ports[-1].writes[1], ('!name ' + suffix + '\n').encode('utf-8'))


if __name__ == '__main__':
    unittest.main()
