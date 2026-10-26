"""dzcomp A/B tool tests: the recordings are replaced by synthetic traces; no serial port is opened."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'tests'))
import dzcomp_ab_test as ab  # noqa: E402
from test_standing_report import make_rows  # noqa: E402


def stepping(t):          # +-65 mm slow creep and quick kick, period 7 s
    phase = (t % 7.0) / 7.0
    return np.where(phase < 6 / 7, 65 - 130 * phase / (6 / 7), -65 + 130 * (phase - 6 / 7) * 7)


def still(t):
    return 1.5 * np.sin(2 * np.pi * 2.2 * t) * np.minimum(t / 20.0, 1.0)


def preload_flips(t):
    return np.where((t // 0.08) % 2 == 0, 40.0, -40.0)      # ~12 flips/s


class AbTests(unittest.TestCase):
    def run_main(self, off_curve, on_curve, on_preload=None, seconds=(75, 105), learn=40):
        commands_seen = []

        def fake_capture(port, output, secs, commands, wait_start, cue):
            commands_seen.append((Path(output).name, list(commands), secs))
            curve = off_curve if Path(output).name == 'off.jsonl' else on_curve
            rows = make_rows(secs + 1, curve, cmd=lambda t: np.zeros_like(t), preload=(on_preload if Path(output).name == 'on.jsonl' and on_preload else None))
            Path(output).write_text(''.join(json.dumps(r) + '\n' for r in rows))
            return 0

        out = Path(tempfile.mkdtemp())
        argv = ['dzcomp_ab_test.py', '--port', 'fake', '--out', str(out), '--off-seconds', str(seconds[0]), '--on-seconds', str(seconds[1]),
                '--learn-seconds', str(learn)]
        with mock.patch.object(ab, 'run_capture', side_effect=fake_capture), mock.patch.object(sys, 'argv', argv):
            ab.main()
        return commands_seen, json.loads((out / 'summary.json').read_text()), (out / 'summary.txt').read_text()

    def test_sends_only_settings_commands_in_the_right_order(self):
        seen, _, _ = self.run_main(stepping, still)
        self.assertEqual(seen[0][:2], ('off.jsonl', ['!dzcomp reset', '!dzcomp off']))
        self.assertEqual(seen[1][:2], ('on.jsonl', ['!dzcomp reset', '!dzcomp on']))
        for _, commands, _ in seen:
            self.assertTrue(all(c.startswith('!dzcomp') for c in commands))

    def test_a_robot_that_steps_with_the_preload_off_and_stands_with_it_on_is_an_improvement(self):
        _, summary, text = self.run_main(stepping, still)
        self.assertEqual(summary['verdict'], 'improved')
        self.assertGreater(summary['off']['p2p_mm'], 100)
        self.assertLess(summary['on']['p2p_mm'], 10)
        self.assertIn('补偿有效', text)

    def test_no_stepping_in_the_off_run_makes_the_comparison_inconclusive(self):
        _, summary, _ = self.run_main(still, still)
        self.assertEqual(summary['verdict'], 'inconclusive')

    def test_no_improvement_is_reported_as_such(self):
        _, summary, _ = self.run_main(stepping, stepping)
        self.assertEqual(summary['verdict'], 'no_clear_improvement')

    def test_chattering_preload_is_flagged_even_when_the_robot_stands_still(self):
        _, summary, text = self.run_main(stepping, still, on_preload=preload_flips)
        self.assertEqual(summary['verdict'], 'improved')
        self.assertGreater(summary['on']['flips_per_s'], 8)
        self.assertIn('!dzcomp off', text)

    def test_verdict_with_missing_data(self):
        self.assertEqual(ab.verdict(None, None)[1], 'no_data')

    def test_summary_is_a_median_so_one_bump_does_not_decide(self):
        windows = [dict(p2p_mm=v, steps=0, v_rms_mm_s=1., hf=1., pitch_sd_deg=.1, preload=0., flips_per_s=0.) for v in (3., 4., 400.)]
        self.assertEqual(ab.summarise(windows)['p2p_mm'], 4.0)
        self.assertIsNone(ab.summarise([]))


class MissingModulesTests(unittest.TestCase):
    def test_a_missing_numpy_stops_the_tool_with_advice_before_anything_is_recorded(self):
        import importlib.util
        for name in ('dzcomp_ab_test', 'standing_report'):
            spec = importlib.util.spec_from_file_location(name + '_probe', ROOT / 'tools' / f'{name}.py')
            module = importlib.util.module_from_spec(spec)
            with mock.patch.dict(sys.modules, {'numpy': None}):
                with self.assertRaises(SystemExit) as raised:
                    spec.loader.exec_module(module)
            self.assertIn('python3', str(raised.exception))
            self.assertIn('numpy', str(raised.exception))


if __name__ == '__main__':
    unittest.main()
