import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import standing_report as sr  # noqa: E402

FS = 55.0


def make_rows(seconds, travel_mm, cmd=None, preload=None, pitch=None, noise=0.0, seed=0):
    """Synthetic trace rows: wheel positions from a travel curve (mm), the policy command `cmd` (common mode) and a preload."""
    rng = np.random.default_rng(seed)
    n = int(seconds * FS)
    t = np.arange(n) / FS
    x = np.asarray(travel_mm(t) if callable(travel_mm) else np.full(n, float(travel_mm)))
    ticks = x / sr.MM_PER_TICK + rng.normal(0, noise, n)
    c = np.zeros(n) if cmd is None else np.asarray(cmd(t))
    p = np.zeros(n) if preload is None else np.asarray(preload(t))
    rows = []
    for i in range(n):
        a0, a1 = c[i], -c[i]
        rows.append(dict(ms=int(1000 + t[i] * 1000), pos11=int(round(500 + ticks[i])) % 1024, pos21=int(round(500 - ticks[i])) % 1024,
                         cmd11=int(round(a0 + p[i])), cmd21=int(round(a1 - p[i])), a0=int(round(a0)), a1=int(round(a1)),
                         pitch=int(round(100 * (pitch(t[i]) if pitch else 0.0)))))
    return rows


class StandingReportTests(unittest.TestCase):
    def test_unwrap_follows_the_encoder_ring(self):
        ticks = sr.unwrap([1020, 1023, 2 % 1024, 5, 1, 1022])
        self.assertEqual([float(v) for v in ticks], [0, 3, 6, 9, 5, 2])

    def test_a_still_robot_has_no_steps_and_no_speed(self):
        rows = make_rows(40, 0.0)
        for w in sr.analyse(rows, window=10):
            self.assertLess(w['p2p_mm'], 0.5)
            self.assertEqual(w['steps'], 0)
            self.assertLess(w['v_rms_mm_s'], 0.5)
            self.assertEqual(w['preload'], 0.0)

    def test_the_stepping_sawtooth_is_counted(self):
        # slow creep back for 6 s, quick kick forward in 1 s: +-65 mm, period 7 s (the simulated stick-slip cycle)
        def travel(t):
            phase = (t % 7.0) / 7.0
            return np.where(phase < 6 / 7, 65 - 130 * phase / (6 / 7), -65 + 130 * (phase - 6 / 7) * 7)
        rows = make_rows(43, travel)
        windows = sr.analyse(rows, window=14)
        self.assertEqual(len(windows), 3)
        for w in windows:
            self.assertGreater(w['p2p_mm'], 120)
            self.assertGreaterEqual(w['steps'], 3)
            self.assertGreater(w['v_rms_mm_s'], 20)

    def test_small_jitter_below_the_threshold_is_not_a_step(self):
        rows = make_rows(30, lambda t: 1.5 * np.sin(2 * np.pi * 2.2 * t))
        for w in sr.analyse(rows, window=15):
            self.assertEqual(w['steps'], 0)
            self.assertLess(w['p2p_mm'], 4)

    def test_preload_and_its_flips_are_read_from_the_command_difference(self):
        # a preload of +-30 that flips every 0.5 s (2 flips/s) on top of a 10-unit network command
        rows = make_rows(30, 0.0, cmd=lambda t: np.full_like(t, 10.0), preload=lambda t: np.where((t // 0.5) % 2 == 0, 30.0, -30.0))
        for w in sr.analyse(rows, window=15):
            self.assertAlmostEqual(w['preload'], 30.0, delta=1.0)
            self.assertAlmostEqual(w['flips_per_s'], 2.0, delta=0.3)

    def test_high_frequency_command_energy_shows_as_hf(self):
        quiet = make_rows(30, 0.0, cmd=lambda t: np.zeros_like(t))
        buzz = make_rows(30, 0.0, cmd=lambda t: 20 * np.sin(2 * np.pi * 9 * t))
        self.assertLess(max(w['hf'] for w in sr.analyse(quiet, window=15)), 0.5)
        self.assertGreater(min(w['hf'] for w in sr.analyse(buzz, window=15)), 10)

    def test_count_steps_zigzag(self):
        self.assertEqual(sr.count_steps([0, 1, 2, 3, 2, 3, 4], 5), 0)
        self.assertEqual(sr.count_steps([0, 10, 0, 10, 0], 5), 3)
        self.assertEqual(sr.count_steps([0, 10, 8, 10, 12, 4, 12], 5), 2)

    def test_cli_prints_a_table_and_rejects_short_traces(self):
        with tempfile.TemporaryDirectory() as folder:
            good = Path(folder) / 'good.jsonl'
            good.write_text(''.join(json.dumps(r) + '\n' for r in make_rows(32, 0.0)))
            short = Path(folder) / 'short.jsonl'
            short.write_text(''.join(json.dumps(r) + '\n' for r in make_rows(1, 0.0)))
            import contextlib
            import io
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                sys.argv = ['standing_report.py', str(good), '--window', '10']
                sr.main()
            self.assertIn('p2p mm', buffer.getvalue())
            sys.argv = ['standing_report.py', str(short)]
            with self.assertRaises(SystemExit):
                sr.main()


if __name__ == '__main__':
    unittest.main()
