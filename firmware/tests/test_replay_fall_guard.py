"""The replay tool runs the firmware's own free-fall detector over a recording; synthetic traces, no hardware."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import replay_fall_guard as rf  # noqa: E402


def trace(path, accel_g, dt_ms=18):
    """accel_g: list of total accelerations (g), one per 55 Hz sample; put it all on the z axis."""
    rows = [dict(ms=1000 + i * dt_ms, acc_x=0, acc_y=0, acc_z=int(round(g * 981))) for i, g in enumerate(accel_g)]
    Path(path).write_text(''.join(json.dumps(r) + '\n' for r in rows))
    return rows


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.binary = rf.build()
        self.folder = Path(tempfile.mkdtemp())

    def run_trace(self, accel_g, **kw):
        path = self.folder / 'trace.jsonl'
        trace(path, accel_g)
        return rf.replay(self.binary, rf.samples(path), kw.get('limit', 0.5), kw.get('ms', 60.))

    def test_standing_and_handling_dips_do_not_trip(self):
        standing = [1.0] * 500
        handling = [1.0] * 100 + [0.23] + [1.0] * 50 + [0.29, 0.6] + [1.0] * 100 + [1.8, 1.2] * 30
        for g in (standing, handling):
            trips, longest, minimum = self.run_trace(g)
            self.assertEqual(trips, 0)
        self.assertLessEqual(longest, 40.0)         # the deepest dips last one or two held samples
        self.assertAlmostEqual(minimum, 0.23, places=2)

    def test_a_table_drop_trips_once(self):
        drop = [1.0] * 100 + [0.04] * 22 + [5.0, 1.2] + [1.0] * 100     # 22 samples = 400 ms of free fall, the impact, lying still
        trips, longest, minimum = self.run_trace(drop)
        self.assertEqual(trips, 1)
        self.assertGreaterEqual(longest, 80.0)
        self.assertLess(minimum, 0.1)

    def test_a_short_hop_does_not_trip_and_the_limits_can_be_changed(self):
        hop = [1.0] * 50 + [0.1] * 2 + [1.0] * 50                       # 2 samples = 8 ticks of 5 ms = 40 ms in the replay
        self.assertEqual(self.run_trace(hop)[0], 0)                      # the default hold is 60 ms
        self.assertEqual(self.run_trace(hop, ms=30.)[0], 1)

    def test_cli_lists_traces_and_totals(self):
        import contextlib
        import io
        path = self.folder / 'drop.jsonl'
        trace(path, [1.0] * 100 + [0.04] * 22 + [1.0] * 100)
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            sys.argv = ['replay_fall_guard.py', str(path)]
            rf.main()
        self.assertIn('total trips: 1', buffer.getvalue())


if __name__ == '__main__':
    unittest.main()
