"""Does the simulator reproduce the robot's measured frequency response?

Injects a small sinusoid on the common wheel command (after the policy, before the actuator) while a
stabilising policy balances, and estimates the response of the *same signals the real trace logs*
(firmware pitch rate, mean wheel speed in rad/s) at the probe frequency (DFT over whole periods; the probe
is independent of the loop noise so the ratio is unbiased in closed loop).  Prints sim / measured
magnitude ratio and phase difference at every measured frequency.  A calibrated plant gives ratios ~1 and
phase differences within ~10 degrees; this is the check that failed (phase off by 45 degrees, wheel signal
3x too large) for the original simulator.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rider_sim.env2 import RiderEnv2  # noqa: E402
from rider_sim.linear_policy import LinearPolicy  # noqa: E402


class ProbeEnv(RiderEnv2):
    probe_amp = 0.
    probe_hz = 7.

    def __init__(self, *a, **k):
        self.rec = []
        super().__init__(*a, **k)

    def _apply_action(self):
        super()._apply_action()
        t = self.us * 1e-6
        p = self.probe_amp * math.sin(2 * math.pi * self.probe_hz * t) if self.elapsed > 2.0 else 0.
        if self.delay:
            stamp, drive = self.delay.pop()
            self.delay.append((stamp, drive + np.array([p, p]) / 1023.))
        u = (self.applied[0] - self.applied[1]) / 2. + p
        self.rec.append((t, u, float(self.out[3]), .5 * float(self.feedback[6] + self.feedback[7]) / 27.5))


def probe(hz, amp, seconds, seed, plant=None, domain=0., policy=None):
    policy = policy or LinearPolicy()
    env = ProbeEnv(domain=domain, commands=False, pushes=False, episode_seconds=seconds, noise=1.0, plant_override=plant)
    env.probe_amp, env.probe_hz = amp, hz
    env.reset(seed=seed, options={'pitch': 0., 'roll': 0., 'yaw': 0.})
    while True:
        _, _, term, trunc, _ = env.step(policy.action(env.raw_observation))
        if term or trunc:
            break
    r = np.array(env.rec)
    env.close()
    t = r[:, 0]
    sel = t > 8.
    t = t[sel]
    n = int((t[-1] - t[0]) * hz)
    keep = t <= t[0] + n / hz
    t = t[keep]
    e = np.exp(-2j * np.pi * hz * t)

    def comp(x):
        x = x[sel][keep]
        return np.sum((x - x.mean()) * e)
    U, W, S = comp(r[:, 1]), comp(r[:, 2]), comp(r[:, 3])
    return W / U, S / U, not term


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--amp', type=float, default=100.)
    parser.add_argument('--seconds', type=float, default=24.)
    parser.add_argument('--seed', type=int, default=11)
    args = parser.parse_args()
    frf = json.loads((ROOT / 'models/measured_frf.json').read_text())
    print(f"{'f[Hz]':>6s} | {'rate sim/meas':>13s} {'dphase':>7s} | {'wheel sim/meas':>14s} {'dphase':>7s}")
    worst = 0.
    for f, rm, rp, wm, wp in zip(frf['freqs_hz'], frf['rate_mag'], frf['rate_phase_deg'], frf['wheel_mag'], frf['wheel_phase_deg']):
        hr, hw, ok = probe(f, args.amp, args.seconds, args.seed)
        dr = np.degrees(np.angle(hr * np.exp(-1j * np.radians(rp))))
        dw = np.degrees(np.angle(hw * np.exp(-1j * np.radians(wp))))
        worst = max(worst, abs(dr), abs(dw))
        print(f"{f:6.1f} | {abs(hr) / rm:13.2f} {dr:7.1f} | {abs(hw) / wm:14.2f} {dw:7.1f}{'' if ok else '   (fell)'}", flush=True)
    print(f'worst phase difference {worst:.1f} deg')


if __name__ == '__main__':
    main()
