"""What can the legs do to the weight on the wheels?  Leg differential (one leg out by x, the other in by x, firmware levelling bypassed) vs.
body roll and the share of the weight on the wheel whose side is raised.  Standing on flat ground, v6 policy, nominal plant.

Usage: .venv/bin/python tools/leg_load_split.py
Result 2026-10-22: +-6 mm -> roll 8.6 deg, that wheel carries 47 % (56 % level); +-12 mm -> 16.8 deg, 37 %; +-18 mm -> 30 deg and the robot tips over.
"""
import sys, os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np, mujoco
from rider_sim.env2 import RiderEnv2
from rider_sim.native_policy import NativePolicy

os.chdir(ROOT)
for amp in (0.0, 0.006, 0.012, 0.018):
    native = NativePolicy(str(ROOT / 'artifacts/balance-v6/policy_weights.h'))
    env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=8, native_policy=native, noise=1.0)
    env.reset(seed=0, options={'pitch': np.radians(-3.0), 'roll': 0., 'yaw': 0.})
    orig = env._apply_action
    nominal = []

    def patched(orig=orig, amp=amp):
        saved = env.out[16:18].copy()
        if not nominal and env.elapsed >= 1.0:
            nominal.append(saved.copy())
        env.out[16:18] = (nominal[0] if nominal else saved) + (np.array([amp, -amp]) if nominal else 0)
        orig()
        env.out[16:18] = saved
    env._apply_action = patched
    for _ in range(1200):
        _, _, term, _, info = env.step(native.action.copy())
        if term:
            break
    m, d = env.model, env.data
    tyre = [m.geom('tyre11').id, m.geom('tyre21').id]
    force = np.zeros(2)
    for c in range(d.ncon):
        con = d.contact[c]
        for i, g in enumerate(tyre):
            if g in (con.geom1, con.geom2):
                fr = np.zeros(6)
                mujoco.mj_contactForce(m, d, c, fr)
                force[i] += fr[0]
    print(f'legs +-{amp * 1000:3.0f} mm: roll {np.degrees(info["roll"]):6.2f} deg, wheel normal force N = {np.round(force, 2)}, '
          f'share of the raised-side wheel {force[0] / force.sum() * 100:4.1f} % (fell={term})')
    env.close(); native.close()
