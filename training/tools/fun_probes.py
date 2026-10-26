"""Quick simulator probes for "what else could this robot be trained to do?" (2026-10-22).  The v6 policy is used as it is.

Usage: .venv/bin/python tools/fun_probes.py [legwave] [shove]      (no argument = both, about 3 minutes)

(A "getup" probe used to be here.  It used the training model's box body, which has no front prongs and no rear arms, and wrongly concluded that the
robot cannot stand up after a fall.  See getup_probe.py, which uses the real shell.)
legwave  A leg dance while standing: both legs together (height wave) or against each other (roll wave, firmware levelling bypassed as in
         step_probe_legs.py), amplitude per leg and frequency swept.  The simulated leg is faster than the real servo and the firmware smooths the
         height command (0.97/0.03 per tick), so the stock actions are slower than this.
shove    Random pushes every 1-3 s (80 ms pulses, 0.1-0.8 N x scale, plus torque pulses) for 40 s on the randomised plant.

Results 2026-10-22 (runs/fun_probes.log): see REBUILD.md section 9.
"""
import sys, os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import mujoco
from multiprocessing import Pool
HDR = str(ROOT / 'artifacts/balance-v6/policy_weights.h')


# ------------------------------------------------------------------------------------------------ legwave
def wave_run(a):
    kind, amp, freq, seed, domain = a
    os.chdir(ROOT)
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HDR)
    env = RiderEnv2(domain=domain, commands=False, pushes=False, episode_seconds=22, native_policy=native, noise=1.0)
    env.reset(seed=seed, options={'pitch': np.radians(-3.0), 'roll': 0., 'yaw': 0.})
    orig = env._apply_action
    nominal = []

    def patched():
        saved = env.out[16:18].copy()
        if env.elapsed >= 2.0:
            if not nominal:
                nominal.append(saved.copy())
            s = amp * np.sin(2 * np.pi * freq * (env.elapsed - 2.0))
            env.out[16:18] = nominal[0] + (np.array([s, s]) if kind == 'height' else np.array([s, -s]))
        orig()
        env.out[16:18] = saved
    env._apply_action = patched
    pitch, roll = [], []
    while True:
        _, _, term, trunc, info = env.step(native.action.copy())
        if env.elapsed > 4.0:
            pitch.append(np.degrees(info['pitch']))
            roll.append(np.degrees(info['roll']))
        if term or trunc:
            fell = bool(term)
            break
    env.close(); native.close()
    return dict(fell=fell, pitch_sd=float(np.std(pitch)) if pitch else 0., roll_max=float(np.max(np.abs(roll))) if roll else 0.)


def legwave():
    amps = {'height': (0.005, 0.010, 0.018), 'roll': (0.003, 0.006, 0.012)}
    freqs = (0.5, 1.0, 2.0, 3.0)
    for domain, seeds in ((0., range(2)), (1., range(6))):
        jobs = [(k, a, f, s, domain) for k in amps for a in amps[k] for f in freqs for s in seeds]
        with Pool(9) as p:
            rows = p.map(wave_run, jobs, chunksize=1)
        print(f"leg wave while standing, {'nominal plant, 2 seeds' if not domain else 'randomised plant, 6 seeds'}; cell = fell/runs, pitch sd [deg], max body roll [deg]")
        for k in amps:
            print(f'  {k} wave (amplitude per leg):')
            for a in amps[k]:
                cells = []
                for f in freqs:
                    rr = [r for j, r in zip(jobs, rows) if j[:3] == (k, a, f)]
                    cells.append(f"{f:.1f} Hz {sum(r['fell'] for r in rr)}/{len(rr)} sd {np.mean([r['pitch_sd'] for r in rr]):3.1f} roll {np.max([r['roll_max'] for r in rr]):4.1f}")
                print(f'    +-{a * 1000:3.0f} mm  ' + ' | '.join(cells))
    print()


# ------------------------------------------------------------------------------------------------ shove
def shove_run(a):
    scale, seed = a
    os.chdir(ROOT)
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HDR)
    env = RiderEnv2(domain=1.0, commands=False, pushes=True, push_scale=scale, episode_seconds=40, native_policy=native, noise=1.0)
    env.reset(seed=seed, options={'pitch': np.radians(-3.0), 'roll': 0., 'yaw': 0.})
    while True:
        _, _, term, trunc, _ = env.step(native.action.copy())
        if term or trunc:
            break
    result = dict(scale=scale, fell=bool(term), t=env.elapsed)
    env.close(); native.close()
    return result


def shove():
    scales = (1., 2., 3., 4., 6.)
    with Pool(9) as p:
        rows = p.map(shove_run, [(sc, s) for sc in scales for s in range(16)], chunksize=1)
    print('random shoves every 1-3 s for 40 s (80 ms pulses of 0.1-0.8 N x scale plus torque pulses), randomised plant, 16 seeds')
    for sc in scales:
        rr = [r for r in rows if r['scale'] == sc]
        print(f"  scale {sc:.0f} (up to {0.8 * sc:.1f} N): fell {sum(r['fell'] for r in rr)}/{len(rr)}, median time to fall {np.median([r['t'] for r in rr]):4.1f} s")
    print()


if __name__ == '__main__':
    todo = sys.argv[1:] or ['legwave', 'shove']
    for name in todo:
        {'legwave': legwave, 'shove': shove}[name]()
