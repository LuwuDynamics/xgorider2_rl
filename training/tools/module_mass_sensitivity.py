"""How much does the compute-module assembly (mass, position) matter?  The simulator assumes the CM5 version (whole robot 578.3 g
including a 62.8 g CM5 assembly with shell, fan and heatsink, see MODEL.md); the robot on the bench has a CM4 without fan/heatsink.

Usage: .venv/bin/python tools/module_mass_sensitivity.py

For module mass changes of -35..+35 g and module position changes of +-10 mm it prints the zero-torque pitch of the whole robot (where
it balances), and the falls under random pushes (24 seeds x 40 s, v6 + dead-zone preload) on the nominal plant and the randomised
family.  Result 2026-10-22: no falls anywhere, pitch jitter within +-20%; the balance point moves by 3-5 degrees (25 g lighter:
-4.4 -> -1.2 deg), which the position integral and the IMU pitch trim absorb.  (Per-job calibration patches must not stack: the
wrapper always wraps the ORIGINAL sample function.)
"""
import sys, os, copy, dataclasses, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from multiprocessing import Pool
SIM = ROOT
HDR = str(SIM / 'artifacts/balance-v6/policy_weights.h')

def patch(delta_g, dx_mm):
    import rider_sim.calibration as cal
    orig = getattr(cal, '_orig_sample', None) or cal.sample      # never wrap a wrapper: workers run many jobs
    cal._orig_sample = orig
    def wrapped(inputs, rng, domain=0.):
        inp = copy.deepcopy(inputs)
        d = delta_g / 1000.
        inp['measured_masses_kg']['robot'] += d          # negative = lighter module
        inp['measured_masses_kg']['cm5'] += d
        for row in inp['cm5']:
            row['center_m'] = [row['center_m'][0] + dx_mm / 1000., row['center_m'][1], row['center_m'][2]]
        return orig(inp, rng, domain)
    cal.sample = wrapped

def physical(a):
    delta_g, dx_mm = a
    os.chdir(SIM); sys.path.insert(0, str(SIM)); patch(delta_g, dx_mm)
    from rider_sim.env2 import RiderEnv2
    env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=1); env.reset(seed=1)
    out = dict(delta_g=delta_g, dx_mm=dx_mm, total_g=1000 * float(env.model.body_mass.sum()), eq=float(np.degrees(env.equilibrium_pitch)))
    env.close(); return out

def run(a):
    try:
        return run_inner(a)
    except ValueError as exc:
        return dict(delta_g=a[0], dx_mm=a[1], domain=a[2], preload=a[4], fell=True, t=0., pitch_sd=float('nan'), invalid=str(exc)[:40])

def run_inner(a):
    delta_g, dx_mm, domain, seed, preload = a
    os.chdir(SIM); sys.path.insert(0, str(SIM)); patch(delta_g, dx_mm)
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HDR)
    if preload: native.set_dead_zone(55., 8., 3., 8., .8, 30., 12., 6.)
    env = RiderEnv2(domain=domain, commands=False, pushes=True, episode_seconds=40, native_policy=native, noise=1.0)
    env.reset(seed=seed)
    pitch = []
    while True:
        _, _, term, trunc, info = env.step(native.action.copy())
        pitch.append(info['pitch'])
        if term or trunc: break
    t = env.elapsed
    env.close(); native.close()
    return dict(delta_g=delta_g, dx_mm=dx_mm, domain=domain, preload=preload, fell=bool(term), t=t, pitch_sd=float(np.degrees(np.std(pitch[len(pitch) // 3:]))))

if __name__ == '__main__':
    variants = [(-35., 0.), (-25., 0.), (-15., 0.), (0., 0.), (15., 0.), (25., 0.), (35., 0.), (0., -10.), (0., 10.), (-35., 10.)]
    began = time.monotonic()
    with Pool(9) as p:
        for ph in p.map(physical, variants):
            print(f"module {ph['delta_g']:+4.0f} g, moved {ph['dx_mm']:+4.0f} mm: whole robot {ph['total_g']:6.1f} g, zero-torque pitch {ph['eq']:6.2f} deg")
        jobs = [(dg, dx, dom, s, True) for (dg, dx) in variants for dom in (0.0, 1.0) for s in range(24)]
        rows = p.map(run, jobs, chunksize=1)
    print('\nrandom pushes every 1-3 s for 40 s, v6 + preload, 24 seeds:  falls  (median pitch sd deg of those that survived)')
    print(f"{'module change':22s} {'nominal plant':>18s} {'randomised family':>20s}")
    for (dg, dx) in variants:
        cells = []
        for dom in (0.0, 1.0):
            rr = [r for r in rows if (r['delta_g'], r['dx_mm'], r['domain']) == (dg, dx, dom)]
            ok = [r for r in rr if not r['fell']]
            cells.append(f"{sum(r['fell'] for r in rr):>2}/24  ({np.median([r['pitch_sd'] for r in ok]) if ok else float('nan'):.2f})")
        print(f"{dg:+4.0f} g, {dx:+4.0f} mm        {cells[0]:>18s} {cells[1]:>20s}")
    print('wall', round(time.monotonic() - began), 's')
