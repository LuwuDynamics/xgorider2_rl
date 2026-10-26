"""Standing still on a wheel with a dead zone: the stepping cycle and the learned dead-zone preload, in the simulator.

Usage: .venv/bin/python tools/standing_dead_zone.py [--seconds 120] [--late-from 60] [--seeds 4] [--plants c0 c25 c45 dz45 ...]
                                                    [--configs off final ...] [--workers 9]

Runs the native C++ controller (rider_balance_v6 plus src/policy/dead_zone_comp.h) standing on plants whose wheels have a dead
zone: Coulomb friction (``cNN`` = NN command units of friction, 45 = the dead zone measured on the robot), a pure servo input
dead band (``dzNN``) or friction plus stiction (``sNN``), and prints, over the time after ``--late-from`` seconds:

* p2p    peak-to-peak wheel travel (mm);   rev  speed reversals above 10 mm/s per minute;
* v_rms  rms ground speed (mm/s);          hf   rms of the common-mode wheel command above 3 Hz;
* K end  the learned preload at the end;   trips  how often the chatter breaker cut it.

``off`` is the controller as it was; ``final`` is the firmware default (max 55, hysteresis 8, ramp 3/s, chatter 8/s, back-off 0.8).
Findings and the alternatives that did not work: simulation/REBUILD.md section 6.
"""
import argparse
import dataclasses
from multiprocessing import Pool
from pathlib import Path
import re
import sys
import time

import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

KT = 1.57e-4 * 1.19          # N*m per command unit of the nominal plant (rider_sim.plant: kt * KT_TRIM)
HEADER = ROOT / 'artifacts/balance-v6/policy_weights.h'
CONFIGS = {'off': None, 'final': (55., 8., 3., 8., 0.8), 'max40': (40., 8., 3., 8., 0.7)}


def nominal_plant():
    from rider_sim.env2 import RiderEnv2
    env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=1)
    env.reset(seed=1)
    plant = env.plant
    env.close()
    return plant


def plant_kwargs(name):
    match = re.fullmatch(r'(c|dz|s)(\d+(?:\.\d+)?)', name)
    if not match:
        raise ValueError(f'plant {name!r}: expected cNN, dzNN or sNN')
    kind, value = match.group(1), float(match.group(2))
    if kind == 'c':
        return dict(coulomb=value * KT)
    if kind == 'dz':
        return dict(deadzone=value)
    return dict(coulomb=value * KT / 2, stiction=value * KT / 2)      # sNN: static friction NN units, kinetic half of it


def stand(plant, seconds, seed, dz):
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HEADER)
    if dz:
        native.set_dead_zone(*dz)
    env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=seconds, native_policy=native, noise=1.0, plant_override=plant)
    env.reset(seed=seed)
    rec = []
    while True:
        _, _, terminated, truncated, info = env.step(native.action.copy())
        command = info['wheel_command']
        rec.append((info['time'], float(env.encoder_position()), info['velocity'], 0.5 * (command[0] - command[1]), native.dead_zone))
        if terminated or truncated:
            break
    env.close()
    native.close()
    return terminated, rec


def metrics(rec, late_from):
    t = np.array([r[0] for r in rec])
    x, v, cmd = np.array([r[1] for r in rec]), np.array([r[2] for r in rec]), np.array([r[3] for r in rec])
    m = t > late_from
    if m.sum() < 400:
        return None
    x = 1000 * (x[m] - x[m][0])
    v = v[m]
    sign = np.sign(np.where(np.abs(v) > 0.01, v, 0))
    nz = sign[sign != 0]
    minutes = (t[m][-1] - t[m][0]) / 60
    fs = 1 / float(np.median(np.diff(t)))
    hf = signal.sosfilt(signal.butter(4, 3., 'highpass', fs=fs, output='sos'), cmd[m] - cmd[m].mean())[int(fs):]
    return dict(p2p=float(x.max() - x.min()), rev=float(np.sum(nz[1:] != nz[:-1]) / minutes) if len(nz) > 1 else 0.,
                v_rms=1000 * float(np.sqrt(np.mean(v ** 2))), hf=float(np.sqrt(np.mean(hf ** 2))),
                k_end=float(rec[-1][4][0]), trips=int(rec[-1][4][2]))


def job(args):
    plant_name, config_name, seed, seconds, late_from = args
    plant = dataclasses.replace(nominal_plant(), **plant_kwargs(plant_name))
    fell, rec = stand(plant, seconds, seed, CONFIGS[config_name])
    m = metrics(rec, late_from)
    return dict(plant=plant_name, config=config_name, fell=fell, **(m or dict(p2p=0., rev=0., v_rms=0., hf=0., k_end=0., trips=0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--late-from', type=float, default=60.)
    parser.add_argument('--seeds', type=int, default=4)
    parser.add_argument('--plants', nargs='+', default=['c0', 'c25', 'c35', 'c45', 'c60', 'dz45', 's60'])
    parser.add_argument('--configs', nargs='+', default=['off', 'final'], choices=sorted(CONFIGS))
    parser.add_argument('--workers', type=int, default=9)
    args = parser.parse_args()
    jobs = [(p, c, s, args.seconds, args.late_from) for c in args.configs for p in args.plants for s in range(args.seeds)]
    began = time.monotonic()
    with Pool(args.workers) as pool:
        rows = pool.map(job, jobs, chunksize=1)
    print(f"{'config':8s} {'plant':6s} {'fell':>4} {'p2p mm':>7} {'rev/min':>8} {'v rms':>6} {'hf':>6} {'K end':>6} {'trips':>5}   (after {args.late_from:.0f} s, mean of {args.seeds} seeds)")
    for c in args.configs:
        for p in args.plants:
            ms = [r for r in rows if r['config'] == c and r['plant'] == p]
            mean = lambda k: float(np.mean([r[k] for r in ms]))
            print(f"{c:8s} {p:6s} {sum(r['fell'] for r in ms):>4} {mean('p2p'):7.1f} {mean('rev'):8.1f} {mean('v_rms'):6.1f} {mean('hf'):6.1f} {mean('k_end'):6.1f} {mean('trips'):5.1f}")
    print('wall', round(time.monotonic() - began), 's')


if __name__ == '__main__':
    main()
