"""Acceptance suite v2: survival *and* jitter, on plants that match the measured robot.

The old suite counted falls and tracking error only; the real robot failed on neither
but rang at 7-14 Hz. This one reports, per scenario:

* survival, velocity / yaw tracking, position drift,
* ``cmd_hf_rms``: rms of the common wheel command above 3 Hz (the jitter the user feels),
* ``cmd_tv``: total variation of the wheel command per second,
* ``pitch_std_deg`` / ``rate_std_dps``,
* the dominant frequency of the pitch-rate signal.

Scenarios cover the nominal identified plant, the randomised family and explicit
stress corners (stronger motor, extra latency, slower actuator, mis-scaled wheel sensor).
"""
import argparse
import copy
import json
from multiprocessing import Pool
from pathlib import Path
import time
import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parent


def make_actor(spec):
    """spec: ('linear',) | ('ppo', path) | ('native', header)"""
    kind = spec[0]
    if kind == 'linear':
        from rider_sim.linear_policy import LinearPolicy
        pol = LinearPolicy()
        return (lambda env, obs: pol.action(env.raw_observation)), None
    if kind == 'linearfile':
        import json as _json
        from rider_sim.linear_policy import LinearPolicy
        gains = _json.loads(Path(spec[1]).read_text())['gains']
        pol = LinearPolicy(gains=gains)
        return (lambda env, obs: pol.action(env.raw_observation)), None
    if kind == 'ppo':
        from stable_baselines3 import PPO
        model = PPO.load(spec[1], device='cpu')
        return (lambda env, obs: model.predict(obs, deterministic=True)[0]), None
    if kind == 'native':
        from rider_sim.native_policy import NativePolicy
        native = NativePolicy(spec[1])
        return (lambda env, obs: native.action.copy()), native
    raise ValueError(kind)


SCENARIOS = {
    'stand': dict(domain=0., commands=False, pushes=False),
    'stand_push': dict(domain=1., commands=False, pushes=True),
    # standing on wheels with the dead zone the robot really has (about 20-45 command units): does it keep still?
    'stand_fric': dict(domain=0., commands=False, pushes=False, plant=dict(coulomb=0.0070)),
    'stand_fric_hi': dict(domain=0., commands=False, pushes=False, plant=dict(coulomb=0.0100)),
    'drive': dict(domain=1., commands=True, pushes=True),
    'stress_gain': dict(domain=0.5, commands=True, pushes=True, plant=dict(kt=1.8)),
    'stress_delay': dict(domain=0.5, commands=True, pushes=True, plant=dict(delay=.012)),
    'stress_lag': dict(domain=0.5, commands=True, pushes=True, plant=dict(lag=.018)),
    'stress_sensor': dict(domain=0.5, commands=True, pushes=True, plant=dict(speed_scale=.4)),
    'stress_weak': dict(domain=0.5, commands=True, pushes=True, plant=dict(kt=.75, speed_scale=.27)),
    # beyond the training distribution: much harder pushes / large initial tilt / 5x sensor noise
    'hard_push': dict(domain=1., commands=True, pushes=True, push_scale=2.5, stand_pitch=(-.15, .15)),
    'noise5': dict(domain=1., commands=True, pushes=True, noise_level=5.0),
    # every randomised plant parameter at one end of its range (the thin-margin cases)
    'corner': dict(domain=1., commands=True, pushes=True, corner_fraction=1.0),
}


def run_episode(args):
    spec, scenario, seed, seconds, policy_every = args
    from rider_sim.env2 import RiderEnv2
    from rider_sim import plant as plantlib
    cfg = copy.deepcopy(SCENARIOS[scenario])
    plant_mod = cfg.pop('plant', None)
    actor, native = make_actor(spec)
    override = None
    if plant_mod:
        # apply the requested corner on top of the plant this seed would have drawn anyway
        base = RiderEnv2(episode_seconds=seconds, noise=1.0, **{k: v for k, v in cfg.items() if k != 'noise_level'})
        base.reset(seed=seed)
        override = copy.copy(base.plant)
        base.close()
        for key, value in plant_mod.items():
            if key == 'kt':
                override.kt *= value
            else:
                setattr(override, key, value)
    noise = cfg.pop('noise_level', 1.0)
    env = RiderEnv2(episode_seconds=seconds, native_policy=native, policy_every=policy_every, noise=noise,
                    plant_override=override, **cfg)
    obs, _ = env.reset(seed=seed)
    t, pitch, rate, cmd, vel_err, yaw_err, pos = [], [], [], [], [], [], []
    terminated = truncated = False
    while True:
        action = actor(env, obs)
        obs, _, terminated, truncated, info = env.step(action)
        t.append(env.elapsed)
        pitch.append(info['pitch'])
        rate.append(info['pitch_rate_fw'])
        cmd.append(0.5 * (env.last_command[0] - env.last_command[1]))
        vel_err.append(info['velocity'] - info['reference'])
        yaw_err.append(info['yaw_rate'] - info['yaw_reference'])
        pos.append(info['position_error'])
        if terminated or truncated:
            break
    env.close()
    if native:
        native.close()
    t = np.array(t)
    cmd = np.array(cmd)
    pitch = np.array(pitch)
    rate = np.array(rate)
    sel = t > 1.5
    result = dict(scenario=scenario, seed=seed, survived=not terminated, duration=float(t[-1]))
    if sel.sum() > 100:
        grid = np.arange(t[sel][0], t[-1], .006)
        c = np.interp(grid, t, cmd)
        r = np.interp(grid, t, rate)
        sos = signal.butter(4, 3., 'highpass', fs=1 / .006, output='sos')
        hf = signal.sosfilt(sos, c - c.mean())
        f, P = signal.welch(r - r.mean(), fs=1 / .006, nperseg=min(512, len(r)))
        band = (f > 2) & (f < 60)
        result.update(
            cmd_hf_rms=float(np.sqrt(np.mean(hf[200:] ** 2))) if len(hf) > 300 else float(np.sqrt(np.mean(hf ** 2))),
            cmd_std=float(np.std(c)), cmd_tv=float(np.sum(np.abs(np.diff(c))) / (grid[-1] - grid[0])),
            pitch_std_deg=float(np.degrees(np.std(pitch[sel]))), rate_std_dps=float(np.degrees(np.std(rate[sel]))),
            rate_peak_hz=float(f[band][np.argmax(P[band])]),
            vel_mae=float(np.mean(np.abs(np.array(vel_err)[sel]))), yaw_mae=float(np.mean(np.abs(np.array(yaw_err)[sel]))),
            pos_final=float(abs(pos[-1])))
    return result


def summarize(rows):
    out = {}
    for scenario in sorted({r['scenario'] for r in rows}):
        group = [r for r in rows if r['scenario'] == scenario]
        ok = [r for r in group if r['survived'] and 'cmd_hf_rms' in r]
        entry = dict(n=len(group), survived=sum(r['survived'] for r in group))
        for key in ('cmd_hf_rms', 'cmd_std', 'cmd_tv', 'pitch_std_deg', 'rate_std_dps', 'vel_mae', 'yaw_mae', 'pos_final'):
            vals = [r[key] for r in ok]
            if vals:
                entry[key] = dict(median=float(np.median(vals)), p95=float(np.percentile(vals, 95)))
        out[scenario] = entry
    return out


def print_table(summary):
    head = f"{'scenario':14s} {'surv':>6s} {'hf_rms':>7s} {'cmd_std':>8s} {'cmd_tv/s':>9s} {'pitch_sd':>8s} {'rate_sd':>8s} {'vel_mae':>8s} {'yaw_mae':>8s} {'pos_end':>8s}"
    print(head)
    for name, e in summary.items():
        g = lambda k: e[k]['median'] if k in e else float('nan')
        print(f"{name:14s} {e['survived']:>3d}/{e['n']:<2d} {g('cmd_hf_rms'):7.1f} {g('cmd_std'):8.1f} {g('cmd_tv'):9.0f} {g('pitch_std_deg'):8.2f} {g('rate_std_dps'):8.1f} {g('vel_mae'):8.3f} {g('yaw_mae'):8.3f} {g('pos_final'):8.2f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', nargs='+', default=['linear'], help="linear | ppo PATH | native HEADER")
    parser.add_argument('--scenarios', nargs='+', default=list(SCENARIOS))
    parser.add_argument('--seeds', type=int, default=8)
    parser.add_argument('--seed-start', type=int, default=51000)
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--policy-every', type=int, default=1)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    spec = tuple(args.policy)
    jobs = [(spec, s, args.seed_start + i, args.seconds, args.policy_every) for s in args.scenarios for i in range(args.seeds)]
    began = time.monotonic()
    with Pool(args.workers) as pool:
        rows = pool.map(run_episode, jobs, chunksize=1)
    summary = summarize(rows)
    print_table(summary)
    print('wall seconds', round(time.monotonic() - began, 1))
    if args.output:
        args.output.write_text(json.dumps(dict(policy=list(spec), summary=summary, episodes=rows), indent=1) + '\n')


if __name__ == '__main__':
    main()
