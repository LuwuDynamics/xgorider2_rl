"""Can the robot stand up again from where it rests after a fall?  Simulator probe with the REAL shell as collision geometry (tools/shell_model.py:
convex hulls of the CAD body with its two front prongs and of the legs with their rear arms), the unchanged v6 policy, and a wheel motor that
cannot deliver torque beyond its no-load speed.

Why not the training model: its body is a 90 x 72 x 50 mm box with no front prongs and no rear arms, so after a fall it rests at +-77 deg on its
chassis edge with one tyre carrying 2.5 N, and nothing can lift it (first answer 2026-10-22: "needs a tail", wrong).  On the real shell it rests
at about +32 deg (nose prongs + both tyres, legs in), +46..56 deg (legs out) or -36 deg (rear arms + both tyres).

Usage: .venv/bin/python tools/getup_probe.py [--quick]
Result (2026-10-22): training/docs/REBUILD.zh-CN.md section 10.

Motor model used here: the simulator's wheel is a pure torque source (0.19 N*m at full command, no speed limit), which would let a wheel spin up to
hundreds of rad/s while pushing on the body like a reaction wheel.  Here torque is full up to half the no-load speed (400 RPM = 42 rad/s per the
official spec) and tapers to zero at the no-load speed, only while driving along the wheel's motion; optionally capped at 0.13 N*m (rated).
"""
import sys, os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
import numpy as np
import mujoco
from multiprocessing import Pool
import shell_model
HDR = str(ROOT / 'artifacts/balance-v6/policy_weights.h')


def build(domain, seed, native_policy):
    shell_model.patch_loader(shell_model.write_shell_xml(f'/tmp/rider_shell_{os.getpid()}.xml'))
    from rider_sim.env2 import RiderEnv2
    env = RiderEnv2(domain=domain, commands=False, pushes=False, episode_seconds=60, native_policy=native_policy, noise=1.0)
    env.reset(seed=seed, options={'pitch': 0., 'roll': 0., 'yaw': 0.})
    return env


def place(env, pitch_deg, legs):
    """Robot at the given pitch with the tyres on the ground and the legs held at `legs` (m, relative to the firmware target)."""
    from rider_sim.env import quaternion
    m, d = env.model, env.data
    d.qpos[3:7] = quaternion(0, np.radians(pitch_deg), 0)
    d.qvel[:] = 0
    d.qpos[env.leg_q] = legs
    mujoco.mj_forward(m, d)
    d.qpos[2] += 0.0308 - min(d.xpos[b, 2] for b in env.wheel_bodies) + 0.0004
    mujoco.mj_forward(m, d)


def hold_legs(env, legs, awake):
    orig = env._apply_action

    def patched():
        saved = env.out[16:18].copy()
        env.out[16:18] = saved + (0. if awake[0] else np.array([legs, legs]))
        orig()
        env.out[16:18] = saved
    env._apply_action = patched


def motor_model(env, omega_nl, cap):
    if not (omega_nl or cap < 0.2):
        return
    omega_nl = omega_nl or 1e9
    knee = 0.5 * omega_nl
    wv = env.wheel_v

    def cb(model, data):
        w = data.qvel[wv]
        tau = data.ctrl[:2].copy()
        f = np.where(tau * w > 0, np.clip((omega_nl - np.abs(w)) / (omega_nl - knee), 0, 1), 1.0)
        data.ctrl[:2] = np.clip(tau * f, -cap, cap)
    mujoco.set_mjcb_control(cb)


def rest_pose(a):
    start, legs = a
    os.chdir(ROOT)
    env = build(0., 0, None)
    awake = [False]
    hold_legs(env, legs, awake)
    place(env, start, legs)
    for _ in range(500):
        env.step(np.zeros(4, np.float32))
    m, d = env.model, env.data
    forces = {}
    for c in range(d.ncon):
        con = d.contact[c]
        fr = np.zeros(6)
        mujoco.mj_contactForce(m, d, c, fr)
        for g in (con.geom1, con.geom2):
            name = m.geom(g).name or f'geom{g}'
            if name != 'ground':
                forces[name] = forces.get(name, 0.) + fr[0]
    return float(np.degrees(env._angles()[1])), {k: round(float(v), 2) for k, v in forces.items()}


def standup(a):
    start, legs, follow, mu, domain, seed, omega_nl, cap = a
    os.chdir(ROOT)
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HDR)
    env = build(domain, seed, native)
    m, d = env.model, env.data
    if mu:
        m.geom_friction[:, 0] = mu
    awake = [False]
    hold_legs(env, legs, awake)
    motor_model(env, omega_nl, cap)
    env.native_policy = None                     # resting, powered off
    place(env, start, legs)
    for _ in range(400):
        env.step(np.zeros(4, np.float32))
    rest = float(np.degrees(env._angles()[1]))
    env.firmware.reset(rest, 0.)
    env.native_policy = native
    native.reset()
    awake[0] = follow                            # legs follow the firmware's standing height once the policy is on
    if not follow:
        awake[0] = False
    t0 = env.elapsed
    upright_at, peak_after, fell = None, 0., False
    for _ in range(1400):                        # 8.4 s
        _, _, _, _, info = env.step(native.action.copy())
        p = float(np.degrees(info['pitch']))
        if upright_at is None and abs(p) < 10:
            upright_at = env.elapsed - t0
        if upright_at is not None:
            peak_after = max(peak_after, abs(p))
        if abs(p) > 70:
            fell = True
            break
    final = float(np.degrees(env._angles()[1]))
    travel = float(np.hypot(d.qpos[0], d.qpos[1]))
    mujoco.set_mjcb_control(None)
    env.close(); native.close()
    return dict(rest=rest, ok=bool((not fell) and abs(final) < 15 and upright_at is not None), t_up=upright_at, peak=peak_after, travel=travel)


def summary(rows):
    ok = [r for r in rows if r['ok']]
    tilt_ok = [r for r in rows if r['ok'] and r['peak'] < 45]
    return (f"success {len(ok)}/{len(rows)}, with the firmware's 45 deg tilt stop {len(tilt_ok)}/{len(rows)}; "
            f"time to upright {np.mean([r['t_up'] for r in ok] or [np.nan]):.2f} s; overshoot past upright median {np.median([r['peak'] for r in rows]):.0f} deg "
            f"(90th percentile {np.percentile([r['peak'] for r in rows], 90):.0f}); travelled median {np.median([r['travel'] for r in rows]) * 100:.0f} cm, "
            f"max {np.max([r['travel'] for r in rows]) * 100:.0f} cm")


if __name__ == '__main__':
    quick = '--quick' in sys.argv
    with Pool(9) as p:
        print('A. where the robot rests after a fall (real shell hulls; legs held at the given extension relative to the standing height)')
        for name, start in (('forward', 30.), ('backward', -20.)):
            for legs in (-0.018, 0.0, 0.018):
                pitch, forces = p.apply(rest_pose, ((start, legs),))
                print(f'   fell {name}, legs {legs * 1000:+3.0f} mm: rests at pitch {pitch:+6.1f} deg, support N {forces}')
        print()
        poses = [('forward, legs in', 30., -0.018, True), ('backward, legs in', -20., -0.018, True), ('forward, legs out', 30., 0.018, False), ('backward, legs out', -20., 0.018, False)]
        print('B. v6 policy switched on at the resting pose, nominal plant, wheel torque cap 0.13 N*m, 401 RPM ceiling, friction mu varied (3 runs each)')
        for name, start, legs, follow in poses:
            for mu in (0.6, 0.9, 1.2):
                rows = p.map(standup, [(start, legs, follow, mu, 0., s, 42., 0.13) for s in range(3)])
                print(f'   {name:20s} mu {mu}: rests at {np.mean([r["rest"] for r in rows]):+5.1f} deg: ' + summary(rows))
        print()
        n = 8 if quick else 24
        print(f'C. same on the randomised plant (domain 1: masses, friction, delay, IMU noise), {n} seeds each; wheel torque cap 0.13 N*m, 401 RPM ceiling')
        for name, start, legs, follow in poses:
            rows = p.map(standup, [(start, legs, follow, 0., 1., s, 42., 0.13) for s in range(n)])
            print(f'   {name:20s}: rests at {np.mean([r["rest"] for r in rows]):+5.1f} deg: ' + summary(rows))
        print()
        print('D. sensitivity of case C "forward, legs in" to the motor: no-load speed 286 / 401 / unlimited, torque cap 0.10 / 0.13 / 0.20')
        for omega_nl in (30., 42., 0.):
            for cap in (0.10, 0.13, 0.20):
                rows = p.map(standup, [(30., -0.018, True, 0., 1., s, omega_nl, cap) for s in range(n)])
                print(f'   no-load {"unlimited" if not omega_nl else str(int(omega_nl * 9.55)) + " RPM":>9s}, cap {cap:.2f}: ' + summary(rows))
