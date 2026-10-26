"""Does the free-fall guard fire, and early enough, whatever attitude and speed the robot starts to fall with?  (2026-10-24)

Usage: .venv/bin/python tools/fall_angle_probe.py [release] [edge] [--quick]      (no argument = both)

The question (from the user): "a robot that drops off a table starts to fall at some angle; does the angle change what the IMU sees, so that at
some angles the servos are not unloaded although it fell?"  The guard (firmware/src/fall_guard.h) looks at |a| of the raw accelerometer, which
is ~0 g in free fall at ANY attitude; the angle only matters through (1) spin: the IMU sits ~41 mm above the centre of mass, so a body that spins
reads omega^2*r + alpha*r although it is falling, (2) the way the fall starts (edge contact, wheels still loaded), (3) the policy driving the wheels in
the air (reaction torque spins the body).  This probe puts those in a simulation with the REAL shell (tools/shell_model.py), the unchanged v6 policy,
a wheel motor that cannot deliver torque beyond its no-load speed (as tools/getup_probe.py), and replicas of the firmware's lift guard
(firmware/src/lift_guard.h) and free-fall guard (firmware/src/fall_guard.h: main 0.5 g / 60 ms, deep 0.061 g / 6 ms).  The 45 deg pitch / 60 deg roll tilt stop is also
checked.  Margin = time from "the stop is issued" (trip + 12 ms for the control tick and the bus) to the first contact with the floor.

release  The robot is held at pitch/roll in the air (a weld to a mocap body = the hand, wheels free), the policy runs for `hold` seconds
         (0.3 s: the policy still drives the wheels; 1.2 s: the lift guard has already zeroed them, as in the real hand drops), then the hand lets go
         and it falls 0.3 m.
edge     The robot balances on a table (height H), is told to drive at the edge at speed v and angle psi to the edge normal, and rolls off.

Results (2026-10-24): training/docs/REBUILD.zh-CN.md section 11.
"""
import sys, os, math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
import numpy as np
import mujoco
from multiprocessing import Pool
import shell_model
HDR = str(ROOT / 'artifacts/balance-v6/policy_weights.h')
G = 9.80665
STOP_LATENCY = 0.012      # s from the detector trip to the servos being released (control tick <= 6 ms + bus)
TILT_PITCH, TILT_ROLL = 45.0, 60.0


# ------------------------------------------------------------------------------------------------ firmware replicas
class Detector:
    """Port of rider2::fall::update (firmware/src/fall_guard.h)."""

    def __init__(self, freefall_g=0.5, freefall_ms=60.0, landed_g=0.8, landed_ms=40.0, max_dt_ms=30.0):
        self.fg, self.fms, self.lg, self.lms, self.max_dt = freefall_g, freefall_ms, landed_g, landed_ms, max_dt_ms
        self.low_ms = self.high_ms = 0.0
        self.falling = False
        self.trips = 0
        self.min_g = 1000.0

    def update(self, g, dt_ms):
        if not (math.isfinite(g) and 0.0 < dt_ms <= self.max_dt):
            self.low_ms = self.high_ms = 0.0
            return
        low = g < self.fg
        self.min_g = min(self.min_g, g)
        if low:
            self.low_ms += dt_ms
            self.high_ms = 0.0
        else:
            self.low_ms = 0.0
            self.high_ms = self.high_ms + dt_ms if g > self.lg else 0.0
        if not self.falling:
            if self.low_ms >= self.fms:
                self.falling = True
                self.trips += 1
        elif self.high_ms >= self.lms:
            self.falling = False
            self.high_ms = 0.0


def deep_detector():
    return Detector(freefall_g=1000.0 / 16384.0, freefall_ms=6.0)


class LiftGuard:
    """Port of rider2::lift::update (firmware/src/lift_guard.h) with its default config."""
    fast_speed, fast_ticks, calm_speed, calm_rate = 300.0, 6, 30.0, 8.0
    up_pitch, up_roll, calm_ticks, retrip_window, retrip_calm = 8.0, 5.0, 60, 200, 100

    def __init__(self):
        self.active = False
        self.fast_count = self.calm_count = 0
        self.calm_needed = 60
        self.since_release = 65535
        self.trips = 0

    def update(self, s11, s21, pitch, roll, pitch_rate, roll_rate):
        common, diff = 0.5 * (s11 + s21), 0.5 * (s11 - s21)
        speed = max(abs(common), abs(diff))
        fast = speed > self.fast_speed
        if not self.active:
            self.since_release = min(self.since_release + 1, 65535)
            self.fast_count = min(self.fast_count + 1, 65535) if fast else 0
            if self.fast_count >= self.fast_ticks:
                self.calm_needed = self.retrip_calm if self.since_release <= self.retrip_window else self.calm_ticks
                self.active = True
                self.calm_count = 0
                self.trips += 1
        else:
            calm = (speed < self.calm_speed and abs(pitch) < self.up_pitch and abs(roll) < self.up_roll and
                    abs(pitch_rate) < self.calm_rate and abs(roll_rate) < self.calm_rate)
            self.calm_count = self.calm_count + 1 if calm else 0
            if self.calm_count >= self.calm_needed:
                self.active = False
                self.fast_count = self.calm_count = 0
                self.since_release = 0


# ------------------------------------------------------------------------------------------------ model and environment
def write_xml(path, floor_depth, edge=None, imu_pos=(0., 0., -.005)):
    """The shell model with the floor `floor_depth` m below the start plane, an optional table (top at z=0, edge at distance d along direction
    psi from the start) and a mocap 'hand' welded to the base on demand."""
    text = Path(shell_model.write_shell_xml(f'/tmp/rider_shell_{os.getpid()}.xml')).read_text()
    old = '<geom name="ground" type="plane" size="8 8 .1" contype="2" conaffinity="1" material="floor" />'
    assert old in text
    new = f'<geom name="ground" type="plane" pos="0 0 {-floor_depth:.4f}" size="8 8 .1" contype="2" conaffinity="1" material="floor" />\n'
    if edge:
        d, psi = edge
        n = np.array([math.cos(psi), math.sin(psi)])
        center = d * n - 2.0 * n
        new += (f'    <geom name="table" type="box" pos="{center[0]:.5f} {center[1]:.5f} -0.025" euler="0 0 {math.degrees(psi):.4f}" '
                f'size="2 2 .025" contype="2" conaffinity="1" material="floor" />\n')
    new += '    <body name="hand" mocap="true" pos="0 0 0"><geom type="sphere" size=".001" contype="0" conaffinity="0" rgba="0 0 0 0" /></body>\n'
    text = text.replace(old, new, 1)
    site = '<site name="imu" pos="0 0 -.005" size=".002" />'
    assert site in text
    text = text.replace(site, f'<site name="imu" pos="{imu_pos[0]} {imu_pos[1]} {imu_pos[2]}" size=".002" />', 1)
    text = text.replace('</worldbody>', '</worldbody>\n  <equality><weld name="grip" body1="hand" body2="base" relpose="0 0 0 1 0 0 0" active="false" solref=".004 1" /></equality>', 1)
    Path(path).write_text(text)
    return path


def motor_model(env, omega_nl, cap):
    """Wheel torque is full up to half the no-load speed and tapers to zero at it (only while driving along the wheel's motion), capped at `cap`."""
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


class Run:
    """One simulated fall: builds the env, records |a| at every 2 ms physics step and the state at every control tick."""

    def __init__(self, domain, seed, floor_depth, edge=None, omega_nl=42., cap=0.13, lift_guard=True, pitch=0., roll=0., imu_pos=(0., 0., -.005)):
        from rider_sim.env2 import RiderEnv2
        from rider_sim.native_policy import NativePolicy
        shell_model.patch_loader(write_xml(f'/tmp/rider_fall_{os.getpid()}.xml', floor_depth, edge, imu_pos))
        self.native = NativePolicy(HDR)
        self.env = env = RiderEnv2(domain=domain, commands=False, pushes=False, episode_seconds=120, native_policy=self.native, noise=1.0)
        env.reset(seed=seed, options={'pitch': math.radians(pitch), 'roll': math.radians(roll), 'yaw': 0.})
        motor_model(env, omega_nl, cap)
        m = env.model
        self.robot_geoms = {m.geom(n).id for n in ('chassis', 'hull12', 'hull22', 'tyre11', 'tyre21')}
        self.floor_id = m.geom('ground').id
        self.table_id = m.geom('table').id if edge else None
        self.imu_site = m.site('imu').id
        self.lift = LiftGuard()
        self.lift_enabled = lift_guard
        self.acc = []           # (t, |a| in g) at every physics step
        self.acc3 = []          # (t, ax, ay, az [m/s^2, sensor frame], gx, gy, gz [rad/s]) at every physics step
        self.rows = []          # per control tick: t, pitch, roll, pitch_rate, roll_rate, z, vz, vx, wheel speeds, lift active, table contact, floor contact
        self.fault = None
        self._wrap()

    def _wrap(self):
        env = self.env
        sub = env._physics_substep

        def substep():
            sub()
            a = np.array(env.data.sensordata[:3])
            if env.noise:
                a = a + env.accel_bias + env.np_random.normal(0, env.accel_noise, 3)
            self.acc.append((env.us * 1e-6, float(np.linalg.norm(a)) / G))
            self.acc3.append((env.us * 1e-6, *map(float, a), *map(float, env.data.sensordata[3:6])))
        env._physics_substep = substep
        apply = env._apply_action

        def patched():
            if self.lift_enabled and self.lift.active:
                self.native.wheels[:] = 0.
            apply()
        env._apply_action = patched

    def contacts(self):
        d = self.env.data
        table = floor = False
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist > 0.0005:
                continue
            if c.geom1 in self.robot_geoms:
                other = c.geom2
            elif c.geom2 in self.robot_geoms:
                other = c.geom1
            else:
                continue
            if other == self.floor_id:
                floor = True
            elif other == self.table_id:
                table = True
        return table, floor

    def step(self):
        env = self.env
        try:
            env.step(self.native.action.copy())
        except RuntimeError as e:
            self.fault = str(e)
            return False
        d = env.data
        roll, pitch, yaw = env._angles()
        out = env.out
        s11, s21 = float(env.feedback[0]), float(env.feedback[1])
        pr = float(np.degrees(out[3])) if np.isfinite(out[3]) else 0.
        rr = float(np.degrees(out[4])) if np.isfinite(out[4]) else 0.
        self.lift.update(s11, s21, math.degrees(pitch), math.degrees(roll), pr, rr)
        table, floor = self.contacts()
        r_body = d_r = env.data.xmat[env.base].reshape(3, 3).T @ (env.data.site_xpos[self.imu_site] - env.data.subtree_com[0])
        self.rows.append(dict(t=env.elapsed, pitch=math.degrees(pitch), roll=math.degrees(roll), fw_pitch=float(out[0]), fw_roll=float(out[1]), pr=pr, rr=rr,
                              z=float(d.qpos[2]), vz=float(d.qvel[2]), vh=float(math.hypot(d.qvel[0], d.qvel[1])),
                              s11=s11, s21=s21, lift=self.lift.active, table=table, floor=floor,
                              gyro=float(np.linalg.norm(d.sensordata[3:6])), r=r_body.tolist()))
        return True

    def close(self):
        mujoco.set_mjcb_control(None)
        self.env.close()
        self.native.close()


def judge(run, t_leave, t_floor, rng, acc=None):
    """Replay the detectors over the recorded |a| (or the series `acc` = (t, g)); returns the trip times and the margin to the floor."""
    acc = np.array(run.acc) if acc is None else acc
    t0 = max(t_leave - 0.6, 0.)
    sel = acc[(acc[:, 0] >= t0) & (acc[:, 0] <= t_floor + 0.05)]
    main, deep = Detector(), deep_detector()
    t_main = t_deep = None
    i, last = 0, None
    gaps = rng.choice([2, 4, 4, 6], size=len(sel)) * 1e-3     # the IMU is read every 2..6 ms (measured 2.0 / 3.35 / 6.0)
    t = sel[0, 0] if len(sel) else 0.
    k = 0
    times, gs = sel[:, 0], sel[:, 1]
    while len(sel) and t <= times[-1] and k < len(gaps):
        j = int(np.searchsorted(times, t, side='right')) - 1
        g = float(gs[max(j, 0)])
        dt = 1e3 * (t - last) if last is not None else 5.0
        main.update(g, dt)
        deep.update(g, dt)
        if main.falling and t_main is None:
            t_main = t
        if deep.falling and t_deep is None:
            t_deep = t
        last = t
        t += gaps[k]
        k += 1
    trips = [x for x in (t_main, t_deep) if x is not None]
    t_ff = min(trips) if trips else None
    t_tilt = None
    for r in run.rows:     # the stop uses the firmware's fused estimate (src/runtime_guard.cpp), not the true angle
        if r['t'] >= t_leave - 0.2 and (abs(r['fw_pitch']) > TILT_PITCH or abs(r['fw_roll']) > TILT_ROLL):
            t_tilt = r['t']
            break
    stops = [x for x in (t_ff, t_tilt) if x is not None]
    t_stop = min(stops) + STOP_LATENCY if stops else None
    flight = acc[(acc[:, 0] >= t_leave) & (acc[:, 0] <= t_floor)]
    return dict(t_ff=t_ff, t_main=t_main, t_deep=t_deep, t_tilt=t_tilt, t_stop=t_stop,
                margin=(t_floor - t_stop) if t_stop is not None else None,
                margin_ff=(t_floor - (t_ff + STOP_LATENCY)) if t_ff is not None else None,
                min_g=float(flight[:, 1].min()) if len(flight) else None,
                g_mean=float(flight[:, 1].mean()) if len(flight) else None,
                g_p90=float(np.percentile(flight[:, 1], 90)) if len(flight) else None)


R_NOMINAL = np.array([-0.0021, 0.0, 0.0398])    # IMU - system CoM in the body frame for the model's IMU site (what the firmware could hard-code)


def compensated(run, r, alpha=False):
    """|f - omega x (omega x r) [- alpha x r]| / g over the run: what the free-fall guard would see if it removed the spin's centripetal part."""
    a3 = np.array(run.acc3)
    f, w = a3[:, 1:4], a3[:, 4:7]
    pred = np.cross(w, np.cross(w, r))
    if alpha:     # alpha from a 4-step (8 ms) difference of the gyro
        dw = np.zeros_like(w)
        dw[4:] = (w[4:] - w[:-4]) / 0.008
        pred = pred + np.cross(dw, r)
    return np.column_stack([a3[:, 0], np.linalg.norm(f - pred, axis=1) / G])


# ------------------------------------------------------------------------------------------------ experiment 1: release from the hand
def release_run(a):
    pitch, roll, hold, domain, seed, lift = a
    os.chdir(ROOT)
    run = Run(domain, seed, floor_depth=0.30, lift_guard=lift, pitch=pitch, roll=roll)
    env, d = run.env, run.env.data
    d.mocap_pos[0] = d.qpos[0:3]
    d.mocap_quat[0] = d.qpos[3:7]
    d.eq_active[0] = 1
    mujoco.mj_forward(env.model, d)
    t_release = None
    ok = True
    while ok and env.elapsed < hold + 0.9:
        if t_release is None and env.elapsed >= hold:
            d.eq_active[0] = 0
            t_release = env.elapsed
        ok = run.step()
        if t_release is not None and run.rows and run.rows[-1]['floor']:
            break
    rows = run.rows
    t_floor = next((r['t'] for r in rows if r['t'] > (t_release or 1e9) and r['floor']), None)
    result = dict(a=a, fault=run.fault)
    if t_release is not None and t_floor is not None:
        rng = np.random.default_rng(seed)
        j = judge(run, t_release, t_floor, rng)
        at_release = next(r for r in rows if r['t'] >= t_release)
        result.update(j, flight=t_floor - t_release, wheels_zeroed=bool(at_release['lift']),
                      s_release=max(abs(at_release['s11']), abs(at_release['s21'])),
                      gyro_max=max(r['gyro'] for r in rows if r['t'] >= t_release and r['t'] <= t_floor),
                      pitch_floor=rows[-1]['pitch'], roll_floor=rows[-1]['roll'])
    run.close()
    return result


# ------------------------------------------------------------------------------------------------ experiment 2: off the table edge
IMU_SITES = {'model': (0., 0., -.005),          # where the training model puts it: ~41 mm above the centre of mass
             'high-front': (.035, 0., .015),    # 6.5 cm from the centre of mass, towards the nose and the top
             'side': (0., .03, -.005),          # 5 cm from it, towards one wheel
             'low': (0., 0., -.040)}            # next to the centre of mass


def edge_run(a):
    """a = (table height, speed command m/s (negative = backwards), psi deg (direction of the edge normal from the robot's nose: 0 head-on, 180 backwards),
    domain, seed, lift guard, start pitch deg, IMU site name, push N (0 = drive at the edge with the speed command, else a 0.2 s pulse toward it))"""
    height, speed, psi_deg, domain, seed, lift, pitch0, imu, push = a
    os.chdir(ROOT)
    psi = math.radians(psi_deg)
    d_edge = 0.40 if push else (0.60 if psi_deg < 60 or psi_deg > 100 else 0.45)     # the robot drifts ~15-25 cm before it settles; the command comes at 3 s
    run = Run(domain, seed, floor_depth=height, edge=(d_edge, psi), lift_guard=lift, pitch=pitch0, imu_pos=IMU_SITES[imu])
    env, d = run.env, run.env.data
    ok = True
    t_floor = None
    t_cmd = 3.0
    pushed = False
    n = np.array([math.cos(psi), math.sin(psi)])
    while ok and env.elapsed < 30.:
        if env.elapsed >= t_cmd:
            if push:
                d.xfrc_applied[run.env.base, :2] = push * n if env.elapsed < t_cmd + 0.2 else 0.
            else:
                env.command[0] = int(round(127.5 + speed / .21 * 127.5))
        ok = run.step()
        if run.rows[-1]['floor']:
            t_floor = run.rows[-1]['t']
            break
    rows = run.rows
    result = dict(a=a, fault=run.fault, t_floor=t_floor)
    if t_floor is not None:
        # leaving the table = the last control tick with a table contact before the floor contact
        t_leave = max((r['t'] for r in rows if r['table'] and r['t'] < t_floor), default=None)
        if t_leave is not None:
            rng = np.random.default_rng(seed)
            j = judge(run, t_leave, t_floor, rng)
            r_true = np.mean([r['r'] for r in rows if t_leave - 0.3 <= r['t'] <= t_leave], axis=0)
            comp = {}
            for name, series in (('c_true', compensated(run, r_true)), ('c_nom', compensated(run, R_NOMINAL)), ('c_nom_a', compensated(run, R_NOMINAL, alpha=True))):
                jc = judge(run, t_leave, t_floor, np.random.default_rng(seed), acc=series)
                comp[name] = dict(t_ff=jc['t_ff'], margin=jc['margin'], margin_ff=jc['margin_ff'], p90=jc['g_p90'])
            at_leave = next(r for r in rows if r['t'] >= t_leave)
            approach = [r['vh'] for r in rows if t_leave - 0.6 <= r['t'] <= t_leave - 0.3]
            result.update(j, comp=comp, r_true=r_true.tolist(), t_leave=t_leave, flight=t_floor - t_leave, vh=at_leave['vh'], vz_floor=rows[-1]['vz'],
                          v_approach=float(np.mean(approach)) if approach else None,
                          pitch_leave=at_leave['pitch'], roll_leave=at_leave['roll'], lift_at_leave=bool(at_leave['lift']),
                          gyro_max=max(r['gyro'] for r in rows if t_leave <= r['t'] <= t_floor),
                          gyro_early=max(r['gyro'] for r in rows if t_leave <= r['t'] <= t_leave + 0.1),
                          pitch_floor=rows[-1]['pitch'], roll_floor=rows[-1]['roll'],
                          early=bool(j['t_ff'] is not None and j['t_ff'] < t_leave - 0.02))
    run.close()
    return result


# ------------------------------------------------------------------------------------------------ reports
def ms(x):
    return '   -  ' if x is None else f'{x * 1000:5.0f}'


def release_report(rows):
    print('A. released from the hand at attitude (pitch, roll), 0.3 m above the floor (246 ms of flight); cells: worst margin of the FREE-FALL GUARD alone [ms] from the stop being issued')
    print('   (trip + 12 ms) to the first floor contact (negative = too late, never = did not trip)')
    for hold, lift in ((0.3, True), (1.2, True), (0.3, False)):
        sel = [r for r in rows if r['a'][2] == hold and r['a'][5] == lift]
        print(f"  hold {hold:.1f} s, lift guard {'on' if lift else 'off'} ({'wheels still driven at release' if hold < 1 else 'wheels already zeroed by the lift guard'}):")
        pitches = sorted({r['a'][0] for r in sel})
        rolls = sorted({r['a'][1] for r in sel})
        print('    roll\\pitch ' + ''.join(f'{p:>10.0f}' for p in pitches))
        for ro in rolls:
            cells = []
            for p in pitches:
                rr = [r for r in sel if r['a'][0] == p and r['a'][1] == ro and r.get('margin') is not None]
                if not rr:
                    cells.append(f'{"n/a":>10s}')
                    continue
                # a robot HELD beyond the tilt limit has been stopped by the tilt guard before it is let go: only the free-fall guard is of interest here
                ff = [r['margin_ff'] for r in rr]
                cells.append(f"{'never':>10s}" if any(x is None for x in ff) else f"{min(ff) * 1000:10.0f}")
            print(f'    {ro:9.0f}  ' + ''.join(cells))
        mg = [r['margin'] for r in sel if r.get('margin') is not None]
        late = [r for r in sel if r.get('margin') is None or r['margin'] < 0.05]
        print(f"    worst margin {min(mg) * 1000:.0f} ms, median {np.median(mg) * 1000:.0f} ms; runs with margin < 50 ms: {len(late)}/{len(sel)}; "
              f"free-fall guard alone: worst {min(r['margin_ff'] for r in sel if r.get('margin_ff') is not None) * 1000:.0f} ms, "
              f"never tripped in {sum(1 for r in sel if r.get('t_ff') is None)}/{len(sel)}; min |a| median {np.median([r['min_g'] for r in sel if r.get('min_g') is not None]):.2f} g, "
              f"p90 |a| median {np.median([r['g_p90'] for r in sel if r.get('g_p90') is not None]):.2f} g, peak gyro median {np.median([r['gyro_max'] for r in sel if r.get('gyro_max') is not None]):.1f} rad/s")
    print()


def edge_report(rows, title='B. rolled off a table edge'):
    print(title + ': cells = worst margin to the floor [ms] of the seeds with the first detector ("F" free-fall guard / "T" tilt stop), then the margin of the')
    print('   free-fall guard ALONE ("never" = it did not trip in at least one seed), and (seeds that fell / seeds run)')
    for height in sorted({r['a'][0] for r in rows}):
        sel = [r for r in rows if r['a'][0] == height]
        print(f"  table {height:.2f} m:")
        cols = sorted({(r['a'][1], r['a'][8]) for r in sel})
        psis = sorted({r['a'][2] for r in sel})
        print('    psi\\command ' + ''.join(f"{('push %.1f N' % c[1]) if c[1] else ('%+.2f m/s' % c[0]):>22s}" for c in cols))
        for psi in psis:
            cells = []
            for c in cols:
                rr = [r for r in sel if (r['a'][1], r['a'][8]) == c and r['a'][2] == psi]
                if not rr:
                    cells.append(f'{"":>22s}')
                    continue
                ok = [r for r in rr if r.get('margin') is not None]
                if not ok:
                    cells.append(f'{"no fall":>22s}')
                    continue
                worst = min(ok, key=lambda r: r['margin'])
                first = 'F' if (worst['t_ff'] is not None and (worst['t_tilt'] is None or worst['t_ff'] <= worst['t_tilt'])) else 'T'
                ff = [r['margin_ff'] for r in ok]
                ffs = 'never' if any(x is None for x in ff) else f'{min(ff) * 1000:.0f}'
                cells.append(f"{worst['margin'] * 1000:4.0f}{first}/ff {ffs} ({len(ok)}/{len(rr)})")
            print(f'    {psi:9.0f}    ' + ''.join(f'{c:>22s}' for c in cells))
        ok = [r for r in sel if r.get('margin') is not None]
        if ok:
            print(f"    flight time median {np.median([r['flight'] for r in ok]) * 1000:.0f} ms; worst margin {min(r['margin'] for r in ok) * 1000:.0f} ms; "
                  f"margin < 50 ms: {sum(1 for r in ok if r['margin'] < 0.05)}/{len(ok)}; stop never issued: {sum(1 for r in ok if r['margin'] is None)}")
    print()


def sweep_report(rows):
    print('C. summary by IMU position (the real one is not known): runs that fell off the table; the free-fall guard alone / the tilt stop alone / either, and the margin')
    print('   IMU site        runs  ff trips   tilt trips  either   worst margin   ff alone never  margin<50ms   |a| p90 median  peak spin (rad/s) median / p90 / max')
    for site in IMU_SITES:
        sel = [r for r in rows if r['a'][7] == site and r.get('flight') is not None]
        if not sel:
            continue
        n = len(sel)
        ff = sum(1 for r in sel if r['t_ff'] is not None)
        tl = sum(1 for r in sel if r['t_tilt'] is not None)
        ei = sum(1 for r in sel if r['t_stop'] is not None)
        margins = [r['margin'] for r in sel if r['margin'] is not None]
        spins = [r['gyro_max'] for r in sel]
        print(f"   {site:14s} {n:5d}  {ff:5d}      {tl:5d}      {ei:5d}    {min(margins) * 1000:6.0f} ms      {n - ff:5d}         {sum(1 for r in sel if r['margin'] is None or r['margin'] < 0.05):5d}"
              f"          {np.median([r['g_p90'] for r in sel]):.2f} g        {np.median(spins):4.1f} / {np.percentile(spins, 90):4.1f} / {max(spins):4.1f}")
    print('   by table height (all IMU sites): ' + '; '.join(
        f"{h:.2f} m: flight {np.median([r['flight'] for r in rows if r['a'][0] == h and r.get('flight')]) * 1000:.0f} ms, worst margin "
        f"{min(r['margin'] for r in rows if r['a'][0] == h and r.get('margin') is not None) * 1000:.0f} ms" for h in sorted({r['a'][0] for r in rows})))
    bad = sorted([r for r in rows if r.get('flight') is not None and (r['margin'] is None or r['margin'] < 0.05)], key=lambda r: (r['margin'] if r['margin'] is not None else -1))
    print(f'   runs with margin < 50 ms or no stop: {len(bad)}')
    for r in bad[:12]:
        print('     ', r['a'], 'flight %.0f ms' % (r['flight'] * 1000), 'margin', None if r['margin'] is None else round(r['margin'] * 1000), 'ff', r['t_ff'] is not None, 'tilt', r['t_tilt'] is not None, 'spin %.1f' % r['gyro_max'])
    blind = [r for r in rows if r.get('flight') is not None and r['t_ff'] is None]
    print(f'   runs where the free-fall guard never tripped: {len(blind)}; their peak spin median {np.median([r["gyro_max"] for r in blind]) if blind else 0:.1f} rad/s; '
          f'tilt stop margin median {np.median([r["margin"] for r in blind if r["margin"] is not None]) * 1000 if blind else 0:.0f} ms')
    print()


def compare_report(rows):
    print('D. spin compensation in the free-fall guard: f - omega x (omega x r) with r = true IMU offset ("true r"), r = the model\'s offset used for every site ("nominal r"),')
    print('   plus alpha x r from the gyro ("nominal r + alpha"); margins in ms (negative / "no stop" = the servos were still powered at the landing)')
    variants = [('baseline', None), ('true r', 'c_true'), ('nominal r', 'c_nom'), ('nominal r + alpha', 'c_nom_a')]

    def get(r, key):
        if key is None:
            return r['t_ff'], r['margin_ff'], r['margin']
        c = r['comp'][key]
        return c['t_ff'], c['margin_ff'], c['margin']
    for site in IMU_SITES:
        for height in sorted({r['a'][0] for r in rows}):
            sel = [r for r in rows if r['a'][7] == site and r['a'][0] == height and r.get('flight') is not None]
            if not sel:
                continue
            parts = []
            for name, key in variants:
                g = [get(r, key) for r in sel]
                blind = sum(1 for t, mf, m in g if t is None)
                worst_ff = min((mf for t, mf, m in g if mf is not None), default=None)
                worst = min((m for t, mf, m in g if m is not None), default=None)
                none = sum(1 for t, mf, m in g if m is None)
                parts.append(f"{name}: ff blind {blind:3d}/{len(sel)}, ff-alone worst {'-' if worst_ff is None else format(worst_ff * 1000, '5.0f')}, with tilt worst {'-' if worst is None else format(worst * 1000, '5.0f')}, no stop {none}")
            print(f'  IMU {site:10s} table {height:.2f} m: ' + ' | '.join(parts))
    print()


def height_report(rows):
    print('E. by table height (all IMU sites, all approaches): how many falls ended with the servos still powered at the landing, and by which layer they were stopped')
    print('   height  falls  flight median  stopped >=100 ms before  50-100 ms  <50 ms  not stopped   free-fall guard blind   stopped by the tilt stop only   worst margin')
    for h in sorted({r['a'][0] for r in rows}):
        sel = [r for r in rows if r['a'][0] == h and r.get('flight') is not None]
        m = [r['margin'] for r in sel]
        none = sum(1 for x in m if x is None)
        ge100 = sum(1 for x in m if x is not None and x >= 0.10)
        mid = sum(1 for x in m if x is not None and 0.05 <= x < 0.10)
        low = sum(1 for x in m if x is not None and x < 0.05)
        blind = sum(1 for r in sel if r['t_ff'] is None)
        tilt_only = sum(1 for r in sel if r['t_ff'] is None and r['t_tilt'] is not None)
        worst = min((x for x in m if x is not None), default=None)
        print(f"   {h:.2f} m {len(sel):5d}   {np.median([r['flight'] for r in sel]) * 1000:6.0f} ms      {ge100:8d}            {mid:6d}   {low:6d}   {none:8d}        {blind:8d}                  {tilt_only:8d}                 {worst * 1000:6.0f} ms")
    print()


def main():
    quick = '--quick' in sys.argv
    todo = [x for x in sys.argv[1:] if not x.startswith('--')] or ['release', 'edge', 'sweep']
    with Pool(9) as p:
        if 'release' in todo:
            pitches = (-60., -30., 0., 30., 60., 90.) if quick else (-90., -60., -45., -30., -15., 0., 15., 30., 45., 60., 90.)
            rolls = (0., 45., 90.) if quick else (0., 30., 60., 90., 135., 180.)
            seeds = range(1) if quick else range(2)
            jobs = [(pi, ro, hold, 1.0, s, lift) for hold, lift in ((0.3, True), (1.2, True), (0.3, False))
                    for pi in pitches for ro in rolls for s in seeds]
            release_report(p.map(release_run, jobs, chunksize=1))
        if 'edge' in todo:
            heights = (0.75,) if quick else (0.75, 0.30)
            speeds = (0.05, 0.12, 0.21)
            psis = (0., 30., 60.) if quick else (0., 20., 40., 60., 75.)
            seeds = range(2) if quick else range(4)
            jobs = [(h, v, psi, 1.0, s, True, -7.0, 'model', 0.) for h in heights for v in speeds for psi in psis for s in seeds]
            edge_report(p.map(edge_run, jobs, chunksize=1))
        if 'sweep' in todo:
            heights = (0.30,) if quick else (0.15, 0.30, 0.75)
            seeds = range(2) if quick else range(6)
            sites = ('model', 'high-front') if quick else tuple(IMU_SITES)
            combos = [(v, psi, 0.) for psi in (0., 20., 40., 60., 75.) for v in (0.12, 0.21)] + \
                     [(v, 180., 0.) for v in (-0.12, -0.21)] + [(0., psi, f) for psi in (0., 40.) for f in (2.0, 3.5)]
            jobs = [(h, v, psi, 1.0, s, True, -7.0, site, f) for site in sites for h in heights for (v, psi, f) in combos for s in seeds]
            rows = p.map(edge_run, jobs, chunksize=1)
            edge_report(rows, 'C0. sweep (all IMU sites pooled)') if False else None
            sweep_report(rows)
            compare_report(rows)
            height_report(rows)
            import json
            keep = ('a', 't_ff', 't_main', 't_deep', 't_tilt', 't_stop', 'margin', 'margin_ff', 'min_g', 'g_p90', 'flight', 'gyro_max', 'gyro_early', 'comp', 'r_true', 'pitch_leave', 'roll_leave', 'vh', 'v_approach')
            (ROOT / 'runs').mkdir(exist_ok=True)
            Path(ROOT / 'runs/fall_angle_sweep.json').write_text(json.dumps([{k: r.get(k) for k in keep} for r in rows], default=float))
            for site in sites:
                edge_report([r for r in rows if r['a'][7] == site], f'C1. sweep, IMU at "{site}" {IMU_SITES[site]}')


if __name__ == '__main__':
    main()
