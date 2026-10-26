"""Can the legs help the robot over a step?  Simulator probe, v6 policy (wheels only, never trained on steps) plus scripted leg motion.

The legs of this robot are two vertical slides (extension 45..81 mm, i.e. +-18 mm around the standing height) 86.6 mm apart: they cannot swing
forward, they only raise or lower their side of the body.  What a leg can still do for a step:

  plain   the baseline; alpha > 0 approaches the step diagonally, so one wheel meets the edge before the other
  lean    while the wheel that has to climb is at the edge, roll the body away from it (its leg out, the other leg in) so that the wheel
          carries less of the weight; moves on to the second wheel when the first is up
  hop     pull the leg of a wheel in by 18 mm just before the tyre reaches the edge (the wheel is lifted for a moment, the body is
          not held up by it), then let it back out -- the "step out a leg" idea.  The simulated leg is a first-order lag of 20 ms
          with a 15 N force limit, i.e. faster than the real servo, so this is an upper bound for the idea.

  high    the body raised by the legs for the whole run (legs out by 11 / 18 mm)
  ramp    not a leg trick: a wedge of a given slope in front of the step

The firmware levels the body with the legs (roll PID, on by default); for 'hop' and 'lean' the script therefore replaces the firmware's leg targets
by the standing height at t = 2 s plus the script, which is what the firmware does with levelling switched off (`!leveling 0`).

Usage: .venv/bin/python tools/step_probe_legs.py [--quick] [--random]
  --random   also repeat the ramps and the plain steps on the randomised plant (domain 1.0, 16 seeds x 2 speeds per row)

Passed = both wheel centres on top of the step (z >= r + h - 4 mm, x past the edge) once both wheels are 10 cm past the obstacle (25 s limit) and the robot never fell.
"""
import sys, os, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from multiprocessing import Pool
import mujoco
_ORIG_FROM_XML = mujoco.MjModel.from_xml_path   # the original, taken once per process (never wrap a wrapper)
HDR = str(ROOT / 'artifacts/balance-v6/policy_weights.h')
STEP_X = 0.25
RADIUS = 0.0308


def make_xml(h_mm, ramp_deg=0.):
    """A box step h_mm high at x = STEP_X; with ramp_deg > 0 a wedge of that slope leads up to it."""
    text = (ROOT / 'models/rider_measured.xml').read_text()
    h = h_mm / 1000.
    run_in = 0.
    ramp = ''
    if ramp_deg and h_mm > 0:
        b = np.radians(ramp_deg)
        length = h / np.sin(b)
        thick = 0.5
        cx = STEP_X + np.cos(b) * length / 2 + np.sin(b) * thick / 2
        cz = np.sin(b) * length / 2 - np.cos(b) * thick / 2
        ramp = f'<geom name="ramp" type="box" pos="{cx} 0 {cz}" euler="0 {-b} 0" size="{length / 2} 1.5 {thick / 2}" contype="2" conaffinity="1" material="floor" />\n'
        run_in = length * np.cos(b)
    step = f'<geom name="step" type="box" pos="{STEP_X + run_in + 1.0} 0 {h / 2}" size="1.0 1.5 {h / 2}" contype="2" conaffinity="1" material="floor" />\n'
    return text.replace('<geom name="ground"', ramp + step + '    <geom name="ground"', 1) if h_mm > 0 else text


def run(a):
    h_mm, v, seed, alpha_deg, mode, amp = a[:6]
    ramp_deg = a[6] if len(a) > 6 else 0.
    domain = a[7] if len(a) > 7 else 0.
    os.chdir(ROOT)
    xml_path = Path(f'/tmp/rider_stepleg_{os.getpid()}_{h_mm}_{ramp_deg}.xml')
    xml_path.write_text(make_xml(h_mm, ramp_deg))
    mujoco.MjModel.from_xml_path = staticmethod(lambda p, *args, **kw: _ORIG_FROM_XML(str(xml_path) if 'rider_measured.xml' in str(p) else p, *args, **kw))
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HDR)
    env = RiderEnv2(domain=domain, commands=False, pushes=False, episode_seconds=25, native_policy=native, noise=1.0)
    env.reset(seed=seed, options={'pitch': np.radians(-3.0), 'roll': 0., 'yaw': np.radians(alpha_deg)})
    h = h_mm / 1000.
    x_contact = STEP_X - np.sqrt(max(2 * RADIUS * h - h * h, 0.))     # axle x at which the tyre first touches the edge
    lead = 0.010
    wheel_ids = list(env.wheel_bodies)
    hop_start = [None, None]
    orig_apply = env._apply_action

    def wheel_x():
        return np.array([float(env.data.xpos[b][0]) for b in wheel_ids])

    def offsets():
        off = np.zeros(2)
        if mode == 'high':                     # body raised for the whole run (legs out by amp)
            return off + amp
        if mode == 'plain' or env.elapsed < 2.0 or h_mm == 0:
            return off
        x = wheel_x()
        if mode == 'hop':
            for i in range(2):
                if hop_start[i] is None and x[i] >= x_contact - lead:
                    hop_start[i] = env.elapsed
                if hop_start[i] is not None and env.elapsed - hop_start[i] < 0.25:
                    off[i] = -amp
        elif mode == 'lean':
            order = np.argsort(-x)                       # leader first
            for i in order:
                if x_contact - lead <= x[i] <= STEP_X + 0.025:
                    off[i] = amp                         # its leg out (its side up), the other leg in: the body rolls away from it
                    off[1 - i] = -amp
                    break
        return off

    nominal = []

    def patched():
        saved = env.out[16:18].copy()
        if mode in ('hop', 'lean') and env.elapsed >= 2.0:
            # the firmware's roll-levelling loop (on by default) would pull a rolled body straight back, so from here on the leg targets are the
            # standing height at t = 2 s plus the script: the same as the firmware with levelling switched off
            if not nominal:
                nominal.append(saved.copy())
            env.out[16:18] = nominal[0] + offsets()
        else:
            env.out[16:18] = saved + offsets()
        orig_apply()
        env.out[16:18] = saved
    env._apply_action = patched

    run_in = h / np.tan(np.radians(ramp_deg)) if ramp_deg and h_mm else 0.
    x_done = STEP_X + run_in + 0.10                       # the run ends once the robot is this far past the obstacle
    fell, roll_max, cmdmax = False, 0., 0.
    pitch_lo, pitch_hi = 0., 0.
    ever_top, last_pitch = False, 0.
    while True:
        if env.elapsed > 2.0:
            env.command[0] = int(np.clip(round(127.5 + v / .21 * 127.5), 0, 255))
        _, _, term, trunc, info = env.step(native.action.copy())
        roll_max = max(roll_max, abs(float(np.degrees(info['roll']))))
        if env.elapsed > 2.0:
            pitch_lo, pitch_hi = min(pitch_lo, float(np.degrees(info['pitch']))), max(pitch_hi, float(np.degrees(info['pitch'])))
        cmdmax = max(cmdmax, float(np.max(np.abs(info['wheel_command']))))
        last_pitch = float(np.degrees(info['pitch']))
        if h_mm and not ever_top:
            zt = [float(env.data.xpos[b][2]) for b in wheel_ids]
            ever_top = bool(all(zz >= RADIUS + h - 0.004 for zz in zt) and all(xx > STEP_X for xx in wheel_x()))
        if term or trunc:
            fell = bool(term)
            break
        if min(float(env.data.xpos[b][0]) for b in wheel_ids) > x_done:
            break
    z = np.array([float(env.data.xpos[b][2]) for b in wheel_ids])
    x = wheel_x()
    env.close(); native.close()
    passed = bool((not fell) and np.all(z >= RADIUS + h - 0.004) and np.all(x > x_done)) if h_mm else bool(not fell and np.all(x > STEP_X))
    return dict(h=h_mm, v=v, seed=seed, alpha=alpha_deg, mode=mode, amp=amp, ramp=ramp_deg, domain=domain, passed=passed, fell=fell,
                x=[round(float(t), 3) for t in x], roll_max=roll_max, max_cmd=cmdmax, pitch_lo=pitch_lo, pitch_hi=pitch_hi,
                ever_top=ever_top, fell_back=bool(fell and last_pitch < 0))


def table(rows, heights, columns, select, title):
    print(title)
    print(f"{'step mm':>8} " + ' '.join(name.rjust(13) for name, _ in columns))
    for h in heights:
        cells = []
        for _, cond in columns:
            rr = [r for r in rows if r['h'] == h and select(r) and cond(r)]
            cells.append(f"{sum(r['passed'] for r in rr)}/{len(rr)} ({sum(r['fell'] for r in rr)})".rjust(13))
        print(f"{h:8d} " + ' '.join(cells))
    print()


if __name__ == '__main__':
    quick = '--quick' in sys.argv
    heights = [8, 10, 12, 15, 20]
    speeds = [0.10, 0.16]
    seeds = range(2) if quick else range(4)
    jobs = []
    for h in heights:                                        # A: what the legs can do, straight and diagonal approach
        for v in speeds:
            for s in seeds:
                for alpha in (0, 30, 50):
                    jobs.append((h, v, s, alpha, 'plain', 0.))
                    jobs.append((h, v, s, alpha, 'hop', 0.018))
                    if alpha:
                        for amp in (0.006, 0.012):
                            jobs.append((h, v, s, alpha, 'lean', amp))
        for v in speeds:                                     # B: body raised (legs out) the whole run
            for s in seeds:
                for amp in (0.011, 0.018):
                    jobs.append((h, v, s, 0, 'high', amp))
    ramp_heights = [20, 40, 60, 100]
    slopes = [5, 10, 15, 20, 25, 30]
    for h in ramp_heights:                                   # C: a wedge in front of the step
        for sl in slopes:
            for v in speeds:
                for s in seeds:
                    jobs.append((h, v, s, 0, 'plain', 0., sl))
    began = time.monotonic()
    with Pool(9) as p:
        rows = p.map(run, jobs, chunksize=1)
    print('v6 policy (wheels only, never trained on steps) + scripted legs, nominal plant, wheel radius 30.8 mm')
    print('cells = passed/runs (fell); runs = 2 speeds x %d seeds\n' % len(seeds))
    for alpha in (0, 30, 50):
        cols = [('plain', lambda r: r['mode'] == 'plain'), ('hop 18', lambda r: r['mode'] == 'hop')]
        if alpha:
            cols += [('lean 6', lambda r: r['mode'] == 'lean' and r['amp'] == 0.006), ('lean 12', lambda r: r['mode'] == 'lean' and r['amp'] == 0.012)]
        table(rows, heights, cols, lambda r, a=alpha: r['alpha'] == a and not r['ramp'], f'A. approach angle {alpha} deg')
    table(rows, heights, [('legs normal', lambda r: r['mode'] == 'plain'), ('legs out 11', lambda r: r['mode'] == 'high' and r['amp'] == 0.011),
                          ('legs out 18', lambda r: r['mode'] == 'high' and r['amp'] == 0.018)],
          lambda r: r['alpha'] == 0 and not r['ramp'], 'B. body raised by the legs (straight approach)')
    table(rows, ramp_heights, [(f'{sl} deg', lambda r, sl=sl: r['ramp'] == sl) for sl in slopes], lambda r: True,
          'C. wedge of the given slope in front of the step (rows: step height)')
    if '--random' in sys.argv:
        cases = [(20, 10), (20, 15), (20, 20), (40, 15), (40, 20), (100, 15), (100, 20), (100, 30)]
        rnd = [(h, v, s, 0, 'plain', 0., sl, 1.0) for h, sl in cases for v in speeds for s in range(16)]
        rnd += [(h, v, s, 0, 'plain', 0., 0., 1.0) for h in (5, 8, 10, 12) for v in speeds for s in range(16)]
        with Pool(9) as p:
            rrows = p.map(run, rnd, chunksize=1)
        print('D. randomised plant (domain 1.0: masses, friction .55-1.15, delay, noise, +-2 deg ground tilt), 32 runs per row')
        for h, sl in cases:
            rr = [r for r in rrows if r['h'] == h and r['ramp'] == sl]
            print(f'  wedge {sl:2d} deg, step {h:3d} mm: passed {sum(r["passed"] for r in rr):2d}/{len(rr)}, fell {sum(r["fell"] for r in rr):2d}, '
                  f'max wheel command {max(r["max_cmd"] for r in rr):4.0f}/1023, body pitch up to {max(r["pitch_hi"] for r in rr):4.1f} deg')
        for h in (5, 8, 10, 12):
            rr = [r for r in rrows if r['h'] == h and not r['ramp']]
            print(f'  no wedge, step {h:3d} mm: passed {sum(r["passed"] for r in rr):2d}/{len(rr)}, fell {sum(r["fell"] for r in rr):2d}')
        print()
        rows = rows + rrows
    import json
    (ROOT / 'runs').mkdir(exist_ok=True)
    (ROOT / 'runs/step_probe_legs.json').write_text(json.dumps(rows))
    print('wall', round(time.monotonic() - began), 's')
