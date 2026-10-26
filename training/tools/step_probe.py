"""(Legs and ramps: see step_probe_legs.py.)  Can the existing v6 policy (never trained on steps) roll up a small step?  Simulator probe: nominal plant, a box step 25 cm ahead,
scripted forward command (what the phone's joystick sends), 4 seeds per cell.

Usage: .venv/bin/python tools/step_probe.py

Result 2026-10-22 (wheel radius 30.8 mm): steps up to 8 mm are climbed at 0.12-0.21 m/s (4/4), 10 mm and higher are not (the wheel stalls
at the edge; at >= 0.16 m/s the robot falls), the wheel command reaches 650-980 of 1023.  Quasi-static drive torque for a step of height h:
T = W_wheel * sqrt(2 r h - h^2) = 0.06 N*m (10 mm) .. 0.08 N*m (28 mm) per wheel against ~0.19 N*m at full command in the model.
"""
import sys, os, re, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from multiprocessing import Pool
SIM = ROOT
HDR = str(SIM / 'artifacts/balance-v6/policy_weights.h')
STEP_X = 0.25

def make_xml(h_mm):
    text = (SIM / 'models/rider_measured.xml').read_text()
    h = h_mm / 1000.
    step = f'<geom name="step" type="box" pos="{STEP_X + 1.0} 0 {h / 2}" size="1.0 1.5 {h / 2}" contype="2" conaffinity="1" material="floor" />\n'
    return text.replace('<geom name="ground"', step + '    <geom name="ground"', 1) if h_mm > 0 else text

def run(a):
    h_mm, v, seed, command_scale = a
    os.chdir(SIM); sys.path.insert(0, str(SIM))
    import mujoco
    xml_path = Path(f'/tmp/rider_step_{h_mm}.xml')
    xml_path.write_text(make_xml(h_mm))
    orig = mujoco.MjModel.from_xml_path
    mujoco.MjModel.from_xml_path = staticmethod(lambda p, *args, **kw: orig(str(xml_path) if 'rider_measured.xml' in str(p) else p, *args, **kw))
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(HDR)
    env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=12, native_policy=native, noise=1.0)
    env.reset(seed=seed, options={'pitch': np.radians(-3.0), 'roll': 0., 'yaw': 0.})
    t, x, pitch, cmdmax = [], [], [], 0.
    started = False
    while True:
        # let it settle for 2 s, then drive forward at v m/s (the phone's forward byte: 127.5 + v / .21 * 127.5)
        if env.elapsed > 2.0:
            env.command[0] = int(np.clip(round(127.5 + v / .21 * 127.5), 0, 255))
        _, _, term, trunc, info = env.step(native.action.copy())
        base_x = float(env.data.qpos[0])
        t.append(env.elapsed); x.append(base_x); pitch.append(np.degrees(info['pitch']))
        cmdmax = max(cmdmax, float(np.max(np.abs(info['wheel_command']))))
        if term or trunc: break
    env.close(); native.close()
    x = np.array(x)
    passed = bool((x[-1] > STEP_X + 0.07) and not term)
    return dict(h=h_mm, v=v, seed=seed, passed=passed, fell=bool(term), x_end=float(x[-1]), max_cmd=cmdmax, pitch_min=float(np.min(pitch)), pitch_max=float(np.max(pitch)))

if __name__ == '__main__':
    heights = [0, 3, 5, 8, 10, 12, 15, 20]
    speeds = [0.08, 0.12, 0.16, 0.21]
    jobs = [(h, v, s, 1.0) for h in heights for v in speeds for s in range(4)]
    began = time.monotonic()
    with Pool(9) as p:
        rows = p.map(run, jobs, chunksize=1)
    print('existing v6 policy, nominal plant, forward command v (m/s), wheel radius 30.8 mm, 4 seeds each: passed / fell')
    print(f"{'step mm':>8} " + ' '.join(f'v={v:.2f}'.rjust(14) for v in speeds))
    for h in heights:
        cells = []
        for v in speeds:
            rr = [r for r in rows if r['h'] == h and r['v'] == v]
            cells.append(f"{sum(r['passed'] for r in rr)}/{len(rr)} pass {sum(r['fell'] for r in rr)} fell")
        print(f"{h:8d} " + ' '.join(c.rjust(14) for c in cells))
    ok = [r for r in rows if r['h'] in (10, 15) and r['v'] == 0.21]
    print('example 10-15 mm at 0.21 m/s: x_end', [round(r['x_end'], 2) for r in ok], 'max wheel command', [round(r['max_cmd']) for r in ok])
    print('wall', round(time.monotonic() - began), 's')
