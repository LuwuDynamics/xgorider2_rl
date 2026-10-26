"""Bench check of the wheel-speed scale -- the one number the simulator could not pin down.

Why: the training simulator reproduces the robot's 5-14 Hz response exactly, but the wheel-speed register
only enters that response through the ratio wheel_speed / pitch_rate, which came out ~0.83 where free-rolling
wheels on this chassis would give ~2.6.  The simplest explanation is that the register reads about 1/pi of the
true rim speed; the simulator is trained on that assumption (``speed_scale = 0.28``).  A different cause would
change the speed scale seen at low frequency, so it is worth two minutes to measure.

How:  robot on the floor with its wheels on the ground, USB attached.  The script locks the motors itself
(``!stop``, repeated until the firmware confirms it; rider_cli no longer resets the board when it attaches, but an older
console or another serial tool might, so it also waits for a boot banner if one appears).  Then roll the robot forward by hand about 1 m (steady, no pushing back and forth).  It polls
``!status`` and compares the distance from the raw wheel encoders (1024 ticks/rev, exact) with the time integral of
the wheel speed.  ratio r = encoder distance / integral(speed).  The firmware converts both with the same wheel
diameter and pi, so r does not depend on the wheel size, only on the unit of the speed register.
(``physical_distance_m`` is not used: the firmware only updates it while the controller runs, so it stays 0 here.)

* r in about 2.3..4.2 (1/pi => 3.14) -> the register reads ~1/3 of the true speed, as the simulator assumes.
              Leave ``!speedcal`` at its default of 1.
* r ~ 1.0  -> the register is accurate; the policy would see ~3.5x too much speed feedback.  Run ``!speedcal 0.28``
              (no retraining needed) and tell me, so the simulator can be re-calibrated.
* anything else -> use ``!speedcal  0.28 * r``.

Measured 2026-10-22 (logs/wheel-scale-20261022): r = 3.63 (wheel 11: 3.65, wheel 21: 3.62), i.e. the register reads
0.275 of the encoder speed, against 0.28 assumed in training.

Usage:  python tools/calibrate_wheel_speed.py --port /dev/cu.usbserial-XXXX [--seconds 45] [--cue] [--save roll.jsonl]
``--cue`` plays a macOS sound when recording starts and ends; ``--save`` keeps the raw status lines so the run can be
analysed again later with ``--from-log``.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Wheel-speed reading / true speed assumed by the training simulator (simulation/rider_sim/plant.py SPEED_SCALE).
# The trained policy tolerates roughly +-40% around it (0.18..0.40); outside that it diverges slowly.
TRAINED_SCALE = 0.28
# Wheel diameter and pi exactly as the firmware's speed (actuator_bus.cpp) and odometry (rider_motion.cpp) use them.
MM_PER_TICK = 55 * np.pi / 1024


def connect(port):
    """Open the console and make sure the motors are locked before anybody touches the robot.

    rider_cli attaches without resetting the board; a tool that did reset it would bring it back up with the policy
    selected, so keep repeating ``!stop`` (after a short wait) until the firmware reports the manual stop."""
    from rider_cli import Session
    session = Session(port)
    try:
        time.sleep(2.5)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            try:
                session.send('!stop')
                time.sleep(.1)
                state = session.status(timeout=1.5)
            except RuntimeError:
                continue
            if state.get('manual_stop') is True and not state.get('driving'):
                return session, state
    except BaseException:
        session.close()
        raise
    session.close()
    raise SystemExit('could not confirm that the robot is stopped; check the power switch and the USB cable')


def cue(name):
    sound = Path('/System/Library/Sounds') / (name + '.aiff')
    if sound.exists():
        subprocess.Popen(['afplay', str(sound)])


def sample_of(status):
    wheels = {f['id']: f for f in status['feedback']}
    return (status['ms'] / 1000., wheels[11]['position'], wheels[21]['position'],
            .5 * (wheels[11]['speed'] + wheels[21]['speed']))


def unwrap(positions):
    step = np.diff(np.asarray(positions, float))
    step = (step + 512) % 1024 - 512          # shortest signed step on the 1024-tick ring
    return np.concatenate([[0.], np.cumsum(step)]), float(np.max(np.abs(step))) if len(step) else 0.


def analyse(samples):
    t = np.array([s[0] for s in samples], float)
    ticks11, worst11 = unwrap([s[1] for s in samples])
    ticks21, worst21 = unwrap([s[2] for s in samples])
    d = .5 * (ticks11 - ticks21) * MM_PER_TICK / 1000.     # wheel 21 is mounted mirrored: forward = falling position
    v = np.array([s[3] for s in samples], float) / 1000.   # m/s (filtered speed, already sign-corrected per wheel)
    cumulative = np.concatenate([[0.], np.cumsum(.5 * (v[1:] + v[:-1]) * np.diff(t))])
    integral = float(cumulative[-1])
    travelled = float(d[-1] - d[0])
    path = float(np.sum(np.abs(np.diff(d))))
    energy = float(np.dot(cumulative, cumulative))
    # slope of the encoder distance against the running integral of the speed (least squares through the origin)
    fit = float(np.dot(cumulative, d - d[0]) / energy) if energy > 0 else None
    return dict(samples=len(samples), seconds=float(t[-1] - t[0]), odometry_net_m=travelled, odometry_path_m=path,
                speed_integral_m=integral, max_speed_m_s=float(np.max(np.abs(v))), fit_ratio=fit,
                largest_step_ticks=max(worst11, worst21))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--cue', action='store_true')
    parser.add_argument('--save', type=Path)
    parser.add_argument('--from-log', type=Path, help='analyse an existing JSON-lines status log instead of the robot')
    args = parser.parse_args()
    samples = []
    if args.from_log:
        for line in args.from_log.read_text().splitlines():
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if 'feedback' in r and 'ms' in r:
                samples.append(sample_of(r))
    else:
        session, state = connect(args.port)
        raw = args.save.open('w') if args.save else None
        try:
            print(f"robot stopped (battery {state.get('battery_voltage')} V); recording {args.seconds:.0f} s: "
                  'roll it forward by hand now (about 1 m, steady)...', flush=True)
            if args.cue:
                cue('Ping')
            began = time.monotonic()
            failures = 0
            while time.monotonic() - began < args.seconds:
                try:
                    s = session.status()
                except RuntimeError:
                    failures += 1
                    if failures > 5:
                        raise SystemExit('lost contact with the robot')
                    continue
                if raw:
                    raw.write(json.dumps(s) + '\n')
                    raw.flush()
                samples.append(sample_of(s))
            if args.cue:
                cue('Glass')
        finally:
            if raw:
                raw.close()
            session.close()
    if len(samples) < 10:
        raise SystemExit('too few samples')
    out = analyse(samples)
    print(json.dumps(out, indent=2))
    if out['largest_step_ticks'] > 400:
        print('warning: a wheel moved more than 400 ticks between two samples; the encoder count may have aliased')
    if abs(out['speed_integral_m']) < 0.05 or out['odometry_path_m'] < 0.15:
        print('the robot barely moved (need >= ~0.3 m); repeat with a longer, steadier roll')
        return
    r = out['odometry_net_m'] / out['speed_integral_m']
    if r < 0:
        print('note: encoder distance and wheel speed have opposite signs; using the magnitude')
        r = abs(r)
    if out['fit_ratio'] and abs(out['fit_ratio'] / r - 1) > .15:
        print(f"warning: the end-to-end ratio {r:.2f} and the fitted ratio {out['fit_ratio']:.2f} disagree; repeat with a steadier roll")
    cal = TRAINED_SCALE * r
    print(f"ratio encoder distance/integral(speed) = {r:.2f}   (1/pi hypothesis predicts {np.pi:.2f}, accurate register predicts 1.00)")
    print(f'the register reads {1 / r:.3f} of the encoder speed (training assumed {TRAINED_SCALE})')
    if 2.3 <= r <= 4.2:
        print('-> within the trained tolerance of the simulator assumption. Keep the default (speedcal 1.0).')
    else:
        print(f'-> set the policy scale with:  !speedcal {cal:.2f}   (BLE register 0xb8 = round(64*{cal:.2f}) = {round(64 * cal)})')


if __name__ == '__main__':
    main()
