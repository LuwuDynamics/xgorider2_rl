"""Stability-margin report for an exported policy (actor.json).

Two independent views of the same question -- "will it ring on the robot?":

1. **Model** : local gains of the network (Jacobian at the operating point) closed around the
   linearised MuJoCo plant with the identified actuator/sensor chain; nominal + stress plants;
   gain margin, delay margin, damping.
2. **Data**  : the same local gains applied to the *measured* real frequency response
   (``models/measured_frf.json``, from the robot's own ringing).  No simulator is involved.
   Reports the loop gain Phi(f) = C(f)P(f) and the sensitivity 1/|1 - Phi| across 4.8-13.5 Hz.
   The deployed v5/v8 policies score |1 - Phi| ~ 0.3 here (sensitivity peak ~3, the ringing).

Usage:  python tools/loop_margin.py artifacts/<name>/actor.json [--old artifacts/balance-legs-v5/actor.json]
"""
import argparse
import json
from pathlib import Path
import re
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))

RADIUS = .0308
POLICY_H = ROOT.parent / 'firmware/src/policy/policy.h'


def firmware_speed_tau():
    """Low-pass time constant the firmware applies to the wheel speeds (policy.h), so the report matches the build."""
    match = re.search(r'kSpeedFilterTauS\s*=\s*([0-9.]+)f', POLICY_H.read_text())
    return float(match.group(1)) if match else 0.


def load_actor(path):
    data = json.loads(Path(path).read_text())
    layers = [(np.array(l['weight'], float), np.array(l['bias'], float)) for l in data['layers']]
    return layers, np.array(data['observation_mean'], float), np.array(data['observation_std'], float), data.get('observation_clip', 5.0)


def forward(actor, raw):
    layers, mean, std, clip = actor
    x = np.clip((raw - mean) / std, -clip, clip)
    h = np.tanh(layers[0][0] @ x + layers[0][1])
    h = np.tanh(layers[1][0] @ h + layers[1][1])
    return np.clip(layers[2][0] @ h + layers[2][1], -1, 1)


def local_gains(actor, pitch_deg=-3.5):
    layers, mean, std, clip = actor
    raw = np.zeros(18)
    raw[0] = np.deg2rad(pitch_deg)
    raw[11] = .5
    raw[15], raw[16] = .006, .006
    J = np.zeros((layers[2][0].shape[0], 18))
    for i in range(18):
        h = 1e-4 * max(1., abs(std[i]))
        d = np.zeros(18)
        d[i] = h
        J[:, i] = (forward(actor, raw + d) - forward(actor, raw - d)) / (2 * h)
    common = (J[0] - J[1]) / 2 * 1023
    diff = (J[0] + J[1]) / 2 * 1023
    return dict(pitch=common[0], rate=common[2], speed=common[5] + common[6], position=common[12], integral=common[17],
                prev=(common[13] - common[14]) / 1023., fwd_ref=common[9], yaw_rate=diff[4], turn_ref=diff[10],
                action_at_point=forward(actor, raw).tolist())


def data_loop(gains, period, frf, speed_tau=0.):
    """Phi(f) over the measured band."""
    rows = []
    for f, rm, rp, wm, wp in zip(frf['freqs_hz'], frf['rate_mag'], frf['rate_phase_deg'], frf['wheel_mag'], frf['wheel_phase_deg']):
        w = 2 * np.pi * f
        jw = 1j * w
        Hr = rm * np.exp(1j * np.radians(rp))
        Hv = RADIUS * wm * np.exp(1j * np.radians(wp)) / (1 + jw * speed_tau)
        theta = -Hr / jw
        x = Hv / jw
        z = x / jw
        raw = gains['pitch'] * theta + gains['rate'] * Hr + gains['speed'] * Hv + gains['position'] * x + gains['integral'] * z
        raw = raw / (1 - gains['prev'] * np.exp(-jw * period))
        hold = np.exp(-jw * period / 2) * np.sinc(w * period / 2 / np.pi)
        phi = raw * hold
        rows.append(dict(hz=f, mag=float(abs(phi)), phase_deg=float(np.degrees(np.angle(phi))), one_minus=float(abs(1 - phi))))
    return rows


def model_margins(gains, period, speed_tau=0.):
    from design_linear import Designer
    from rider_sim import looplin
    designer = Designer(period=period, speed_tau=speed_tau)
    g = [gains['pitch'], gains['rate'], gains['speed'], gains['position']]
    ki = gains['integral']
    report = {}
    for name, loop in [('nominal', designer.nominal_loop)] + list(designer.stress.items()):
        s = looplin.spectral_summary(loop, g, gains['prev'], period, ki)
        report[name] = dict(stable=bool(s['stable']), growth=float(s['growth']), min_zeta=float(s['min_zeta']),
                            unstable=[(float(a), float(b)) for a, b in s['unstable_poles']])
    m = looplin.margins(designer.model, designer.probe, designer.wheel_dofs, designer.nominal, g, gains['prev'], period, ki, speed_filter_tau=speed_tau)
    report['margins'] = dict(gain_up=m['gain_up'], gain_down=m['gain_down'], delay_margin_ms=None if m['delay_margin'] is None else 1000 * m['delay_margin'])
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('actor', type=Path)
    parser.add_argument('--old', type=Path, help='an older actor.json to print for comparison')
    parser.add_argument('--period', type=float, default=.006)
    parser.add_argument('--old-period', type=float, default=.012)
    parser.add_argument('--pitch-deg', type=float, default=-3.5)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--speed-tau', type=float, default=None, help='wheel-speed low-pass of the policy under test (default: policy.h)')
    parser.add_argument('--old-speed-tau', type=float, default=0.)
    args = parser.parse_args()
    tau = firmware_speed_tau() if args.speed_tau is None else args.speed_tau
    frf = json.loads((ROOT / 'models/measured_frf.json').read_text())
    result = {}
    for label, path, period, speed_tau in [('policy', args.actor, args.period, tau)] + ([('reference', args.old, args.old_period, args.old_speed_tau)] if args.old else []):
        actor = load_actor(path)
        gains = local_gains(actor, args.pitch_deg)
        rows = data_loop(gains, period, frf, speed_tau)
        entry = dict(path=str(path), period_ms=period * 1000, speed_filter_tau=speed_tau, gains={k: v for k, v in gains.items() if k != 'action_at_point'}, data_loop=rows,
                     data_sensitivity_peak=float(1 / min(r['one_minus'] for r in rows)),
                     data_max_loop_gain_above_8hz=float(max(r['mag'] for r in rows if r['hz'] >= 8)))
        if label == 'policy':
            entry['model'] = model_margins(gains, period, speed_tau)
        result[label] = entry
        print(f"== {label}: {path}")
        print('   local gains: ' + ', '.join(f"{k} {v:.1f}" for k, v in entry['gains'].items() if k != 'prev') + f", prev {gains['prev']:.3f}")
        print('   measured-plant loop  f[Hz]  |Phi|  phase  |1-Phi|')
        for r in rows:
            print(f"                         {r['hz']:5.1f}  {r['mag']:5.2f}  {r['phase_deg']:6.0f}  {r['one_minus']:5.2f}")
        print(f"   sensitivity peak on measured plant: {entry['data_sensitivity_peak']:.2f}   max |Phi| above 8 Hz: {entry['data_max_loop_gain_above_8hz']:.2f}")
        if 'model' in entry:
            m = entry['model']
            print('   model: nominal stable', m['nominal']['stable'], 'min zeta', round(m['nominal']['min_zeta'], 2),
                  '| gain margin up x%.2f down x%.2f | delay margin %.0f ms' % (m['margins']['gain_up'], m['margins']['gain_down'], m['margins']['delay_margin_ms']))
            for name, v in m.items():
                if name not in ('nominal', 'margins'):
                    print(f"      {name:18s} stable={v['stable']!s:5s} min_zeta {v['min_zeta']:.2f} growth {v['growth']:.2f}/s")
    if args.output:
        args.output.write_text(json.dumps(result, indent=2, default=float) + '\n')


if __name__ == '__main__':
    main()
