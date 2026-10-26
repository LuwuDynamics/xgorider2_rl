"""Robust linear core for the balance policy (common-mode channel).

Finds static gains ``u = kp*pitch + kd*pitch_rate_fw + kv*wheel_speed + kx*position_error``
that minimise a noise-driven LQ cost on the *nominal* identified plant while staying
stable on a set of stress plants (2x torque constant, +10 ms latency, 2x lag, wheel
sensor 0.25x..1x, 0.6x torque constant). Those stress cases are the margin
requirement the previous controller failed on the real robot (see REBUILD.md).

The result seeds the neural policy (``train2.py``) and is also a reference
controller for judging what the learned policy adds.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import numpy as np
import scipy.linalg as sla
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rider_sim.env2 import RiderEnv2  # noqa: E402
from rider_sim.plant import Plant, NOMINAL  # noqa: E402
from rider_sim import looplin  # noqa: E402

NOISE = np.array([.002, .03, .004, .002])   # sensor noise std: pitch rad, rate rad/s, speed m/s, position m
DISTURBANCE = 40.                           # command-equivalent disturbance std (units)
SCALES = np.array([.01, .1, .05, 150.])     # pitch rad, speed m/s, position m, command units


def nominal_plant(env):
    """Nominal plant from the environment's own sampler at domain 0 (FRF-pinned)."""
    env.domain = 0.
    env.reset(seed=1, options={'pitch': 0., 'roll': 0., 'yaw': 0.})
    return copy.copy(env.plant)


def stress_set(nominal):
    def variant(**kw):
        p = copy.copy(nominal)
        for k, v in kw.items():
            setattr(p, k, v)
        return p
    slow = dict(speed_delay=.010)          # wheel-speed channel latency (bus slot staleness) is the least certain
    return {
        # low-side (LF robustness): the loop must still be stable
        'gain x0.8': (variant(kt=nominal.kt * 0.8), 0., {}),
        'sensor 0.20': (variant(speed_scale=0.20), 0., {}),
        'corner low': (variant(kt=nominal.kt * 0.75, delay=0.001, lag=nominal.lag * 0.6, speed_scale=0.20), 0., dict(speed_delay=.002)),
        # high-side (HF robustness): the loop must stay well damped, this is what rings on hardware
        'gain x2.0': (variant(kt=nominal.kt * 2.0), 0., {}),
        'delay +12ms': (variant(), .012, {}),
        'lag x2': (variant(lag=nominal.lag * 2), 0., {}),
        'sensor 0.36': (variant(speed_scale=0.36), 0., {}),
        'speed delay 10ms': (variant(), 0., slow),
        # joint corners of the training family: everything unfavourable at once
        'corner high': (variant(kt=nominal.kt * 1.35, delay=0.006, lag=nominal.lag * 1.6, speed_scale=0.36), 0., dict(speed_delay=.008)),
        'corner high2': (variant(kt=nominal.kt * 1.5, delay=0.004, lag=nominal.lag * 1.3, speed_scale=0.33), 0., dict(speed_delay=.006)),
    }


HIGH_SIDE = ('gain x2.0', 'delay +12ms', 'lag x2', 'sensor 0.36', 'speed delay 10ms', 'corner high', 'corner high2')


def step_disturbance_cost(loop, gains, period, ki, seconds=12., bias=100.):
    """Integrated squared position error after a constant command-equivalent disturbance (CoM offset)."""
    n = loop.A.shape[0]
    M = np.zeros((n + 1, n + 1))
    M[:n, :n] = loop.A
    M[:n, n:] = loop.B
    E = sla.expm(M * period)
    Bd = E[:n, n:]
    Acl = loop.closed_loop(np.asarray(gains, float), 0., period, ki)
    x = np.zeros(n + 2)
    drive = np.zeros(n + 2)
    drive[:n] = Bd[:, 0] * bias
    ise = 0.
    cx = np.zeros(n + 2)
    cx[:n] = loop.C[3]
    for _ in range(int(seconds / period)):
        x = Acl @ x + drive
        ise += (cx @ x) ** 2 * period
        if not np.isfinite(ise) or ise > 1e3:
            return 1e3
    return ise


def noise_cost(loop, gains, period, ki=0.):
    """Noise-driven variances (pitch, speed, position, command) of the closed loop."""
    n = loop.A.shape[0]
    M = np.zeros((n + 1, n + 1))
    M[:n, :n] = loop.A
    M[:n, n:] = loop.B
    E = sla.expm(M * period)
    Bd = E[:n, n:]
    g = np.asarray(gains, float)
    Acl = loop.closed_loop(g, 0., period, ki)
    if np.max(np.abs(np.linalg.eigvals(Acl))) >= 1.0 - 1e-6:
        return None
    Bw = np.zeros((n + 2, 5))
    Bw[:n, 0] = Bd[:, 0] * DISTURBANCE
    for j in range(4):
        Bw[:n, 1 + j] = Bd[:, 0] * g[j] * NOISE[j]
        Bw[n, 1 + j] = g[j] * NOISE[j]
    Bw[n + 1, 4] = period * NOISE[3]
    P = sla.solve_discrete_lyapunov(Acl, Bw @ Bw.T)
    C = loop.C
    Pp = P[:n, :n]
    var_theta = float(C[0] @ Pp @ C[0])
    var_v = float(C[2] @ Pp @ C[2])
    var_x = float(C[3] @ Pp @ C[3])
    ku = np.concatenate([(g.reshape(1, 4) @ C)[0], [0.], [ki]])
    var_u = float(ku @ P @ ku + np.sum((g * NOISE) ** 2))
    return np.array([var_theta, var_v, var_x, var_u])


def data_margin(gains, ki, period, frf, rate_scale=1., wheel_scale=1., extra_delay=0., wheel_phase_deg=0., speed_tau=0.):
    """min over the measured band of |1 - Phi| using the measured (not simulated) plant response."""
    radius = .0308
    worst = 9.
    for f, rm, rp, wm, wp in zip(frf['freqs_hz'], frf['rate_mag'], frf['rate_phase_deg'], frf['wheel_mag'], frf['wheel_phase_deg']):
        w = 2 * np.pi * f
        jw = 1j * w
        d = np.exp(-jw * extra_delay)
        Hr = rate_scale * rm * np.exp(1j * np.radians(rp)) * d
        Hw = wheel_scale * wm * np.exp(1j * np.radians(wp + wheel_phase_deg)) * d
        Hv = radius * Hw / (1 + jw * speed_tau)
        theta = -Hr / jw
        x = Hv / jw
        z = x / jw
        raw = gains[0] * theta + gains[1] * Hr + gains[2] * Hv + gains[3] * x + ki * z
        phi = raw * np.exp(-jw * period / 2) * np.sinc(w * period / 2 / np.pi)
        worst = min(worst, abs(1 - phi))
    return worst


DATA_VARIANTS = (dict(), dict(rate_scale=1.25, wheel_scale=1.25), dict(rate_scale=.8, wheel_scale=.8), dict(extra_delay=.004),
                 dict(wheel_scale=1.5), dict(wheel_scale=.65), dict(wheel_phase_deg=20.), dict(wheel_phase_deg=-20.))


class Designer:
    def __init__(self, period=.006, weights=(1., 1., .3, 1.), speed_tau=0.):
        self.env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=5, noise=0.)
        self.env.reset(seed=1, options={'pitch': 0., 'roll': 0., 'yaw': 0.})
        self.model = self.env.model
        self.probe = self.env.probe_pose()
        self.wheel_dofs = [int(i) for i in self.env.wheel_v]
        self.period = period
        self.speed_tau = speed_tau
        self.frf = json.loads((ROOT / 'models/measured_frf.json').read_text())
        self.data_base_min = .60
        self.data_variant_min = .40
        self.weights = np.asarray(weights)
        self.margin_decay = .08
        self.zeta_min = .30
        self.drift_weight = 1.0
        self.nominal = nominal_plant(self.env)
        self.probe = self.env.probe_pose()
        self.nominal_loop = looplin.LinearLoop(self.model, self.probe, self.wheel_dofs, self.nominal, speed_filter_tau=speed_tau)
        self.stress = {name: looplin.LinearLoop(self.model, self.probe, self.wheel_dofs, plant, extra_delay=extra, speed_filter_tau=speed_tau, **kw)
                       for name, (plant, extra, kw) in stress_set(self.nominal).items()}

    def robust_growth(self, g, ki=0.):
        """Worst-case violation score over nominal + stress plants (<= 0 means all requirements met)."""
        worst = -1e9
        for name, loop in [('nominal', self.nominal_loop)] + list(self.stress.items()):
            s = looplin.spectral_summary(loop, g, 0., self.period, ki)
            if name in HIGH_SIDE or name == 'nominal':
                # stable AND damping >= zeta_min on every oscillatory pole
                value = (s['growth'] + self.margin_decay) if s['growth'] > -self.margin_decay else s['growth'] + self.margin_decay
                value = max(value, self.zeta_min - s['min_zeta'])
            else:
                value = s['growth'] + self.margin_decay
            worst = max(worst, value)
        # the measured plant (no simulator): sensitivity in the band where the robot actually rang
        margins = [data_margin(list(g), ki, self.period, self.frf, speed_tau=self.speed_tau, **kw) for kw in DATA_VARIANTS]
        worst = max(worst, self.data_base_min - margins[0], self.data_variant_min - min(margins[1:]))
        return worst

    def cost(self, params):
        params = np.asarray(params, float)
        g, ki = params[:4], params[4]
        if g[0] <= 0 or g[1] >= 0 or g[2] < 0 or g[3] < 0 or ki < 0:
            return 1e9
        robust = self.robust_growth(g, ki)
        if robust > 0:
            return 1e6 * (1 + robust)
        costs = noise_cost(self.nominal_loop, g, self.period, ki)
        if costs is None:
            return 1e6
        drift = step_disturbance_cost(self.nominal_loop, g, self.period, ki)
        return float(np.sum(self.weights * costs / SCALES ** 2)) + 3e-3 * float(np.sum(np.abs(g)) / 1000.) + self.drift_weight * drift / (.05 ** 2 * 12.)

    def report(self, params):
        params = np.asarray(params, float)
        g, ki = params[:4], params[4]
        out = {}
        for name, loop in [('nominal', self.nominal_loop)] + list(self.stress.items()):
            s = looplin.spectral_summary(loop, g, 0., self.period, ki)
            out[name] = dict(stable=bool(s['stable']), growth=s['growth'], min_zeta=s['min_zeta'])
        out['margins'] = looplin.margins(self.model, self.probe, self.wheel_dofs, self.nominal, g, 0., self.period, ki, speed_filter_tau=self.speed_tau)
        out['data_margin'] = [round(data_margin(list(g), ki, self.period, self.frf, speed_tau=self.speed_tau, **kw), 3) for kw in DATA_VARIANTS]
        c = noise_cost(self.nominal_loop, g, self.period, ki)
        out['noise_std'] = None if c is None else dict(
            pitch_deg=float(np.degrees(np.sqrt(c[0]))), speed_mm_s=float(1000 * np.sqrt(c[1])),
            position_mm=float(1000 * np.sqrt(c[2])), command_units=float(np.sqrt(c[3])))
        return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--period', type=float, default=.006)
    parser.add_argument('--output', type=Path, default=ROOT / 'models/robust_linear_gains.json')
    parser.add_argument('--integral', action='store_true', help='allow a position-error integral (contract 6)')
    parser.add_argument('--zeta-min', type=float, default=.30)
    parser.add_argument('--drift-weight', type=float, default=1.0)
    parser.add_argument('--samples', type=int, default=6000)
    parser.add_argument('--speed-tau', type=float, default=0.)
    args = parser.parse_args()
    designer = Designer(period=args.period, speed_tau=args.speed_tau)
    designer.zeta_min = args.zeta_min
    designer.drift_weight = args.drift_weight
    rng = np.random.default_rng(0)
    candidates = []
    lu = lambda lo, hi: float(np.exp(rng.uniform(np.log(lo), np.log(hi))))
    for _ in range(args.samples):
        g = np.array([lu(150, 6000), -lu(20, 400), lu(200, 8000), lu(50, 4000)])
        ki = (lu(5, 800) if args.integral else 0.)
        r = designer.robust_growth(g, ki)
        candidates.append((r, np.concatenate([g, [ki]])))
    candidates.sort(key=lambda t: t[0])
    print('best random robust growth', candidates[0][0], np.round(candidates[0][1], 1), flush=True)
    best = None
    for i, (r, x0) in enumerate(candidates[:8]):
        fun = designer.cost if args.integral else (lambda p: designer.cost(np.array([*p[:4], 0.])))
        res = minimize(fun, x0, method='Nelder-Mead', options=dict(maxiter=1500, xatol=1., fatol=1e-5, adaptive=True))
        print(f'start {i}: robust0 {r:.2f} -> cost {res.fun:.4f} gains {np.round(res.x, 1)}', flush=True)
        if best is None or res.fun < best.fun:
            best = res
    params = np.array(best.x, float)
    if not args.integral:
        params[4] = 0.
    report = designer.report(params)
    result = dict(period=args.period, speed_filter_tau=args.speed_tau, gains=dict(pitch=params[0], pitch_rate_fw=params[1], wheel_speed=params[2],
                                                  position=params[3], position_integral=params[4]),
                  cost=best.fun, nominal_plant=designer.nominal.as_dict(), report=report)
    args.output.write_text(json.dumps(result, indent=2, default=float) + '\n')
    print(json.dumps(result, indent=2, default=float))


if __name__ == '__main__':
    main()
