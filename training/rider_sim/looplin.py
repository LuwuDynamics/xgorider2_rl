"""Linear loop analysis of a balance policy on a sampled plant.

The MuJoCo model is linearised about upright (``plant.linearize``); the wheel
actuator (dead time + lag + torque constant), the IMU filter and the bus
latencies are added as linear filters, the loop is closed through the policy's
local gains and discretised at the policy period (so sample-and-hold lag is
exact).  The result answers the question the old acceptance tests never asked:
*how much extra gain or latency can this controller tolerate before it rings?*

It is deliberately conservative about what it does not model: command slew
limiting, hidden-unit saturation and friction all *reduce* effective gain, so a
policy that is robust here is robust in the nonlinear simulation too.
"""
import numpy as np
import scipy.linalg as sla
from . import plant as plantlib

RADIUS = .0308
NV = 10


def _pade(T, n=2):
    """State-space (A,B,C,D) of a 2nd-order Pade delay."""
    T = max(float(T), 1e-4)
    A = np.array([[0., 1.], [-12 / T ** 2, -6 / T]])
    B = np.array([[0.], [1.]])
    C = np.array([[0., -12 / T]])
    return A, B, C, np.array([[1.]])


def _lag(tau):
    tau = max(float(tau), 1e-4)
    return np.array([[-1 / tau]]), np.array([[1 / tau]]), np.array([[1.]]), np.array([[0.]])


def _series(s1, s2):
    """s2 after s1."""
    A1, B1, C1, D1 = s1
    A2, B2, C2, D2 = s2
    n1, n2 = A1.shape[0], A2.shape[0]
    A = np.block([[A1, np.zeros((n1, n2))], [B2 @ C1, A2]])
    B = np.vstack([B1, B2 @ D1])
    C = np.hstack([D2 @ C1, C2])
    D = D2 @ D1
    return A, B, C, D


_CACHE = {}


def planar_plant(model, probe, wheel_dofs, armature, damping):
    """Cached planar linearisation (A, b) of the upright robot for a given wheel impedance."""
    key = (id(model), round(armature, 9), round(damping, 9))
    if key not in _CACHE:
        Ac, Bc = plantlib.linearize(model, probe, armature, damping, wheel_dofs)
        keep = [2, 4, 6, 7, 8, 9, NV + 0, NV + 2, NV + 4, NV + 6, NV + 7, NV + 8, NV + 9]
        A13 = Ac[np.ix_(keep, keep)]
        b13 = (Bc[:, 0] + Bc[:, 1])[keep].reshape(len(keep), 1)
        # Common-mode wheel coordinates: replace (left, right) wheel angle/speed by their
        # mean and drop the difference (a neutral, uncontrolled yaw-coupled mode).
        n = len(keep)
        T = np.eye(n)
        iL, iR = keep.index(wheel_dofs[0]), keep.index(wheel_dofs[1])
        jL, jR = keep.index(NV + wheel_dofs[0]), keep.index(NV + wheel_dofs[1])
        T[iL, iL] = T[iL, iR] = .5
        T[iR, iL], T[iR, iR] = .5, -.5
        T[jL, jL] = T[jL, jR] = .5
        T[jR, jL], T[jR, jR] = .5, -.5
        Ti = np.linalg.inv(T)
        A = T @ A13 @ Ti
        b = T @ b13
        keep2 = [k for k in range(n) if k not in (iR, jR)]
        _CACHE[key] = (A[np.ix_(keep2, keep2)], b[keep2], keep, (iL, jL), keep2)
    return _CACHE[key]


class LinearLoop:
    """Open-loop model from the common wheel command (units) to the policy's measurements."""

    def __init__(self, model, probe, wheel_dofs, plant, imu_delay=.0035, speed_delay=.004,
                 position_delay=.004, gyro_tau=.0017, command_delay=.0005, extra_delay=0., speed_filter_tau=0.):
        self.plant = plant
        # Planar subsystem only: lateral/yaw motion is decoupled at upright, and the
        # absolute position states are neutral integrators that would mask stability.
        Ac, bw, keep, (iW, jW), keep2 = planar_plant(model, probe, wheel_dofs, plant.armature, plant.damping)
        # rebuild the index map for the reduced (common-mode) state vector
        full_pos = {k: n for n, k in enumerate(keep2)}
        n = len(keep2)
        pos = {full: full_pos[keep.index(full)] for full in keep if keep.index(full) in full_pos}
        wheel_angle, wheel_speed = full_pos[iW], full_pos[jW]
        Cth = np.zeros((1, n)); Cth[0, pos[4]] = 1.
        Crate = np.zeros((1, n)); Crate[0, pos[NV + 4]] = -1.      # firmware sign
        Cv = np.zeros((1, n)); Cv[0, wheel_speed] = plant.speed_scale * RADIUS
        Cx = np.zeros((1, n)); Cx[0, wheel_angle] = RADIUS
        plant_ss = (Ac, bw, np.vstack([Cth, Crate, Cv, Cx]), np.zeros((4, 1)))
        # actuator: command -> delay -> lag -> torque constant
        act = _series(_pade(plant.delay + command_delay + extra_delay), _lag(plant.lag))
        act = (act[0], act[1], act[2] * plant.kt, act[3] * plant.kt)
        chain = _series(act, plant_ss)           # u -> [theta, rate, v, x]
        A, B, C, D = chain
        # per-channel measurement filters
        speed_chain = _pade(speed_delay) if not speed_filter_tau else _series(_pade(speed_delay), _lag(speed_filter_tau))
        filters = [_pade(imu_delay), _series(_lag(gyro_tau), _pade(imu_delay)), speed_chain, _pade(position_delay)]
        nA = A.shape[0]
        extra_states = [f[0].shape[0] for f in filters]
        total = nA + sum(extra_states)
        Afull = np.zeros((total, total))
        Afull[:nA, :nA] = A
        Bfull = np.zeros((total, 1)); Bfull[:nA] = B
        Cfull = np.zeros((4, total))
        row = nA
        for k, (f, ns) in enumerate(zip(filters, extra_states)):
            Af, Bf, Cf, Df = f
            Afull[row:row + ns, row:row + ns] = Af
            Afull[row:row + ns, :nA] = Bf @ C[k:k + 1]
            Cfull[k, row:row + ns] = Cf[0]
            Cfull[k, :nA] = Df[0, 0] * C[k]
            row += ns
        self.A, self.B, self.C = Afull, Bfull, Cfull

    def closed_loop(self, gains, prev_coeff, period, ki=0.):
        """Discrete closed loop for u_k = gains . y_k + prev_coeff * u_{k-1} + ki * z_k at ``period`` seconds.

        ``z`` is the firmware-side position-error integral (z_{k+1} = z_k + period * position_k).
        """
        n = self.A.shape[0]
        M = np.zeros((n + 1, n + 1))
        M[:n, :n] = self.A
        M[:n, n:] = self.B
        E = sla.expm(M * period)
        Ad, Bd = E[:n, :n], E[:n, n:]
        K = np.asarray(gains, float).reshape(1, 4) @ self.C
        Cx = self.C[3:4]
        # state: [plant (n); u_prev; z]
        Acl = np.zeros((n + 2, n + 2))
        Acl[:n, :n] = Ad + Bd @ K
        Acl[:n, n] = (Bd * prev_coeff)[:, 0]
        Acl[:n, n + 1] = (Bd * ki)[:, 0]
        Acl[n, :n] = K[0]
        Acl[n, n] = prev_coeff
        Acl[n, n + 1] = ki
        Acl[n + 1, :n] = period * Cx[0]
        Acl[n + 1, n + 1] = 1.
        return Acl

    def poles(self, gains, prev_coeff, period, ki=0.):
        lam = np.linalg.eigvals(self.closed_loop(gains, prev_coeff, period, ki))
        lam = np.where(np.abs(lam) < 1e-12, 1e-12, lam)
        s = np.log(lam.astype(complex)) / period
        return lam, s


def policy_gains(jac_common, jac_prev):
    """Collapse an actor Jacobian (units per raw observation) to loop gains."""
    j = jac_common
    gains = [j[0], j[2], j[5] + j[6], j[12]]
    prev = (jac_prev[0] - jac_prev[1]) / 1023.
    return gains, prev


def spectral_summary(loop, gains, prev, period, ki=0.):
    lam, s = loop.poles(gains, prev, period, ki)
    growth = float(np.max(s.real))
    worst = int(np.argmax(s.real))
    unstable = [(abs(x.imag) / (2 * np.pi), float(x.real)) for x in s if x.real > 1e-3]
    # damping of the least-damped oscillatory pole (imag part > 0.5 Hz)
    osc = [x for x in s if abs(x.imag) > 2 * np.pi * .5]
    zeta = float(min((-x.real / abs(x)) for x in osc)) if osc else 1.0
    return dict(stable=growth < 1e-3, growth=growth, worst_hz=abs(s[worst].imag) / (2 * np.pi),
                unstable_poles=sorted(unstable), min_zeta=zeta)


def margins(model, probe, wheel_dofs, plant, gains, prev, period, ki=0., **kw):
    """Gain margin (multiplicative kt range) and delay margin (extra seconds) for a controller."""
    import copy

    def stable(kt_scale=1., extra=0.):
        p = copy.copy(plant)
        p.kt = plant.kt * kt_scale
        loop = LinearLoop(model, probe, wheel_dofs, p, extra_delay=extra, **kw)
        return spectral_summary(loop, gains, prev, period, ki)

    nominal = stable()
    result = dict(nominal=nominal)
    if not nominal['stable']:
        result.update(gain_up=None, gain_down=None, delay_margin=None)
        return result
    lo, hi = 1., 1.
    for scale in np.geomspace(1.05, 40, 60):
        if not stable(kt_scale=scale)['stable']:
            hi = scale
            break
    else:
        hi = float('inf')
    for scale in np.geomspace(1 / 1.05, 1 / 40, 60):
        if not stable(kt_scale=scale)['stable']:
            lo = scale
            break
    else:
        lo = 0.
    margin = float('inf')
    for extra in np.arange(.001, .08, .001):
        if not stable(extra=extra)['stable']:
            margin = extra
            break
    result.update(gain_up=hi, gain_down=lo, delay_margin=margin)
    return result
