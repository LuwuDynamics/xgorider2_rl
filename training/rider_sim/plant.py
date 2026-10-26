"""Data-calibrated wheel plant for the Rider2 simulation.

Why this exists
---------------
The first simulation modelled the wheel drive as an assumed "speed servo"
and was never compared with the robot. Real traces (data/real_robot_traces/) show the opposite structure in the 5-14 Hz band that
decides whether a balance policy rings:

* wheel command -> pitch rate behaves like an integrator (-90 deg) plus about
  13 ms of extra lag (bus + motor current loop + IMU filter),
* the gain is about 0.5 rad/s^2 per command unit,
* the wheel speed seen over the bus is only ~0.86x the pitch rate, whereas a
  free-rolling wheel pair on this chassis would give ~2.6x.

``measured_frf.json`` stores those targets (extracted by
``tools/sysid_frf.py`` from the logs).  Every plant sampled here is *conditioned
on them*: masses/CoM/friction are randomised first, then the wheel torque
constant and wheel-speed sensor scale are solved so the sampled robot still
reproduces the measured frequency response.  The actuator lag/delay, wheel
impedance and Coulomb friction -- which the data cannot separate -- are
randomised around the fitted values.  A policy trained across this family
cannot rely on a plant that the real robot does not resemble.
"""
from dataclasses import dataclass, asdict
import json
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
TARGET_PATH = ROOT / 'models/measured_frf.json'

# Fallback targets (median of the 100+ limit-cycle chunks in the logs).
DEFAULT_TARGET = dict(
    ref_hz=8.0,
    rate_per_unit=0.0107,    # |pitch_rate_fw / wheel command|, rad/s per command unit
    wheel_to_rate=0.86,      # |wheel_speed / pitch_rate_fw| (both rad/s)
    freqs_hz=[4.8, 6.0, 7.0, 8.0, 9.0, 10.0, 11.5, 13.5],
    rate_mag=[0.01479, 0.01313, 0.01214, 0.01066, 0.00898, 0.00840, 0.00721, 0.00572],
    rate_phase_deg=[-91.6, -99.6, -105.5, -109.7, -118.3, -124.2, -131.1, -153.0],
    wheel_mag=[0.01229, 0.01139, 0.01063, 0.00934, 0.00747, 0.00648, 0.00579, 0.00443],
    wheel_phase_deg=[-85.7, -87.7, -93.5, -92.5, -103.5, -111.9, -118.1, -145.4],
)


def load_target():
    if TARGET_PATH.exists():
        return json.loads(TARGET_PATH.read_text())
    return dict(DEFAULT_TARGET)


@dataclass
class Plant:
    """One sampled wheel/sensor plant. Units: N.m, kg.m^2, s."""
    kt: float            # wheel torque per command unit (per wheel, N.m)
    armature: float      # reflected wheel inertia (relative coordinate)
    damping: float       # viscous wheel damping, N.m.s/rad
    coulomb: float       # Coulomb (stiction) wheel friction, N.m
    delay: float         # command->torque dead time, s
    lag: float           # first-order torque lag, s
    speed_scale: float   # wheel-speed sensor gain relative to the firmware conversion
    com_shift: float     # extra CoM offset along +x, m
    roll_resist: float = 0.0  # tyre rolling-resistance coefficient (m): soft floor / gear drag
    imu_delay: float = 0.0025  # extra IMU latency on top of the firmware filter path, s
    deadzone: float = 0.0  # servo input dead band, command units (the drive does nothing below it); 0 = none
    stiction: float = 0.0  # extra static wheel friction on top of the Coulomb level, N.m (Stribeck: fades with wheel speed)
    stribeck_speed: float = 0.4  # wheel speed (rad/s) over which that extra friction fades out

    def as_dict(self):
        return asdict(self)


# Nominal plant = best fit to the real FRF (see tools/fit_plant.py).
# tools/fit_plant.py (separate rate/wheel channel latencies): Kt 1.57e-4 N.m/unit, actuator dead time 1.7 ms,
# lag 9.6 ms, rate channel +5.2 ms, wheel channel +1.6 ms, wheel-speed reading 0.324 of the true speed.
NOMINAL = dict(armature=0.0, damping=2.0e-3, coulomb=0.0, delay=0.002, lag=0.0072, com_shift=0.0, imu_delay=0.0025)
# Closed-loop probe of the simulator (tools/sim_frf_check.py) reads ~15% below the measured pitch-rate gain and
# 6-16 degrees too much lag at the nominal fit values; this empirical gain/lag trim removes that residual.
KT_TRIM = 1.19
# Wheel-speed reading / true speed: the fit gives 0.324 (~1/pi); the probe check wants a further 15% down.
SPEED_SCALE = 0.28


def linearize(model, data, armature, damping, wheel_dofs):
    """Continuous-time (A, B) of the MuJoCo model about ``data`` with the given wheel impedance.

    State order is [qpos tangent (nv), qvel (nv)]; inputs are the model actuators.
    """
    base_arm = model.dof_armature.copy()
    base_damp = model.dof_damping.copy()
    base_fric = model.dof_frictionloss.copy()
    try:
        for i in wheel_dofs:
            model.dof_armature[i] = base_arm[i] + armature
            model.dof_damping[i] = base_damp[i] + damping
            model.dof_frictionloss[i] = 0.0
        nv = model.nv
        A = np.zeros((2 * nv, 2 * nv))
        B = np.zeros((2 * nv, model.nu))
        mujoco.mjd_transitionFD(model, data, 1e-6, True, A, B, None, None)
        dt = model.opt.timestep
        return (A - np.eye(2 * nv)) / dt, B / dt
    finally:
        model.dof_armature[:] = base_arm
        model.dof_damping[:] = base_damp
        model.dof_frictionloss[:] = base_fric


def _freq_response(model, data, hz, armature, damping, wheel_dofs):
    """Linear response of (body pitch rate, wheel relative speed) to common wheel torque."""
    base_arm = model.dof_armature.copy()
    base_damp = model.dof_damping.copy()
    base_fric = model.dof_frictionloss.copy()
    try:
        for i in wheel_dofs:
            model.dof_armature[i] = base_arm[i] + armature
            model.dof_damping[i] = base_damp[i] + damping
            model.dof_frictionloss[i] = 0.0
        nv = model.nv
        A = np.zeros((2 * nv, 2 * nv))
        B = np.zeros((2 * nv, model.nu))
        mujoco.mjd_transitionFD(model, data, 1e-6, True, A, B, None, None)
        dt = model.opt.timestep
        Ac = (A - np.eye(2 * nv)) / dt
        Bc = B / dt
        s = 2j * np.pi * hz
        X = np.linalg.solve(s * np.eye(2 * nv) - Ac, Bc[:, [0, 1]] @ np.array([1., 1.]))
        pitch_rate = X[nv + 4]
        wheel = 0.5 * (X[nv + wheel_dofs[0]] + X[nv + wheel_dofs[1]])
        return pitch_rate, wheel
    finally:
        model.dof_armature[:] = base_arm
        model.dof_damping[:] = base_damp
        model.dof_frictionloss[:] = base_fric


def calibrate(model, data, wheel_dofs, armature, damping, delay, lag, target=None):
    """Return (kt, speed_scale) that make this plant match the measured FRF.

    The robot must already be placed upright with the sampled mass properties.
    """
    target = target or load_target()
    hz = target['ref_hz']
    rate, wheel = _freq_response(model, data, hz, armature, damping, wheel_dofs)
    s = 2j * np.pi * hz
    act = abs(np.exp(-s * delay) / (1 + lag * s))
    kt = target['rate_per_unit'] / (abs(rate) * act)
    # firmware pitch-rate sign is opposite to physical, magnitude is what matters here
    ratio = abs(wheel) / abs(rate)
    speed_scale = target['wheel_to_rate'] / ratio
    return float(kt), float(speed_scale)


def sample(model, data, wheel_dofs, rng, domain, target=None, corner=False):
    """Sample a plant consistent with the measured response. ``domain`` in [0,1] scales spread.

    ``corner=True`` puts every randomised parameter at one end of its range (chosen at random):
    those episodes are where a controller with thin margins fails, so training must see them.
    """
    d = float(domain)

    def u(lo, hi):
        if corner and d > 0:
            return float(lo if rng.random() < .5 else hi)
        return float(rng.uniform(lo, hi))
    nominal = NOMINAL
    # Fit to the real FRF (tools/fit_plant.py): light wheels, back-EMF-like damping, ~13 ms of
    # command-to-torque latency, wheel-speed readings ~1/pi of the true rim speed.
    armature = max(0.0, nominal['armature'] + u(0., 5e-5) * d)
    damping = max(5e-4, nominal['damping'] * (1 + u(-0.5, 0.8) * d))
    delay = max(0.001, nominal['delay'] + u(-0.001, 0.005) * d)
    imu_delay = max(0.0, nominal['imu_delay'] + u(-0.0025, 0.0035) * d)
    lag = max(0.003, nominal['lag'] * (1 + u(-0.4, 0.6) * d))
    coulomb = u(0.0, 0.012) * d
    kt, _ = calibrate(model, data, wheel_dofs, armature, damping, delay, lag, target)
    kt *= KT_TRIM
    # the speed sensor scale is only identified through the 5-14 Hz wheel/rate ratio: 1/pi, +-30%
    speed_scale = SPEED_SCALE * (1 + u(-0.3, 0.3) * d)
    # the FRF pins kt only up to the scatter seen between runs (battery state, surface)
    kt *= 1 + u(-0.25, 0.35) * d
    com_shift = nominal['com_shift'] + u(-0.004, 0.014) * d
    return Plant(kt=kt, armature=armature, damping=damping, coulomb=coulomb, delay=delay, lag=lag,
                 speed_scale=speed_scale, com_shift=com_shift, imu_delay=imu_delay)
