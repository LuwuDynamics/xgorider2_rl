"""Rider2 balance environment, version 2.

Differences from ``env.py`` (see REBUILD.md for the evidence):

* the wheel drive is a torque source with measured latency/lag, sampled from the
  family of plants that reproduce the *real* frequency response
  (``plant.py``), instead of an assumed speed servo;
* the policy may act on every firmware control tick (6 ms) rather than every
  second tick, which removes ~6 ms of average sample-and-hold lag;
* wheel-speed sensor scaling, Coulomb friction, wheel impedance, CoM offset and
  larger IMU noise are randomised;
* the reward penalises command chatter explicitly (rate, jerk, body-rate) and
  records jitter metrics so training/evaluation can see the thing the robot
  actually does wrong.

The firmware C++ (IMU filter, leg controller, bus decoder, policy controller) is
still compiled and run in the loop, exactly as in ``env.py``.
"""
from collections import deque
import dataclasses
import math
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from .env import OBS_MEAN, OBS_STD, RADIUS, ENCODER_RADIUS, ROOT, quaternion
from .env import LEG_ACTION_SCALE, LEG_SLEW_PER_SECOND, LEG_TAU_DEFAULT
from .firmware import Firmware
from . import plant as plantlib

# Contract 6 replaces input 17 (yaw-integral state, never needed by the actor) with the
# firmware-side position-error integral: the only way a memoryless network can remove the
# steady lean/drift caused by an unknown centre-of-mass offset.
POS_INTEGRAL_LIMIT = 1.0
# Firmware low-pass on the wheel speeds the policy sees (contract 6): the wheel-speed channel is the least
# trustworthy one at 5-14 Hz (measured phase/gain scatter), so it is removed from the fast loop by structure.
SPEED_FILTER_TAU = 0.12
OBS_STD2 = OBS_STD.copy()
OBS_STD2[17] = 0.3
# Inputs the exported actor must ignore: timing jitter (15, 16), the previous wheel commands
# (13, 14: they act as a high-frequency amplifier) and leg encoder offsets (7, 8).
# The exporter zeroes the matching first-layer columns.
MASKED_INPUTS = (7, 8, 13, 14, 15, 16)
SPEED_UNITS = 1024 / (2 * math.pi * 4.8)  # firmware speed counts per rad/s


class RiderEnv2(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': 50}

    def __init__(self, domain=1.0, commands=True, pushes=True, episode_seconds=20, seed=None,
                 native_policy=None, yaw_integral_gain=.08, yaw_integral_limit=.08,
                 command_interval=(1.5, 3.), policy_every=1, noise=1.0, plant_override=None,
                 reward=None, control_ticks=None, feedback_ticks=None, render_mode=None,
                 stand_pitch=(-.06, .06), legs_enabled=False, push_scale=1.0, corner_fraction=0.,
                 friction_prob=0., friction_units=(10., 55.)):
        cad = ROOT / 'models/cad/rider_measured_cad.xml'
        # the CAD-mesh model has identical bodies, inertias and joints; it is only used when rendering
        self.model = mujoco.MjModel.from_xml_path(str(cad if render_mode and cad.exists() else ROOT / 'models/rider_measured.xml'))
        from .calibration import MeasuredCalibration
        self.calibration = MeasuredCalibration(self.model)
        self.data = mujoco.MjData(self.model)
        self.probe = mujoco.MjData(self.model)
        self.firmware = Firmware()
        self.action_space = spaces.Box(-1, 1, (4,), np.float32)
        self.observation_space = spaces.Box(-5, 5, (18,), np.float32)
        self.domain = float(domain)
        self.commands = commands
        self.pushes = pushes
        self.episode_seconds = episode_seconds
        self.native_policy = native_policy
        self.yaw_integral_gain = yaw_integral_gain
        self.yaw_integral_limit = yaw_integral_limit
        self.command_interval = command_interval
        self.policy_every = policy_every
        self.noise = noise
        self.plant_override = plant_override
        self.fixed_control_ticks = control_ticks
        self.fixed_feedback_ticks = feedback_ticks
        self.stand_pitch = stand_pitch
        self.legs_enabled = legs_enabled
        self.push_scale = push_scale
        self.corner_fraction = corner_fraction
        # training only: with this probability the sampled plant gets a wheel dead zone / friction of 'friction_units'
        # command units (the robot's is 12-45), as Coulomb friction, a servo input dead band or friction with stiction
        self.friction_prob = friction_prob
        self.friction_units = friction_units
        self.render_mode = render_mode
        self.renderer = None
        self.reward_cfg = dict(rate=6.0, jerk=3.0, magnitude=.05, roll=.5, body_rate=.02, wheel_tv=.0, still_v=.0, still_x=.0)
        if reward:
            self.reward_cfg.update(reward)
        m = self.model
        # real drivers deliver far more than the +/-0.2 N.m in the original XML
        m.actuator_ctrlrange[:2] = [-1., 1.]
        self.base = m.body('base').id
        self.ground = m.geom('ground').id
        self.wheel_bodies = [m.body(x).id for x in ('wheel11', 'wheel21')]
        self.wheel_q = np.array([m.jnt_qposadr[m.joint(x).id] for x in ('wheel11', 'wheel21')])
        self.wheel_v = np.array([m.jnt_dofadr[m.joint(x).id] for x in ('wheel11', 'wheel21')])
        self.leg_q = np.array([m.jnt_qposadr[m.joint(x).id] for x in ('extension12', 'extension22')])
        self.leg_v = np.array([m.jnt_dofadr[m.joint(x).id] for x in ('extension12', 'extension22')])
        self.tyre_geoms = [m.geom('tyre11').id, m.geom('tyre21').id]
        self.base_armature = m.dof_armature.copy()
        self.base_damping = m.dof_damping.copy()
        self.base_frictionloss = m.dof_frictionloss.copy()
        self.nominal_ipos = m.body_ipos.copy()
        self.body_velocity = np.zeros(6)
        self.reset(seed=seed)

    # ------------------------------------------------------------------ helpers
    def _angles(self):
        r = self.data.xmat[self.base].reshape(3, 3)
        return math.atan2(r[2, 1], r[2, 2]), math.asin(float(np.clip(-r[2, 0], -1, 1))), math.atan2(r[1, 0], r[0, 0])

    def axle_velocity(self):
        heading = self._angles()[2]
        velocity = 0.
        for body in self.wheel_bodies:
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body, self.body_velocity, 0)
            velocity += self.body_velocity[3] * math.cos(heading) + self.body_velocity[4] * math.sin(heading)
        return velocity * .5

    def encoder_position(self):
        return np.float32(np.float32(float(self.feedback[2] + self.feedback[3]) * .5 / 1024 * 3.1415 * .055) * np.float32(61.6 / 55))

    def _equilibrium_pitch(self):
        """Physical lean at which the sprung CoM is above the axle (zero wheel torque)."""
        m, d = self.model, self.data
        axle = np.mean([d.xpos[b] for b in self.wheel_bodies], axis=0)
        mass = 0.
        com = np.zeros(3)
        for i in range(1, m.nbody):
            if m.body_mass[i] <= 0 or i in self.wheel_bodies:
                continue
            mass += m.body_mass[i]
            com += m.body_mass[i] * d.xipos[i]
        com = com / mass - axle
        # express in the base frame
        r = d.xmat[self.base].reshape(3, 3)
        local = r.T @ com
        return -math.atan2(local[0], local[2])

    def _read_feedback(self, slot):
        if slot < 8:
            i = slot % 2
            direction = 1 if i == 0 else -1
            speed = round(float(self.data.qvel[self.wheel_v[i]]) * SPEED_UNITS * self.encoder_scale * self.plant.speed_scale) * direction
            if self.wheel_feedback_noise:
                speed = round(speed + float(self.np_random.normal(0, self.wheel_feedback_noise)))
            angle = float(self.data.qpos[self.wheel_q[i]]) / (2 * math.pi) * 1024 * self.encoder_scale
            position = math.floor(angle * direction) % 1024
            self.firmware.feedback((11, 21)[i], position, speed, self.us, self.feedback)
        else:
            i = slot - 8
            extension = float(self.data.qpos[self.leg_q[i]]) + .063
            ticks = round((1.1408376963350786 - math.acos(float(np.clip(extension / .096, -1, 1)))) / (2 * math.pi) * 1024)
            position = (610 - ticks) if i == 0 else (431 + ticks)
            if self.leg_feedback_noise:
                position = (position + int(round(float(self.np_random.normal(0, self.leg_feedback_noise))))) % 1024
            self.firmware.feedback((12, 22)[i], position, 0, self.us, self.feedback)

    def _control(self):
        self.out = self.firmware.control(self.feedback, self.command, self.us).copy()
        if self.ticks:
            self.odom_reference += float(self.out[10]) * self.control_ticks * .002
        if self.native_policy:
            self.native_policy.tick(self)

    def _apply_action(self):
        dt = self.control_ticks * .002
        if self.native_policy:
            self.applied[:] = self.native_policy.wheels
            self.yaw_integral = float(self.native_policy.yaw_integral)
            leg_action = np.asarray(self.native_policy.commands[2:4], np.float32)
        else:
            target = self.action[:2] * 1023
            step = .004 if self.first_control else dt
            self.yaw_integral = float(np.clip(
                self.yaw_integral + self.yaw_integral_gain * (self.out[11] - self.out[5]) * step,
                -self.yaw_integral_limit, self.yaw_integral_limit))
            target = np.clip(target - self.yaw_integral * 1023, -1023, 1023)
            delta = 80 if self.first_control else 20000 * step
            self.applied += np.clip(target - self.applied, -delta, delta)
            self.first_control = False
            leg_delta = LEG_SLEW_PER_SECOND * dt
            self.leg_action_applied += np.clip(self.action[2:4] - self.leg_action_applied, -leg_delta, leg_delta)
            leg_action = self.leg_action_applied
        command = np.sign(self.applied) * np.floor(np.abs(self.applied) + .5)
        self.last_command = command
        drive = command
        if self.plant.deadzone:   # servo input dead band: nothing happens until the command clears it
            drive = np.sign(command) * np.maximum(np.abs(command) - self.plant.deadzone, 0.)
        self.delay.append((self.us + self.plant_delay_us, np.array([drive[0], -drive[1]]) / 1023))
        base = np.asarray(self.out[16:18], np.float32)
        target = base + np.asarray(leg_action, np.float32) * LEG_ACTION_SCALE
        self.leg_servo += (target - self.leg_servo) * (1 - math.exp(-dt / self.leg_tau))
        for i in range(2):
            if self.leg_servo[i] > self.leg_hyst[i] + self.leg_backlash:
                self.leg_hyst[i] = self.leg_servo[i] - self.leg_backlash
            elif self.leg_servo[i] < self.leg_hyst[i] - self.leg_backlash:
                self.leg_hyst[i] = self.leg_servo[i] + self.leg_backlash
        self.data.ctrl[2:] = np.clip(self.leg_hyst - .063, -.018, .018)

    def _filtered_speeds(self):
        tick = .004 if self.ticks == 0 else self.control_ticks * .002
        raw = np.array([self.out[6], self.out[7]], np.float32)
        if self.speed_filter is None:
            self.speed_filter = raw.copy()
        else:
            alpha = np.float32(tick / (SPEED_FILTER_TAU + tick))
            self.speed_filter = self.speed_filter + alpha * (raw - self.speed_filter)
        return self.speed_filter

    def _observation(self, dt):
        v = self.out
        odom = self.encoder_position()
        speeds = self._filtered_speeds()
        position_error = odom - self.odom_reference
        self.raw_observation = np.array([
            v[0] * math.pi / 180, v[1] * math.pi / 180, v[3], v[4], v[5],
            speeds[0] * RADIUS / ENCODER_RADIUS, speeds[1] * RADIUS / ENCODER_RADIUS, v[8], v[9], v[10], v[11], v[12],
            odom - self.odom_reference,
            np.sign(self.applied[0]) * np.floor(abs(self.applied[0]) + .5) / 1023,
            np.sign(self.applied[1]) * np.floor(abs(self.applied[1]) + .5) / 1023, dt,
            .004 if self.ticks == 0 else self.control_ticks * .002, self.pos_integral], np.float32)
        normalized = np.clip((self.raw_observation - OBS_MEAN) / OBS_STD2, -5, 5).astype(np.float32)
        normalized[list(MASKED_INPUTS)] = 0.
        # firmware samples the integral before updating it with this tick's error
        tick = .004 if self.ticks == 0 else self.control_ticks * .002
        self.pos_integral = float(np.clip(self.pos_integral + position_error * tick, -POS_INTEGRAL_LIMIT, POS_INTEGRAL_LIMIT))
        return normalized

    # ------------------------------------------------------------------- reset
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        rng = self.np_random
        d = self.domain
        m, data = self.model, self.data
        self.physics_sample = self.calibration.apply(m, rng, d)
        self.supported_mass = self.physics_sample['fixed_chassis_mass_kg']
        m.geom_friction[:, 0] = rng.uniform(.55, 1.15) if d else .9
        # restore wheel joint parameters, then build the plant on the sampled masses
        m.dof_armature[:] = self.base_armature
        m.dof_damping[:] = self.base_damping
        m.dof_frictionloss[:] = self.base_frictionloss
        mujoco.mj_setConst(m, data)
        mujoco.mj_resetData(m, data)
        self._place_upright(data)
        mujoco.mj_forward(m, data)
        if self.plant_override is not None:
            self.plant = self.plant_override
        else:
            corner = bool(self.corner_fraction and rng.random() < self.corner_fraction)
            self.plant = plantlib.sample(m, self.probe_pose(), [int(i) for i in self.wheel_v], rng, d, corner=corner)
            if self.friction_prob and rng.random() < self.friction_prob:
                units = float(rng.uniform(*self.friction_units))
                kind = rng.random()
                if kind < .5:
                    self.plant = dataclasses.replace(self.plant, coulomb=units * self.plant.kt, deadzone=0., stiction=0.)
                elif kind < .75:
                    self.plant = dataclasses.replace(self.plant, coulomb=0., deadzone=units, stiction=0.)
                else:
                    self.plant = dataclasses.replace(self.plant, coulomb=.5 * units * self.plant.kt, deadzone=0.,
                                                     stiction=.5 * units * self.plant.kt)
        p = self.plant
        # CoM offset (applied after calibration so the FRF-pinned gain is for the measured robot)
        m.body_ipos[self.base, 0] += p.com_shift
        for i in self.wheel_v:
            m.dof_armature[i] = self.base_armature[i] + p.armature
            m.dof_damping[i] = self.base_damping[i] + p.damping
            m.dof_frictionloss[i] = self.base_frictionloss[i] + p.coulomb
        self.plant_delay_us = int(round(p.delay * 1e6 / 2000.)) * 2000
        self.imu_fifo = deque(maxlen=16)
        self.imu_steps = p.imu_delay / .002
        for g in self.tyre_geoms:
            m.geom_condim[g] = 6 if p.roll_resist > 0 else 3
            m.geom_friction[g, 2] = max(p.roll_resist, 1e-4)
        self.torque_tau = p.lag
        self.encoder_scale = 1 + rng.uniform(-.04, .04) * d
        self.control_ticks = self.fixed_control_ticks or (int(rng.choice([2, 3, 3, 3, 3, 3, 3, 4])) if d else 3)
        self.feedback_ticks = self.fixed_feedback_ticks or (int(rng.choice([1, 1, 1, 2, 2, 3])) if d else 1)
        n = self.noise
        self.gyro_bias = rng.uniform(-.5, .5, 3) * n
        self.accel_bias = rng.uniform(-.05, .05, 3) * n
        self.gyro_noise = (.1 + .4 * rng.uniform()) * n
        self.accel_noise = (.02 + .04 * rng.uniform()) * n
        self.wheel_feedback_noise = abs(rng.uniform(0, 2.5)) * n
        self.leg_feedback_noise = abs(rng.uniform(0, 1.5)) * n
        self.leg_tau = max(.006, LEG_TAU_DEFAULT + float(rng.uniform(-.014, .045)) * d)
        self.leg_backlash = abs(float(rng.uniform(0, .0012))) * d
        self.push_force = rng.uniform(.1, .8) * (0.4 + .6 * d) * self.push_scale
        self.push_roll_torque = rng.uniform(.004, .02) * (0.4 + .6 * d) * self.push_scale
        self.push_pitch_torque = rng.uniform(.01, .04) * (0.4 + .6 * d) * self.push_scale
        m.geom_quat[self.ground] = quaternion(0, rng.uniform(-.035, .035) * d, 0)
        mujoco.mj_setConst(m, data)
        mujoco.mj_resetData(m, data)
        pitch0 = options.get('pitch', rng.uniform(*self.stand_pitch))
        roll0 = options.get('roll', rng.uniform(-.012, .012))
        data.qpos[3:7] = quaternion(roll0, pitch0, options.get('yaw', rng.uniform(-math.pi, math.pi)))
        data.qpos[self.leg_q] = -.018
        data.ctrl[2:] = -.018
        data.qpos[2] = .0905
        mujoco.mj_forward(m, data)
        z = min(data.xpos[b, 2] for b in self.wheel_bodies)
        data.qpos[2] += RADIUS - z - .0002
        data.qfrc_applied[self.leg_v] = self.supported_mass * 9.81 * .5
        mujoco.mj_forward(m, data)
        self.equilibrium_pitch = self._equilibrium_pitch()
        self.us = self.ticks = 0
        self.firmware.reset(pitch0 * 180 / math.pi, roll0 * 180 / math.pi)
        self.feedback = np.array([0, 0, 0, 0, 610, 431, 0, 0], np.float32)
        for slot in range(10):
            self._read_feedback(slot)
        self.command = np.array([128, 128, 128], np.uint8)
        self.action = np.zeros(4, np.float32)
        self.previous_action = np.zeros(4, np.float32)
        self.previous_action2 = np.zeros(4, np.float32)
        self.applied = np.zeros(2)
        self.last_command = np.zeros(2)
        self.torque = np.zeros(2)
        self.drive = np.zeros(2)
        self.delay = deque()
        self.first_control = True
        self.yaw_integral = 0.
        self.pos_integral = 0.
        self.speed_filter = None
        self.leg_action_applied = np.zeros(2, np.float32)
        self.command_time = 1.
        self.push_time = 2.
        self.push_end = 0.
        self.push = np.zeros(3)
        self.push_torque = np.zeros(3)
        self.position_reference = 0.
        self.odom_reference = 0.
        self.travel = 0.
        self.elapsed = 0.
        self.steps = 0
        if self.native_policy:
            self.native_policy.reset()
        self._control()
        self.leg_servo = np.asarray(self.out[16:18], np.float32).copy()
        self.leg_hyst = self.leg_servo.copy()
        data.ctrl[2:] = np.clip(self.leg_hyst - .063, -.018, .018)
        return self._observation(.006), {}

    def _place_upright(self, data):
        data.qpos[2] = .0905
        data.qpos[3:7] = [1, 0, 0, 0]
        data.qpos[self.leg_q] = -.018
        data.ctrl[2:] = -.018
        data.qfrc_applied[self.leg_v] = self.supported_mass * 9.81 * .5
        mujoco.mj_forward(self.model, data)
        z = min(data.xpos[b, 2] for b in self.wheel_bodies)
        data.qpos[2] += RADIUS - z - .0002
        mujoco.mj_forward(self.model, data)

    def probe_pose(self):
        """A scratch MjData in the upright pose, used for the FRF calibration."""
        mujoco.mj_resetData(self.model, self.probe)
        self._place_upright(self.probe)
        return self.probe

    # -------------------------------------------------------------------- step
    def step(self, action):
        action = np.asarray(action, np.float32)
        if action.shape[-1] == 2:
            action = np.concatenate([action, np.zeros(action.shape[:-1] + (2,), np.float32)], -1)
        self.action = np.clip(action, -1, 1)
        if not self.legs_enabled:
            self.action[2:] = 0.
        old_action = self.previous_action
        ticks_in_step = self.policy_every
        if self.commands and self.elapsed >= self.command_time:
            forward = self.np_random.uniform(-.16, .16) if self.np_random.random() > .3 else 0.
            yaw = self.np_random.uniform(-.6, .6) if self.np_random.random() > .5 else 0.
            self.command[0] = round(127.5 + forward / .21 * 127.5)
            self.command[1] = round(127.5 + yaw / (300 * math.pi / 180) * 127.5)
            self.command[2] = int(self.np_random.uniform(55, 200)) if self.domain else 128
            self.command_time = self.elapsed + self.np_random.uniform(*self.command_interval)
        wheel_tv = 0.
        for _ in range(ticks_in_step):
            before = self.last_command.copy()
            self._apply_action()
            wheel_tv += float(np.sum(np.abs(self.last_command - before)))
            for _sub in range(self.control_ticks):
                self._physics_substep()
            self._control()
        dt_step = ticks_in_step * self.control_ticks * .002
        self.steps += 1
        roll, pitch, yaw = self._angles()
        velocity = self.axle_velocity()
        yaw_rate = float(self.data.sensordata[5])
        reference = float(self.out[10])
        turn = float(self.out[11])
        v_error = velocity - reference
        yaw_error = yaw_rate - turn
        self.travel += velocity * dt_step
        self.position_reference += reference * dt_step
        rates = self.data.sensordata[3:6]
        a = self.action
        c = self.reward_cfg
        rew = 1. + math.exp(-((pitch - self.equilibrium_pitch) / .2) ** 2) + math.exp(-(v_error / .12) ** 2)
        rew += 2. * math.exp(-(yaw_error / .08) ** 2)
        position_error = self.travel - self.position_reference
        rew += 1.2 * math.exp(-(position_error / .08) ** 2) - .25 * min((position_error / .15) ** 2, 100)
        rew -= c['magnitude'] * float(np.sum(a ** 2))
        rew -= c['rate'] * float(np.sum((a - old_action) ** 2))
        rew -= c['jerk'] * float(np.sum((a - 2 * old_action + self.previous_action2) ** 2))
        if c['still_v'] or c['still_x']:   # standing still: no wandering when nobody is driving the robot
            if abs(reference) < 1e-6:
                rew -= c['still_v'] * min((velocity / .03) ** 2, 16.)
                rew -= c['still_x'] * min((position_error / .03) ** 2, 16.)
        rew -= c['roll'] * roll ** 2
        rew -= c['body_rate'] * float(rates[1] ** 2) + 2 * c['body_rate'] * float(rates[0] ** 2)
        rew -= c['wheel_tv'] * wheel_tv / 1023.
        rew *= dt_step / .012
        self.previous_action2 = old_action.copy()
        self.previous_action = a.copy()
        terminated = bool(abs(pitch) > .65 or abs(roll) > .5 or self.data.qpos[2] < .045 or not np.isfinite(self.data.qpos).all())
        if terminated:
            rew -= 50
        truncated = self.elapsed >= self.episode_seconds
        info = dict(velocity=velocity, reference=reference, yaw_rate=yaw_rate, yaw_reference=turn, pitch=pitch,
                    roll=roll, time=self.elapsed, position_error=position_error, wheel_command=self.last_command.copy(),
                    fell=terminated, pitch_rate_fw=float(self.out[3]), wheel_tv=wheel_tv)
        return self._observation(dt_step), float(rew), terminated, truncated, info

    def _physics_substep(self):
        m, data = self.model, self.data
        while self.delay and self.delay[0][0] <= self.us:
            self.drive = self.delay.popleft()[1]
        wanted = np.clip(self.plant.kt * 1023 * self.drive, -.8, .8)
        self.torque += (wanted - self.torque) * (1 - math.exp(-.002 / self.torque_tau))
        data.ctrl[:2] = self.torque
        data.qfrc_applied[self.leg_v] = self.supported_mass * 9.81 * .5
        if self.pushes and self.elapsed >= self.push_time:
            ang = self.np_random.uniform(-math.pi, math.pi)
            self.push = np.array([math.cos(ang), math.sin(ang) * .8, 0]) * self.push_force
            heading = self._angles()[2]
            tx = self.np_random.uniform(-1, 1) * self.push_roll_torque
            ty = self.np_random.uniform(-1, 1) * self.push_pitch_torque
            self.push_torque = np.array([tx * math.cos(heading) - ty * math.sin(heading),
                                         tx * math.sin(heading) + ty * math.cos(heading), 0])
            self.push_end = self.elapsed + .08
            self.push_time = self.elapsed + self.np_random.uniform(1., 3.)
        active = self.elapsed < self.push_end
        data.xfrc_applied[self.base, :3] = self.push if active else 0
        data.xfrc_applied[self.base, 3:6] = self.push_torque if active else 0
        if self.plant.stiction:   # Stribeck: the wheel is harder to start than to keep moving
            for i in self.wheel_v:
                m.dof_frictionloss[i] = self.base_frictionloss[i] + self.plant.coulomb + self.plant.stiction * math.exp(-(data.qvel[i] / self.plant.stribeck_speed) ** 2)
        mujoco.mj_step(m, data)
        self.ticks += 1
        self.us += 2000
        self.elapsed = self.us / 1e6
        if self.ticks % self.feedback_ticks == 0:
            self._read_feedback((self.ticks // self.feedback_ticks - 1) % 10)
        acceleration = data.sensordata[:3]
        omega = data.sensordata[3:6]
        native_acc = np.array([acceleration[1], -acceleration[0], acceleration[2]])
        native_gyro = np.array([-omega[0], -omega[1], omega[2]]) * 180 / math.pi
        if self.noise:
            native_acc = native_acc + self.accel_bias + self.np_random.normal(0, self.accel_noise, 3)
            native_gyro = native_gyro + self.gyro_bias + self.np_random.normal(0, self.gyro_noise, 3)
        # IMU latency beyond the firmware's own filter path (identified from the real rate-channel phase)
        self.imu_fifo.append((native_acc.copy(), native_gyro.copy()))
        steps = self.imu_steps
        whole = int(steps)
        frac = steps - whole
        newest = min(whole, len(self.imu_fifo) - 1)
        older = min(whole + 1, len(self.imu_fifo) - 1)
        acc = (1 - frac) * self.imu_fifo[-1 - newest][0] + frac * self.imu_fifo[-1 - older][0]
        gyro = (1 - frac) * self.imu_fifo[-1 - newest][1] + frac * self.imu_fifo[-1 - older][1]
        if np.dot(acc, acc) > .4:
            self.firmware.sense(acc, gyro, self.us)

    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=720, width=1280)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = self.data.xpos[self.base]
        camera.distance = .48
        camera.azimuth = 135
        camera.elevation = -20
        self.renderer.update_scene(self.data, camera)
        return self.renderer.render()

    def close(self):
        self.firmware.close()
        if self.renderer is not None:
            self.renderer.close()
