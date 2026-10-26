"""Regression tests for the contract-6 rebuild: calibrated plant, observation contract, margin analysis."""
import json
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
from rider_sim import plant as plantlib  # noqa: E402
from rider_sim.env2 import RiderEnv2, MASKED_INPUTS  # noqa: E402
from rider_sim.linear_policy import LinearPolicy  # noqa: E402
from rider_sim.native_policy import NativePolicy  # noqa: E402

FRF = json.loads((ROOT / 'models/measured_frf.json').read_text())
V6 = ROOT / 'artifacts/balance-v6'


class CalibratedPlant(unittest.TestCase):
    def test_measured_frf_file_is_complete(self):
        n = len(FRF['freqs_hz'])
        self.assertGreaterEqual(n, 6)
        for key in ('rate_mag', 'rate_phase_deg', 'wheel_mag', 'wheel_phase_deg'):
            self.assertEqual(len(FRF[key]), n)
        self.assertTrue(all(-180 < p < -60 for p in FRF['rate_phase_deg']))   # integrator plus lag, never a lead
        self.assertGreater(FRF['chunks'], 50)

    def test_simulator_reproduces_the_measured_response(self):
        import sim_frf_check
        policy = LinearPolicy()
        for hz in (7.5, 11.6):
            k = int(np.argmin(np.abs(np.array(FRF['freqs_hz']) - hz)))
            hr, hw, ok = sim_frf_check.probe(FRF['freqs_hz'][k], 100., 20., 11, policy=policy)
            self.assertTrue(ok)
            self.assertAlmostEqual(abs(hr) / FRF['rate_mag'][k], 1.0, delta=.15)
            self.assertAlmostEqual(abs(hw) / FRF['wheel_mag'][k], 1.0, delta=.2)
            dphase = np.degrees(np.angle(hr * np.exp(-1j * np.radians(FRF['rate_phase_deg'][k]))))
            self.assertLess(abs(dphase), 15.)

    def test_every_sampled_plant_keeps_the_measured_hf_gain(self):
        env = RiderEnv2(domain=1., commands=False, pushes=False, episode_seconds=1)
        wheels = [int(i) for i in env.wheel_v]
        target = plantlib.load_target()
        s = 2j * np.pi * target['ref_hz']
        for seed in range(6):
            env.reset(seed=seed)
            p = env.plant
            # calibration happens on the sampled masses *before* the CoM offset and wheel impedance are applied
            env.model.body_ipos[env.base, 0] -= p.com_shift
            saved = (env.model.dof_armature.copy(), env.model.dof_damping.copy())
            env.model.dof_armature[:], env.model.dof_damping[:] = env.base_armature, env.base_damping
            rate, _ = plantlib._freq_response(env.model, env.probe_pose(), target['ref_hz'], p.armature, p.damping, wheels)
            env.model.dof_armature[:], env.model.dof_damping[:] = saved
            env.model.body_ipos[env.base, 0] += p.com_shift
            gain = abs(rate) * p.kt * abs(np.exp(-s * p.delay) / (1 + p.lag * s))
            # kt = pinned value * KT_TRIM * run-to-run scatter (-25%..+35%, documented in plant.sample)
            ratio = gain / target['rate_per_unit']
            self.assertTrue(.99 * plantlib.KT_TRIM * .75 < ratio < 1.01 * plantlib.KT_TRIM * 1.35, (seed, ratio))
        env.close()


class ObservationContract(unittest.TestCase):
    def test_native_controller_matches_the_training_observation(self):
        native = NativePolicy(V6 / 'policy_weights.h')
        env = RiderEnv2(domain=1., commands=True, episode_seconds=8, native_policy=native, noise=1.0)
        env.reset(seed=3099)
        idx = [0, 1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 17]   # 17 = position integral, 5/6 = low-passed speeds
        worst = 0.
        while True:
            worst = max(worst, float(np.max(np.abs(env.raw_observation[idx] - native.observation[idx]))))
            _, _, done, trunc, _ = env.step(native.action.copy())
            if done or trunc:
                break
        self.assertFalse(done)
        self.assertLess(worst, 2e-5)
        env.close()
        native.close()

    def test_exported_actor_ignores_masked_inputs(self):
        data = json.loads((V6 / 'actor.json').read_text())
        w1 = np.array(data['layers'][0]['weight'])
        for column in MASKED_INPUTS:
            self.assertTrue(np.all(w1[:, column] == 0.), column)
        self.assertTrue(np.all(np.array(data['layers'][2]['weight'])[2:] == 0.))   # leg outputs pinned to zero

    def test_linear_core_stands_on_the_nominal_plant(self):
        env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=6, noise=1.0)
        env.reset(seed=4, options={'pitch': .04, 'roll': 0., 'yaw': 0.})
        policy = LinearPolicy()
        pitch = []
        while True:
            _, _, done, trunc, info = env.step(policy.action(env.raw_observation))
            pitch.append(info['pitch'])
            if done or trunc:
                break
        self.assertFalse(done)
        self.assertLess(np.degrees(np.std(pitch[len(pitch) // 2:])), .5)
        env.close()


class MarginAnalysis(unittest.TestCase):
    def test_previous_policies_ring_on_the_measured_plant_and_v6_does_not(self):
        import loop_margin as lm
        old = lm.local_gains(lm.load_actor(ROOT / 'artifacts/balance-legs-v5/actor.json'), -3.5)
        new = lm.local_gains(lm.load_actor(V6 / 'actor.json'), -3.5)
        old_rows = lm.data_loop(old, .012, FRF, 0.)
        new_rows = lm.data_loop(new, .006, FRF, lm.firmware_speed_tau())
        old_peak = 1 / min(r['one_minus'] for r in old_rows)
        new_peak = 1 / min(r['one_minus'] for r in new_rows)
        self.assertGreater(old_peak, 10.)        # the 7-14 Hz ringing seen on the robot
        self.assertLess(new_peak, 2.5)
        self.assertGreater(max(r['mag'] for r in old_rows if r['hz'] >= 8), 1.2)
        self.assertLess(max(r['mag'] for r in new_rows if r['hz'] >= 8), .6)


class DeadZone(unittest.TestCase):
    """Wheel dead zone / friction: the stepping cycle it causes and the learned preload that stops it (REBUILD.md section 6)."""

    @staticmethod
    def stand(friction_units, preload, seconds=45., seed=0):
        import dataclasses
        base = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=1)
        base.reset(seed=1)
        plant = dataclasses.replace(base.plant, coulomb=friction_units * 1.57e-4 * 1.19)
        base.close()
        native = NativePolicy(V6 / 'policy_weights.h')
        if preload:
            native.set_dead_zone(55., 8., 3., 8., .8)
        env = RiderEnv2(domain=0., commands=False, pushes=False, episode_seconds=seconds, native_policy=native, noise=1.0, plant_override=plant)
        env.reset(seed=seed)
        t, x = [], []
        while True:
            _, _, done, trunc, info = env.step(native.action.copy())
            t.append(info['time'])
            x.append(1000 * float(env.encoder_position()))
            if done or trunc:
                break
        k = native.dead_zone[0]
        env.close()
        native.close()
        t, x = np.array(t), np.array(x)
        late = t > seconds - 12
        return done, float(x[late].max() - x[late].min()), k

    def test_new_plant_fields_default_to_the_old_plant(self):
        import dataclasses
        fields = {f.name: f for f in dataclasses.fields(plantlib.Plant)}
        for name in ('deadzone', 'stiction'):
            self.assertEqual(fields[name].default, 0.0)

    def test_bridge_validates_and_reports_the_preload(self):
        native = NativePolicy(V6 / 'policy_weights.h')
        native.set_dead_zone(55., 8., 3., 8., .8)
        self.assertEqual(native.dead_zone, (0.0, 0, 0, 0.0))
        for bad in (dict(max_units=-1.), dict(max_units=81.), dict(max_units=40., hysteresis=1.), dict(max_units=40., backoff=1.), dict(max_units=40., chatter_max_u=5.)):
            with self.assertRaises(ValueError):
                native.set_dead_zone(**bad)
        native.close()

    def test_friction_makes_the_robot_step_and_the_preload_stops_it(self):
        fell, off_p2p, off_k = self.stand(45, preload=False)
        self.assertFalse(fell)
        self.assertGreater(off_p2p, 80.)                  # +-65 mm stick-slip cycle
        self.assertEqual(off_k, 0.)
        fell, on_p2p, on_k = self.stand(45, preload=True)
        self.assertFalse(fell)
        self.assertLess(on_p2p, 25.)
        self.assertGreater(on_k, 20.)                     # learned something close to the dead zone, never above the cap
        self.assertLessEqual(on_k, 55.)

    def test_a_frictionless_robot_never_learns_a_preload(self):
        fell, p2p, k = self.stand(0, preload=True, seconds=30.)
        self.assertFalse(fell)
        self.assertLess(k, 5.)
        self.assertLess(p2p, 15.)


if __name__ == '__main__':
    unittest.main()
