"""Verify an exported contract-6 actor against the MCU C++ (float32 inference, scheduling, observation)."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from rider_sim.env2 import RiderEnv2, OBS_MEAN, OBS_STD2, MASKED_INPUTS, POS_INTEGRAL_LIMIT
from rider_sim.native_policy import NativePolicy


class CheckedController(NativePolicy):
    """Native controller plus an independent reference of the post-network stages."""

    def reset(self):
        super().reset()
        self.reference_integral = 0.
        self.reference_command = np.zeros(4)
        self.expected_pos_integral = 0.
        self.last_tick = None
        self.maximum_command_error = getattr(self, 'maximum_command_error', 0.)
        self.maximum_yaw_integral_error = getattr(self, 'maximum_yaw_integral_error', 0.)
        self.maximum_pos_integral_error = getattr(self, 'maximum_pos_integral_error', 0.)

    def tick(self, env):
        dt = .004 if self.last_tick is None else (env.us - self.last_tick) / 1e6
        self.last_tick = env.us
        gain = self.metadata.get('yaw_integral_gain', 0.)
        limit = self.metadata.get('yaw_integral_limit', .05)
        self.reference_integral = float(np.clip(self.reference_integral + gain * (env.out[11] - env.out[5]) * dt, -limit, limit))
        result = super().tick(env)
        observed = float(self.observation[17])
        self.maximum_pos_integral_error = max(self.maximum_pos_integral_error, abs(observed - self.expected_pos_integral))
        self.expected_pos_integral = float(np.clip(self.expected_pos_integral + float(self.observation[12]) * dt,
                                                   -POS_INTEGRAL_LIMIT, POS_INTEGRAL_LIMIT))
        target = np.zeros(4, np.float32)
        target[:2] = np.clip((self.action[:2] - self.reference_integral) * self.metadata['action_scale'], -1023, 1023)
        target[2:] = np.clip(self.action[2:], -1, 1)
        delta = np.array([20000 * dt, 20000 * dt, 10 * dt, 10 * dt], np.float32)
        self.reference_command += np.clip(target - self.reference_command, -delta, delta)
        self.maximum_command_error = max(self.maximum_command_error, float(np.max(np.abs(self.reference_command - self.commands))))
        self.maximum_yaw_integral_error = max(self.maximum_yaw_integral_error, abs(self.reference_integral - float(self.yaw_integral)))
        return result


def exported_actor(folder):
    data = json.loads((folder / 'actor.json').read_text())
    layers = [(np.array(l['weight'], np.float64), np.array(l['bias'], np.float64)) for l in data['layers']]
    mean = np.array(data['observation_mean'], np.float64)
    std = np.array(data['observation_std'], np.float64)

    def forward(raw):
        x = np.clip((raw - mean) / std, -5, 5)
        h = np.tanh(layers[0][0] @ x + layers[0][1])
        h = np.tanh(layers[1][0] @ h + layers[1][1])
        return np.clip(layers[2][0] @ h + layers[2][1], -1, 1)
    return forward


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    model = PPO.load(args.folder / 'policy.zip', device='cpu')
    native = CheckedController(args.folder / 'policy_weights.h')
    reference = exported_actor(args.folder)
    rng = np.random.default_rng(10620)

    # 1. dense kernel: random observations, C++ float32 vs float64 reference of the exported weights
    raw = rng.uniform(-5, 5, (4096, 18)).astype(np.float32) * OBS_STD2 + OBS_MEAN
    raw = np.concatenate([raw, np.zeros((1, 18), np.float32), np.full((1, 18), 1e6, np.float32), np.full((1, 18), -1e6, np.float32)])
    cpp = np.stack([native.infer(x) for x in raw])
    ref = np.stack([reference(x.astype(np.float64)) for x in raw])
    random_error = float(np.max(np.abs(cpp - ref)))
    if random_error > 2e-5:
        raise AssertionError(('dense actor export mismatch', random_error))

    # 2. the exported actor must equal the trained SB3 actor on the observations it actually receives
    #    (masked inputs are zeroed by the environment and by the zeroed first-layer columns)
    normalized = np.clip((raw - OBS_MEAN) / OBS_STD2, -5, 5).astype(np.float32)
    normalized[:, list(MASKED_INPUTS)] = 0.
    trained = model.predict(normalized, deterministic=True)[0]
    trained = np.clip(trained, -1, 1)
    trained[:, 2:] = 0.
    sb3_error = float(np.max(np.abs(trained[:, :2] - cpp[:, :2])))
    # inputs seen by the firmware have the masked values present; the exported actor must ignore them
    perturbed = raw.copy()
    perturbed[:, list(MASKED_INPUTS)] = rng.uniform(-5, 5, (len(raw), len(MASKED_INPUTS))) * OBS_STD2[list(MASKED_INPUTS)]
    mask_independence = float(np.max(np.abs(np.stack([native.infer(x) for x in perturbed[:512]]) - cpp[:512])))
    if mask_independence > 1e-6:
        raise AssertionError(('exported actor depends on masked inputs', mask_independence))

    # 3. closed loop in the simulator with the native controller
    raw_error = action_error = 0.
    samples = 0
    episodes = []
    for domain in (0., 1.):
        for seed in (3001, 3017, 3099):
            env = RiderEnv2(domain=domain, commands=True, episode_seconds=15, native_policy=native, noise=1.0)
            obs, _ = env.reset(seed=seed)
            while True:
                raw_error = max(raw_error, float(np.max(np.abs(env.raw_observation[[0, 1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 17]]
                                                           - native.observation[[0, 1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 17]]))))
                expected = reference(native.observation.astype(np.float64))
                action = native.action.copy()
                action_error = max(action_error, float(np.max(np.abs(expected - action))))
                samples += 1
                obs, _, done, trunc, info = env.step(action)
                if done or trunc:
                    break
            episodes.append(dict(domain=domain, seed=seed, survived=not done, elapsed=info['time']))
            env.close()
    native.close()
    if raw_error > 2e-5 or action_error > 2e-5:
        raise AssertionError(('firmware contract mismatch', raw_error, action_error))
    if native.maximum_command_error > .02 or native.maximum_yaw_integral_error > 5e-6 or native.maximum_pos_integral_error > 5e-6:
        raise AssertionError(('native post-controller mismatch', native.maximum_command_error,
                              native.maximum_yaw_integral_error, native.maximum_pos_integral_error))
    report = dict(random_samples=len(raw), trajectory_samples=samples, maximum_random_action_error=random_error,
                  sb3_vs_native_wheel_error=sb3_error, masked_input_independence=mask_independence,
                  model_sha256=native.metadata['sha256'],
                  checkpoint_sha256=hashlib.sha256((args.folder / 'policy.zip').read_bytes()).hexdigest(),
                  weights_header_sha256=hashlib.sha256((args.folder / 'policy_weights.h').read_bytes()).hexdigest(),
                  maximum_trajectory_action_error=action_error, maximum_raw_observation_error=raw_error,
                  episodes=episodes, maximum_command_error_units=native.maximum_command_error,
                  maximum_yaw_integral_error=native.maximum_yaw_integral_error,
                  maximum_pos_integral_error=native.maximum_pos_integral_error,
                  check='exported weights, native C++ float32 inference, Controller scheduling, observation construction, '
                        'position/yaw integrals and per-tick rate limiting',
                  mcu_timing_verified=False)
    (args.folder / 'native_verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
