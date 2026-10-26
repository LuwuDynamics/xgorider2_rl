"""Expand the legacy two-output wheel actor into contract-5 wheel+leg outputs.

The legacy checkpoint was trained with the common-turn action wrapper, so its
two outputs are latent common/turn coordinates. The deployed header already
removes that transform algebraically; this script applies the same transform
to the final layer, zero-initialises the two new leg channels, and saves a
checkpoint that can be resumed directly with a four-output environment.
"""
import argparse
from pathlib import Path
import sys
import numpy as np
import torch
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rider_sim.env import RiderEnv  # noqa: E402


def expand(checkpoint, output, leg_scale=.005):
    torch.set_num_threads(1)
    old = PPO.load(checkpoint, device='cpu')
    if old.action_space.shape != (2,):
        raise ValueError(f'Expected a two-output checkpoint, got {old.action_space.shape}')
    env = RiderEnv(physics='measured', domain=0., commands=False, pushes=False, episode_seconds=5)
    new = PPO('MlpPolicy', env, policy_kwargs=dict(net_arch=dict(pi=[32, 32], vf=[64, 64]),
        activation_fn=torch.nn.Tanh, log_std_init=-3.5), learning_rate=1e-4, n_steps=256,
        batch_size=512, n_epochs=8, gamma=.995, gae_lambda=.95, clip_range=.15, ent_coef=0.,
        target_kl=.025, seed=91000, device='cpu', verbose=0)
    new_state = new.policy.state_dict()
    old_state = old.policy.state_dict()
    for key, value in new_state.items():
        source = old_state[key]
        if value.shape == source.shape:
            value.copy_(source)
        elif value.ndim == 2:
            rows = min(value.shape[0], source.shape[0])
            cols = min(value.shape[1], source.shape[1])
            value.zero_()
            value[:rows, :cols].copy_(source[:rows, :cols])
        elif value.ndim == 1:
            rows = min(value.shape[0], source.shape[0])
            value.zero_()
            value[:rows].copy_(source[:rows])
        else:
            raise ValueError(f'Unsupported transfer {key}: {source.shape} -> {value.shape}')
    scale = getattr(old, 'rider_common_turn_scale', None)
    if scale:
        weight = new_state['action_net.weight'].clone()
        bias = new_state['action_net.bias'].clone()
        new_state['action_net.weight'][0].copy_(weight[0] - scale * weight[1])
        new_state['action_net.weight'][1].copy_(-weight[0] - scale * weight[1])
        new_state['action_net.bias'][0].copy_(bias[0] - scale * bias[1])
        new_state['action_net.bias'][1].copy_(-bias[0] - scale * bias[1])
    with torch.no_grad():
        new.policy.log_std[2:] = torch.log(torch.tensor(.05))
    new.policy.load_state_dict(new_state)
    new.num_timesteps = old.num_timesteps
    new.rider_contract = 5
    new.rider_physics = 'measured'
    new.rider_leg_action_scale = leg_scale
    new.rider_yaw_integral_gain = getattr(old, 'rider_yaw_integral_gain', 0.)
    new.rider_yaw_integral_limit = getattr(old, 'rider_yaw_integral_limit', .05)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    new.save(output)
    env.close()
    print(f'expanded {checkpoint} -> {output} (common_turn_scale={scale}, leg_scale={leg_scale})')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoint', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--leg-scale', type=float, default=.005)
    args = parser.parse_args()
    expand(args.checkpoint, args.output, args.leg_scale)
