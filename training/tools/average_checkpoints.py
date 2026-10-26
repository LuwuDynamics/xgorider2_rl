"""Average the policy weights of several PPO checkpoints (stochastic weight averaging).

PPO iterates wander around a good region; their mean is usually smoother and more robust than any single
iterate.  The result is only accepted if it beats the individual checkpoints in evaluate2.py.
"""
import argparse
from pathlib import Path
import torch
from stable_baselines3 import PPO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoints', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    models = [PPO.load(p, device='cpu') for p in args.checkpoints]
    state = {k: v.clone().float() for k, v in models[0].policy.state_dict().items()}
    for key in state:
        state[key] = sum(m.policy.state_dict()[key].float() for m in models) / len(models)
    models[0].policy.load_state_dict(state)
    models[0].save(args.output)
    print('averaged', len(models), 'checkpoints ->', args.output)


if __name__ == '__main__':
    main()
