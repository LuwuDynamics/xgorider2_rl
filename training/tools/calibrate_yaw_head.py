"""Explicit supervised steering initialization before further measured-model PPO.

The balance head is unchanged. The steering head fits a low-gain yaw reference
controller to actual development rollouts; this is not RL training evidence.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import torch
from stable_baselines3 import PPO

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rider_sim.env import RiderEnv,OBS_MEAN,OBS_STD
from rider_sim.actions import predict_native


def main():
    parser=argparse.ArgumentParser();parser.add_argument('checkpoint',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    torch.set_num_threads(1);model=PPO.load(args.checkpoint,device='cpu');observations=[]
    for domain in (0.,1.):
        for seed in range(72200,72208):
            env=RiderEnv(physics='measured',domain=domain,commands=True,pushes=True,episode_seconds=20,
                yaw_integral_gain=model.rider_yaw_integral_gain)
            obs,_=env.reset(seed=seed)
            while True:
                observations.append(obs.copy());obs,_,done,trunc,_=env.step(predict_native(model,obs))
                if done or trunc:break
            env.close()
    x=np.asarray(observations);raw=x*OBS_STD+OBS_MEAN
    with torch.no_grad():hidden=model.policy.mlp_extractor.policy_net(torch.from_numpy(x)).numpy()
    design=np.column_stack([hidden,np.ones(len(hidden))]).astype(np.float64)
    feedforward=.0866/(2*.0308*41.89);proportional=.03
    # CommonTurnEnv: native wheel outputs both receive -scale*latent_turn.
    desired=(feedforward*raw[:,10]+proportional*(raw[:,10]-raw[:,4]))/model.rider_common_turn_scale
    ridge=np.eye(33)*.01;ridge[-1,-1]=1e-8
    weights=np.linalg.solve(design.T@design+ridge,design.T@desired)
    error=(design@weights-desired)*model.rider_common_turn_scale
    with torch.no_grad():
        model.policy.action_net.weight[1].copy_(torch.tensor(weights[:32],dtype=torch.float32))
        model.policy.action_net.bias[1].copy_(float(weights[32]))
    note=dict(kind='supervised yaw-head initialization; not PPO',source=str(args.checkpoint),
        samples=len(x),development_seeds=[72200,72207],domain=[0.,1.],
        yaw_feedforward=feedforward,yaw_proportional=proportional,
        fitted_native_command_rmse=float(np.sqrt(np.mean(error**2))),
        fitted_native_command_max_error=float(np.max(np.abs(error))),balance_head_unchanged=True)
    model.rider_yaw_head_initialization=note
    args.output.parent.mkdir(parents=True,exist_ok=True);model.save(args.output)
    args.output.with_suffix('.json').write_text(json.dumps(note,indent=2)+'\n');print(json.dumps(note,indent=2))


if __name__=='__main__':main()
