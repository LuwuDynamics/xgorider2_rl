"""Export a contract-6 PPO checkpoint to the MCU weight header.

Differences from ``export_actor.py``:

* observation contract 6 (input 17 = position-error integral, normalised with ``OBS_STD2``),
* inputs the policy must ignore (timing, previous commands, leg encoders) are zeroed in the first
  layer, so the exported network is *mathematically independent* of them even though the firmware
  still supplies them,
* the leg output rows are zero (this iteration trains the wheels only; the contract keeps the two
  leg channels for a later step),
* provenance records the training stages honestly (robust linear init -> behaviour cloning -> PPO).
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import torch
from stable_baselines3 import PPO
from rider_sim.env2 import OBS_MEAN, OBS_STD2, MASKED_INPUTS
from rider_sim.firmware import FIRMWARE


def export(checkpoint, output, name, require_ppo=True, trained_steps=None, note=None):
    torch.set_num_threads(1)
    model = PPO.load(checkpoint, device='cpu')
    config_path = Path(checkpoint).parent / 'config.json'
    config = json.loads(config_path.read_text()) if config_path.exists() else {}
    if config.get('observation_contract') != 6:
        raise ValueError('Checkpoint is not recorded as trained with observation contract 6')
    if require_ppo and model.num_timesteps <= 0:
        raise ValueError('No PPO steps recorded: a behaviour-cloned init is not a reinforcement-learning result')
    if model.observation_space.shape != (18,) or model.action_space.shape != (4,):
        raise ValueError('Checkpoint must have 18 observations and 4 actions')
    network = model.policy.mlp_extractor.policy_net
    if len(network) != 4 or not all(isinstance(network[i], torch.nn.Tanh) for i in (1, 3)):
        raise ValueError('Unsupported actor architecture')
    state = {}
    for target, source in ((0, network[0]), (2, network[2]), (4, model.policy.action_net)):
        for suffix in ('weight', 'bias'):
            state[f'{target}.{suffix}'] = getattr(source, suffix).detach().cpu().clone()
    # inputs the actor must not depend on
    for column in MASKED_INPUTS:
        state['0.weight'][:, column] = 0.
    state['4.weight'][2:] = 0.
    state['4.bias'][2:] = 0.
    data = dict(contract=6, architecture=[18, 32, 32, 4], name=name, trained=True, hidden_activation='tanh',
                output_activation='clip', action_space='wheels_legs', action_scale=1023., leg_action_scale=.005,
                observation_clip=5., observation_mean=OBS_MEAN.tolist(), observation_std=OBS_STD2.tolist(),
                state_dict=state, yaw_integral_gain=.08, yaw_integral_limit=.08)
    output.mkdir(parents=True, exist_ok=True)
    torch.save(data, output / 'actor.pt')
    spec = importlib.util.spec_from_file_location('rider_policy_export', FIRMWARE / 'tools/export_policy.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    validated = exporter.load(output / 'actor.pt', True)
    (output / 'actor.json').write_text(json.dumps(validated, indent=2) + '\n')
    (output / 'policy_weights.h').write_text(exporter.render(validated))
    info = exporter.metadata(validated)
    (output / 'policy_weights.manifest.json').write_text(json.dumps(info, indent=2) + '\n')
    shutil.copy2(checkpoint, output / 'policy.zip')
    if config_path.exists():
        shutil.copy2(config_path, output / 'training_config.json')
    provenance = dict(
        created_at=datetime.now(timezone.utc).isoformat(), trained_steps=trained_steps or model.num_timesteps, checkpoint_note=note,
        checkpoint=str(Path(checkpoint).resolve()), checkpoint_sha256=hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest(),
        model_sha256=info['sha256'], observation_contract=6,
        training='robust linear initialisation (delay-aware, stress-tested) -> behaviour cloning -> critic warm-up -> '
                 'MuJoCo PPO on the measured-plant family; checkpoint chosen by evaluate2 acceptance score',
        masked_inputs=list(MASKED_INPUTS), legs='pinned to zero in this iteration',
        deployment='native float32 actor; no ONNX Runtime or Python on ESP32', hardware_verified=False,
        training_configuration=config)
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print(json.dumps(dict(**info, trained_steps=trained_steps or model.num_timesteps, output=str(output)), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('checkpoint', type=Path)
    parser.add_argument('--output', type=Path, default=Path('artifacts/v2-candidate'))
    parser.add_argument('--name', default='rider_balance_v6')
    parser.add_argument('--trained-steps', type=int, help='PPO steps behind the weights (needed for averaged checkpoints)')
    parser.add_argument('--note', help='how the checkpoint was produced (e.g. weight average of steps ...)')
    parser.add_argument('--allow-untrained', action='store_true', help='export the behaviour-cloned init (for analysis only)')
    args = parser.parse_args()
    export(args.checkpoint, args.output, args.name, require_ppo=not args.allow_untrained, trained_steps=args.trained_steps, note=args.note)
