#!/usr/bin/env python3
"""Export a validated 18-32-32-4 tanh actor (two wheels, two leg offsets) to
allocation-free ESP32 C++ weights.

Input is explicit JSON, or --torch for a weights-only PyTorch checkpoint using
the same metadata and a Sequential actor state_dict (Linear at keys 0, 2, 4).
Export does not train a model or certify its physical stability.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct

ARCHITECTURE = [18, 32, 32, 4]
CONTRACT = 6
ROOT = Path(__file__).resolve().parents[1]


def f32(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('Model values must be finite numbers')
    try:
        result = struct.unpack('<f', struct.pack('<f', value))[0]
    except (OverflowError, struct.error) as exc:
        raise ValueError('Value cannot be represented in float32') from exc
    if not math.isfinite(result):
        raise ValueError('Nonfinite float32 value')
    return result


def vector(value, size):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f'Expected vector of length {size}')
    return [f32(v) for v in value]


def validate(data):
    if not isinstance(data, dict):
        raise ValueError('Expected a model dictionary')
    if data.get('contract') != CONTRACT or data.get('architecture') != ARCHITECTURE:
        raise ValueError('Expected contract 6 and architecture [18,32,32,4]')
    if data.get('hidden_activation') != 'tanh' or data.get('output_activation') != 'clip':
        raise ValueError('Actor must use tanh hidden layers and linear output clipped to [-1,1]')
    name = data.get('name', '')
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,48}', name):
        raise ValueError('Name must contain 1..48 ASCII letters, digits, dot, underscore or hyphen')
    if type(data.get('trained')) is not bool:
        raise ValueError('trained must explicitly be true or false; export does not establish training')
    space = data.get('action_space')
    if space not in ('residual', 'wheels', 'wheels_legs'):
        raise ValueError('action_space must be residual, wheels or wheels_legs')
    scale = f32(data.get('action_scale'))
    if not 0 < scale <= (128 if space == 'residual' else 1023):
        raise ValueError('action_scale exceeds firmware action limits')
    leg_scale = f32(data.get('leg_action_scale', 0.005))
    if not 0 < leg_scale <= 0.02:
        raise ValueError('leg_action_scale must be in (0,0.02] metres')
    obs_clip = f32(data.get('observation_clip', 5))
    if not 0 < obs_clip <= 100:
        raise ValueError('observation_clip must be in (0,100]')
    mean = vector(data.get('observation_mean'), 18)
    std = vector(data.get('observation_std'), 18)
    if any(v <= 0 for v in std):
        raise ValueError('Observation standard deviations must be positive')
    inv_std = [f32(1 / v) for v in std]
    yaw_gain=f32(data.get('yaw_integral_gain',0));yaw_limit=f32(data.get('yaw_integral_limit',.05))
    if not 0<=yaw_gain<=f32(.2) or not 0<yaw_limit<=f32(.1) or (yaw_gain>0 and space=='residual'):
        raise ValueError('Invalid yaw PI profile; gain <=0.2, limit <=0.1, direct wheels only')
    layers = data.get('layers')
    if not isinstance(layers, list) or len(layers) != 3:
        raise ValueError('Expected exactly three dense layers')
    cleaned = []
    for layer, ni, no in zip(layers, ARCHITECTURE, ARCHITECTURE[1:]):
        if not isinstance(layer, dict):
            raise ValueError('Each layer must contain weight and bias')
        weight = layer.get('weight')
        if not isinstance(weight, list) or len(weight) != no:
            raise ValueError(f'Expected weight shape [{no},{ni}], output-major')
        cleaned.append(dict(weight=[vector(row, ni) for row in weight], bias=vector(layer.get('bias'), no)))
    return dict(contract=CONTRACT, architecture=ARCHITECTURE, name=name,
                trained=data['trained'], action_space=space, action_scale=scale, leg_action_scale=leg_scale,
                hidden_activation='tanh', output_activation='clip', observation_clip=obs_clip,
                observation_mean=mean, observation_std=std, observation_inv_std=inv_std, layers=cleaned,
                yaw_integral_gain=yaw_gain,yaw_integral_limit=yaw_limit)


def load(path, torch_checkpoint=False):
    if not torch_checkpoint:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    else:
        import torch
        data = torch.load(path, map_location='cpu', weights_only=True)
        if not isinstance(data, dict):
            raise ValueError('Expected metadata dictionary containing actor state_dict')
        data = dict(data)
        state = data.pop('state_dict', {})
        expected = {f'{i}.{suffix}' for i in (0, 2, 4) for suffix in ('weight', 'bias')}
        if set(state) != expected:
            raise ValueError('Expected actor-only Sequential state_dict with Linear keys 0, 2, 4')
        data['layers'] = [dict(weight=state[f'{i}.weight'].detach().cpu().tolist(),
                               bias=state[f'{i}.bias'].detach().cpu().tolist()) for i in (0, 2, 4)]
    if not isinstance(data, dict):
        raise ValueError('Expected a model dictionary')
    return validate(data)


def metadata(data):
    canonical = json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    digest = hashlib.sha256(canonical).hexdigest()
    parameters = sum(len(layer['weight'])*len(layer['weight'][0]) + len(layer['bias'])
                     for layer in data['layers'])
    return dict(name=data['name'], trained=data['trained'], action_space=data['action_space'],
                action_scale=data['action_scale'], leg_action_scale=data['leg_action_scale'],
                contract=CONTRACT, architecture=ARCHITECTURE,
                sha256=digest, model_id=int(digest[:8], 16), parameters=parameters,
                parameter_bytes=parameters*4, normalization_bytes=18*8,
                policy_period_us=3000, inference_deadline_us=2000,
                max_policy_gap_us=30000, action_slew_units_per_second=20000,
                leg_action_slew_units_per_second=10,
                direct_wheel_reference_limits=dict(forward_mm_s=165,turn_deg_s=36,height_units=[0,29],roll_command_deg=4),
                yaw_integral_gain=data['yaw_integral_gain'],yaw_integral_limit=data['yaw_integral_limit'],
                on_device_profiled=False)


def literal(value):
    text = format(value, '.9g')
    if '.' not in text and 'e' not in text.lower():
        text += '.0'
    return text + 'f'


def render(data):
    info = metadata(data)
    lines = ['#pragma once', '#include "policy/policy.h"',
             '// Generated by tools/export_policy.py; do not hand-edit weights.',
             '// policy_export: ' + json.dumps(info, separators=(',', ':')),
             'namespace rider { namespace policy { namespace weights {',
             f'constexpr unsigned kExportedContract={CONTRACT};',
             'inline const Model& compiledModel() {']

    def array(name, values):
        lines.append(f'    static const float {name}[{len(values)}]={{')
        for offset in range(0, len(values), 8):
            lines.append('        ' + ','.join(literal(v) for v in values[offset:offset+8]) + ',')
        lines.append('    };')

    array('mean', data['observation_mean'])
    array('inv_std', data['observation_inv_std'])
    for i, layer in enumerate(data['layers'], 1):
        array('w' + str(i), [v for row in layer['weight'] for v in row])
        array('b' + str(i), layer['bias'])
    space = {'residual':'Residual','wheels':'Wheels','wheels_legs':'WheelsLegs'}[data['action_space']]
    lines += [f'    static const Model model={{"{data["name"]}",{info["model_id"]}u,{CONTRACT},'
              f'{str(data["trained"]).lower()},ActionSpace::{space},'
              f'{literal(data["action_scale"])},{literal(data["leg_action_scale"])},{literal(data["observation_clip"])},',
              f'        mean,inv_std,w1,b1,w2,b2,w3,b3,{literal(data["yaw_integral_gain"])},{literal(data["yaw_integral_limit"])}}};',
              '    return model;', '}', '}}}', '']
    return '\n'.join(lines)


def reference_infer(data, raw):
    """Float32 reference, including rounding at every multiply and accumulation."""
    x = vector(raw, 18)
    x = [max(-data['observation_clip'], min(data['observation_clip'],
         f32(f32(v - mean) * inv))) for v, mean, inv in
         zip(x, data['observation_mean'], data['observation_inv_std'])]
    for i, layer in enumerate(data['layers']):
        result = []
        for row, bias in zip(layer['weight'], layer['bias']):
            value = bias
            for weight, value_in in zip(row, x):
                value = f32(value + f32(weight * value_in))
            result.append(f32(math.tanh(value)) if i < 2 else max(-1.0, min(1.0, value)))
        x = result
    return x


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--torch', action='store_true', help='Read a weights-only PyTorch checkpoint')
    parser.add_argument('--output', type=Path, default=ROOT / 'src/policy/policy_weights.h')
    args = parser.parse_args()
    try:
        data = load(args.input, args.torch)
        header = render(data)
    except (ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(header, encoding='utf-8')
    report = args.output.with_suffix('.manifest.json')
    report.write_text(json.dumps(metadata(data), indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(header=str(args.output), manifest=str(report), **metadata(data)), indent=2))


if __name__ == '__main__':
    main()
