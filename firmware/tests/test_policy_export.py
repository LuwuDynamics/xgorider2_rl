"""Actual generated C++ vs Python/PyTorch actor outputs; no physical training."""
import copy
import ctypes
import importlib.util
import json
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import export_policy as exporter


def fixture():
    rng = random.Random(10620)
    return dict(contract=6, architecture=[18, 32, 32, 4], name='numeric_fixture',
                trained=False, action_space='residual', action_scale=80, leg_action_scale=.005,
                hidden_activation='tanh', output_activation='clip', observation_clip=5,
                observation_mean=[rng.uniform(-.2, .2) for _ in range(18)],
                observation_std=[rng.uniform(.5, 2) for _ in range(18)],
                layers=[dict(weight=[[rng.uniform(-.3, .3) for _ in range(ni)] for _ in range(no)],
                             bias=[rng.uniform(-.1, .1) for _ in range(no)])
                        for ni, no in zip([18, 32, 32], [32, 32, 4])])


class PolicyExportTests(unittest.TestCase):
    def test_validation(self):
        good = fixture()
        for key, value in [('contract', 5), ('architecture', [18,32,32,2]), ('leg_action_scale',.03),
                           ('hidden_activation','relu'), ('output_activation','tanh'),
                           ('trained',1), ('name','bad"name'), ('action_space','torque'),
                           ('action_scale',129), ('observation_clip',float('nan')),
                           ('yaw_integral_gain',.21),('yaw_integral_gain',.04),('yaw_integral_limit',0),
                           ('observation_std',[0]*18), ('observation_mean',[float('inf')]*18)]:
            bad=copy.deepcopy(good);bad[key]=value
            with self.subTest(key=key), self.assertRaises(ValueError):exporter.validate(bad)
        direct=copy.deepcopy(good);direct.update(action_space='wheels',yaw_integral_gain=.2,yaw_integral_limit=.1)
        self.assertGreater(exporter.validate(direct)['yaw_integral_gain'],0)
        both=copy.deepcopy(good);both.update(action_space='wheels_legs',action_scale=1023,yaw_integral_gain=.08,yaw_integral_limit=.08)
        self.assertEqual(exporter.validate(both)['action_space'],'wheels_legs')
        bad=copy.deepcopy(good);bad['layers'][0]['weight'][0].pop()
        with self.assertRaises(ValueError):exporter.validate(bad)

    def test_generated_cpp_and_reference(self):
        data=exporter.validate(fixture())
        with tempfile.TemporaryDirectory(prefix='rider-policy-test-') as folder:
            root=Path(folder);header=root/'actor.h';header.write_text(exporter.render(data))
            bridge=root/'bridge.cpp'
            bridge.write_text('#include "actor.h"\nextern "C" bool predict(const float* x,float* y){'
                              'const auto& m=rider::policy::weights::compiledModel();'
                              'return rider::policy::validateModel(m)&&rider::policy::infer(m,x,y);}\n')
            library=root/'actor.dylib'
            subprocess.run([shutil.which('clang++') or 'c++','-std=c++17','-O1','-ffp-contract=off',
                            '-shared','-fPIC','-I',str(ROOT/'src'),str(bridge),
                            str(ROOT/'src/policy/policy.cpp'),'-o',str(library)],check=True)
            lib=ctypes.CDLL(str(library));lib.predict.restype=ctypes.c_bool
            lib.predict.argtypes=[ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_float)]
            rng=random.Random(921);maximum=0
            samples=[[0.0]*18,[1e6]*18,[-1e6]*18]
            samples += [[rng.uniform(-8,8) for _ in range(18)] for _ in range(256)]
            torch_actor=None
            if importlib.util.find_spec('torch'):
                import torch
                torch_actor=torch.nn.Sequential(torch.nn.Linear(18,32),torch.nn.Tanh(),
                    torch.nn.Linear(32,32),torch.nn.Tanh(),torch.nn.Linear(32,4)).eval()
                with torch.no_grad():
                    for layer,index in zip(data['layers'],(0,2,4)):
                        torch_actor[index].weight.copy_(torch.tensor(layer['weight']))
                        torch_actor[index].bias.copy_(torch.tensor(layer['bias']))
            for raw in samples:
                values=(ctypes.c_float*18)(*raw);output=(ctypes.c_float*4)()
                self.assertTrue(lib.predict(values,output))
                ref=exporter.reference_infer(data,list(values))
                for a,b in zip(output,ref):self.assertLessEqual(abs(a-b),2e-6)
                if torch_actor is not None:
                    with torch.no_grad():
                        x=(torch.tensor(list(values))-torch.tensor(data['observation_mean']))*torch.tensor(data['observation_inv_std'])
                        expected=torch_actor(x.clamp(-5,5)).clamp(-1,1).tolist()
                    error=max(abs(a-b) for a,b in zip(output,expected));maximum=max(maximum,error)
                    self.assertLessEqual(error,1e-5)
            values=(ctypes.c_float*18)(*[float('nan')]*18);output=(ctypes.c_float*4)(71,92,0,0)
            self.assertFalse(lib.predict(values,output));self.assertEqual(list(output),[71,92,0,0])
            print(f'Generated C++: {len(samples)} observations; PyTorch compared={torch_actor is not None}; max error={maximum:.9g}')

    @unittest.skipUnless(importlib.util.find_spec('torch'),'PyTorch is not installed')
    def test_torch_checkpoint_roundtrip(self):
        import torch
        data=fixture();layers=data.pop('layers');state={}
        for index,layer in zip((0,2,4),layers):
            state[f'{index}.weight']=torch.tensor(layer['weight'])
            state[f'{index}.bias']=torch.tensor(layer['bias'])
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'actor.pt';torch.save(dict(data,state_dict=state),path)
            actual=exporter.load(path,True)
            expected=exporter.validate(dict(data,layers=layers))
            self.assertEqual(actual,expected)
            self.assertEqual(exporter.metadata(actual),exporter.metadata(expected))
            header=Path(folder)/'policy_weights.h'
            subprocess.run([sys.executable,str(ROOT/'tools/export_policy.py'),str(path),'--torch','--output',str(header)],
                           check=True,capture_output=True,text=True)
            self.assertEqual(header.read_text(),exporter.render(expected))
            self.assertFalse(json.loads(header.with_suffix('.manifest.json').read_text())['trained'])


if __name__=='__main__':unittest.main()
