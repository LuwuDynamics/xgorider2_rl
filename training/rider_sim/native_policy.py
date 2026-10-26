"""Run the exported MCU Controller and actor inside MuJoCo, using the C++ sources."""
import ctypes
import hashlib
import json
import re
from pathlib import Path
import subprocess
import numpy as np
from .firmware import ROOT,FIRMWARE,build_shared_library


class NativePolicy:
    def __init__(self,header):
        header=Path(header).resolve()
        self.metadata=json.loads(re.search(r'^// policy_export: (.+)$',header.read_text(),re.M).group(1))
        sources=[ROOT/'native/policy_bridge.cpp',FIRMWARE/'src/policy/policy.cpp']
        digest=hashlib.sha256(b''.join(p.read_bytes() for p in [header,*sources,FIRMWARE/'src/policy/policy.h'])).hexdigest()[:16]
        output=ROOT/f'build/policy-{digest}.dylib';output.parent.mkdir(exist_ok=True)
        build_shared_library(output,['clang++','-std=c++17','-O2','-ffp-contract=off','-shared','-fPIC',
                '-I',str(FIRMWARE/'src'),f'-DPOLICY_HEADER="{header}"',*map(str,sources)])
        self.lib=ctypes.CDLL(str(output));ptr=ctypes.c_void_p
        fptr=np.ctypeslib.ndpointer(dtype=np.float32,flags='C_CONTIGUOUS')
        self.lib.policy_new.restype=ptr;self.lib.policy_free.argtypes=[ptr]
        self.lib.policy_reset.argtypes=[ptr];self.lib.policy_reset.restype=ctypes.c_int
        self.lib.policy_tick.argtypes=[ptr,ctypes.c_uint32,fptr,fptr]
        self.lib.policy_infer.argtypes=[fptr,fptr];self.lib.policy_infer.restype=ctypes.c_int
        self.lib.policy_inputs.restype=ctypes.c_uint
        self.lib.policy_set_dead_zone.argtypes=[ptr,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float];self.lib.policy_set_dead_zone.restype=ctypes.c_int
        self.lib.policy_reset_dead_zone.argtypes=[ptr]
        self.inputs=self.lib.policy_inputs()
        if self.inputs!=self.metadata['architecture'][0]:raise ValueError('Header and native observation size differ')
        self.pointer=self.lib.policy_new();self.output=np.zeros(self.inputs+21,np.float32)
        self.reset()

    @property
    def observation(self):return self.output[:self.inputs]
    @property
    def action(self):return self.output[self.inputs:self.inputs+4]
    @property
    def commands(self):return self.output[self.inputs+4:self.inputs+8]
    @property
    def wheels(self):return self.output[self.inputs+8:self.inputs+10]
    @property
    def legs(self):return self.output[self.inputs+10:self.inputs+12]
    @property
    def yaw_integral(self):return self.output[self.inputs+16]
    @property
    def dead_zone(self):
        """(learned preload in command units, relay sign, chatter trips, flip rate per second)"""
        o=self.output;n=self.inputs
        return float(o[n+17]),int(o[n+18]),int(o[n+19]),float(o[n+20])

    def set_dead_zone(self,max_units,hysteresis=8.,ramp=1.5,chatter_rate=8.,backoff=.8,chatter_max_u=30.,initial_units=0.,grow_mean_u=0.):
        if not self.lib.policy_set_dead_zone(self.pointer,max_units,hysteresis,ramp,chatter_rate,backoff,chatter_max_u,initial_units,grow_mean_u):raise ValueError('dead-zone configuration rejected')

    def reset(self):
        if not self.lib.policy_reset(self.pointer):raise ValueError('native model is invalid or untrained')

    def tick(self,env):
        v=env.out;feedback=env.feedback
        x=np.array([v[0],v[1],v[3],v[4]*180/np.pi,v[5]*180/np.pi,
            v[6]*1000*np.float32(61.6/55),v[7]*1000*np.float32(61.6/55),
            feedback[4],feedback[5],610,431,v[10]*1000,v[11]*180/np.pi,v[12]*36,env.encoder_position(),v[13]*20],np.float32)
        self.lib.policy_tick(self.pointer,env.us,x,self.output)
        if self.output[self.inputs+13]!=0 or self.output[self.inputs+14]!=2:
            raise RuntimeError(f'native policy fault: {self.output[self.inputs+13:self.inputs+15]}')
        return self.output

    def infer(self,raw):
        action=np.zeros(4,np.float32)
        if not self.lib.policy_infer(np.ascontiguousarray(raw,np.float32),action):raise ValueError('native inference failed')
        return action

    def close(self):
        if self.pointer:self.lib.policy_free(self.pointer);self.pointer=None
