import ctypes
import hashlib
import fcntl
import os
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
FIRMWARE=ROOT.parent/'firmware'
_library=None


def build_shared_library(output,arguments):
    """Keep independent evaluator processes from loading a half-linked library."""
    output.parent.mkdir(exist_ok=True)
    with output.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if output.exists():return
        temporary=output.with_name(output.stem+f'-{os.getpid()}.tmp.dylib')
        try:
            subprocess.run([*arguments,'-o',str(temporary)],check=True)
            os.replace(temporary,output)
        finally:
            temporary.unlink(missing_ok=True)


def library():
    global _library
    if _library is not None:return _library
    sources=[ROOT/'native/firmware_bridge.cpp']+[FIRMWARE/'src'/p for p in
        ('leg_position_controller.cpp','actuator_bus.cpp','control/pid.cpp','control/rider_motion.cpp','control/rider_leg_imu.cpp')]
    headers=list((FIRMWARE/'src/control').glob('*.h'))+[FIRMWARE/'src/motion_controller.h',FIRMWARE/'src/leg_position_controller.h',FIRMWARE/'src/actuator_bus.h']
    digest=hashlib.sha256(b''.join(p.read_bytes() for p in sources+headers)).hexdigest()[:16]
    output=ROOT/f'build/firmware-{digest}.dylib';output.parent.mkdir(exist_ok=True)
    build_shared_library(output,['clang++','-std=c++17','-O2','-ffp-contract=off','-shared','-fPIC','-I',str(FIRMWARE/'src'),*map(str,sources)])
    lib=ctypes.CDLL(str(output));ptr=ctypes.c_void_p
    fptr=np.ctypeslib.ndpointer(dtype=np.float32,flags='C_CONTIGUOUS')
    dptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
    uptr=np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS')
    lib.rider_new.restype=ptr;lib.rider_free.argtypes=[ptr]
    lib.rider_reset.argtypes=[ptr,ctypes.c_float,ctypes.c_float]
    lib.rider_sense.argtypes=[ptr,dptr,dptr,ctypes.c_uint32]
    lib.rider_control.argtypes=[ptr,fptr,uptr,ctypes.c_uint32,fptr]
    lib.rider_feedback.argtypes=[ptr,ctypes.c_uint8,ctypes.c_int,ctypes.c_int,ctypes.c_uint32,fptr]
    _library=lib;return lib


class Firmware:
    def __init__(self):
        self.lib=library();self.pointer=self.lib.rider_new();self.output=np.zeros(18,np.float32)
    def reset(self,pitch_deg,roll_deg):self.lib.rider_reset(self.pointer,pitch_deg,roll_deg)
    def sense(self,accel,gyro,us):self.lib.rider_sense(self.pointer,np.ascontiguousarray(accel,np.float64),np.ascontiguousarray(gyro,np.float64),us)
    def control(self,feedback,command,us):
        self.lib.rider_control(self.pointer,np.ascontiguousarray(feedback,np.float32),np.ascontiguousarray(command,np.uint8),us,self.output)
        return self.output
    def feedback(self,actuator_id,position,speed,us,output):
        self.lib.rider_feedback(self.pointer,actuator_id,position,speed,us,output)
    def close(self):
        if self.pointer:self.lib.rider_free(self.pointer);self.pointer=None
