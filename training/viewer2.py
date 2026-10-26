"""Interactive Rider controls on the calibrated contract-6 simulator. Run with the normal Python main thread on macOS."""
import argparse
import json
from pathlib import Path
from queue import SimpleQueue
import time
import glfw
from rider_sim.env2 import RiderEnv2
from rider_sim.native_policy import NativePolicy
from rider_sim.window import RiderWindow


class KeyboardDrive:
    """Latched reference commands; callbacks only queue events for the sim thread."""
    def __init__(self,automatic=False):
        self.automatic=automatic;self.forward=0.;self.turn=0.
        self.events=SimpleQueue()

    def stop(self):self.forward=self.turn=0.

    def update(self):
        reset=False
        while not self.events.empty():
            code=self.events.get()
            if code==glfw.KEY_R:self.stop();reset=True
            elif code==glfw.KEY_T:self.automatic=not self.automatic;self.stop()
            elif code==glfw.KEY_SPACE:self.automatic=False;self.stop()
            elif code in (glfw.KEY_W,glfw.KEY_UP):self.automatic=False;self.forward=.1
            elif code in (glfw.KEY_S,glfw.KEY_DOWN):self.automatic=False;self.forward=-.1
            elif code in (glfw.KEY_A,glfw.KEY_LEFT):self.automatic=False;self.turn=.4
            elif code in (glfw.KEY_D,glfw.KEY_RIGHT):self.automatic=False;self.turn=-.4
            elif code==glfw.KEY_X:self.automatic=False;self.turn=0.
        return reset

    def apply(self,env):
        env.commands=self.automatic
        if not self.automatic:
            env.command[0]=round(127.5+self.forward/.21*127.5)
            env.command[1]=round(127.5+self.turn/(300*3.141592653589793/180)*127.5)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--model',type=Path,default=Path('artifacts/balance-v6/policy_weights.h'))
    parser.add_argument('--domain',type=float,default=0.);parser.add_argument('--seed',type=int,default=40007)
    parser.add_argument('--duration',type=float,help='Close after this many wall seconds, for a GUI smoke check')
    parser.add_argument('--auto',action='store_true',help='Start with random command demonstration instead of keyboard control')
    parser.add_argument('--pushes',action='store_true',help='Enable random external disturbances')
    args=parser.parse_args();native=NativePolicy(args.model)
    drive=KeyboardDrive(args.auto)
    env=RiderEnv2(domain=args.domain,commands=args.auto,pushes=args.pushes,episode_seconds=3600,render_mode='rgb_array',native_policy=native,noise=1.0)
    env.reset(seed=args.seed)
    print('W/Up forward; S/Down reverse; A/Left left; D/Right right; X straight; Space stop; R reset; T auto. Commands stay active until changed.',flush=True)
    try:
        with RiderWindow(env.model,env.data,drive.events.put) as viewer:
            opened=time.monotonic();next_step=opened;info=None
            while viewer.is_running():
                now=time.monotonic()
                if args.duration is not None and now-opened>=args.duration:break
                viewer.poll()
                if drive.update():
                    env.reset(seed=args.seed);viewer.camera_offset[:]=0;next_step=now
                # Keep physical control intervals fixed while rendering at the
                # display rate. Bound catch-up after moving/minimizing a window.
                if now-next_step>.2:next_step=now
                while now>=next_step:
                    drive.apply(env)
                    _,_,done,trunc,info=env.step(native.action.copy())
                    next_step+=env.policy_every*env.control_ticks*.002
                    if done or trunc:
                        print('Episode',info,flush=True);drive.stop();env.reset()
                if info is not None:
                    mode='AUTO: T to switch' if drive.automatic else 'KEYBOARD: latched commands'
                    viewer.render(env.data.xpos[env.base],
                        'Rider2 | '+mode+'\nW / S | Up / Down\nA / D | Left / Right\nX | Space\nR | T\nMouse drag / wheel | Esc\nTarget v / yaw\nActual v / yaw',
                        '\nForward / reverse\nLeft / right\nStraight / stop\nReset / auto\nCamera / zoom | Close\n'
                        f'{info["reference"]:+.2f} m/s / {info["yaw_reference"]:+.2f} rad/s\n'
                        f'{info["velocity"]:+.2f} m/s / {info["yaw_rate"]:+.2f} rad/s')
                delay=min(.002,next_step-time.monotonic())
                if delay>0:time.sleep(delay)
    finally:
        env.close();native.close()


if __name__=='__main__':main()
