"""USB console for recovered Rider2 firmware; attaching does not reset the board (RTS is released before DTR)."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import time

def percent_byte(value):
    if not -100 <= value <= 100:
        raise ValueError('Movement percentage must be -100..100')
    return max(0, min(255, round(128 + value * (127 if value >= 0 else 128) / 100)))

class Session:
    def __init__(self, port):
        import serial
        self.port=serial.Serial(port=None,baudrate=115200,timeout=.2)
        # The auto-reset circuit pulses EN when RTS is asserted while DTR is released.  The OS asserts both lines on
        # open, so release RTS first and DTR only after the port is open: (1,1) -> (1,0) -> (0,0) never resets the board.
        self.port.rts=False;self.port.port=port
        self.port.open()
        self.port.dtr=False
    def send(self,text):
        if '\r' in text or '\n' in text or not text.startswith('!'):
            raise ValueError('One explicit console command is required')
        self.port.write((text+'\n').encode('utf-8'));self.port.flush()
    def status(self,timeout=3):
        self.send('!status');deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            line=self.port.readline().decode('utf-8',errors='replace').strip()
            try:data=json.loads(line)
            except ValueError:continue
            if isinstance(data,dict) and data.get('firmware')=='Rider2-recovered-full':return data
        raise RuntimeError('No recovered-full firmware response. Check the USB download port and installed image.')
    def close(self):self.port.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    sub=p.add_subparsers(dest='command',required=True)
    for name in ['status','arm','stop','neutral','trim']:sub.add_parser(name)
    policy=sub.add_parser('policy');policy.add_argument('mode',choices=['status','off','shadow','active'])
    for name in ['height','roll','action','loop','level','heading','heightwave','rollwave','maxforward','maxturn']:
        child=sub.add_parser(name);child.add_argument('value',type=int,choices=range(256),metavar='0..255')
    move=sub.add_parser('move');move.add_argument('--forward',type=float,default=0);move.add_argument('--turn',type=float,default=0)
    move.add_argument('--seconds',type=float,default=1)
    cal=sub.add_parser('cal');cal.add_argument('operation',choices=['begin','save'])
    name=sub.add_parser('name');name.add_argument('suffix')
    led=sub.add_parser('led');led.add_argument('index',type=int,choices=range(4));led.add_argument('rgb',type=int,nargs=3)
    log=sub.add_parser('log');log.add_argument('--seconds',type=int,default=30)
    args=p.parse_args()
    if args.command=='move':
        if not 0<args.seconds<=30:p.error('Move duration must be >0 and <=30 seconds')
        try:forward,turn=percent_byte(args.forward),percent_byte(args.turn)
        except ValueError as exc:p.error(str(exc))
    if args.command=='name':
        encoded=args.suffix.encode('utf-8')
        if not 1<=len(encoded)<=20 or any(v<32 or v==127 for v in encoded):p.error('Name must contain 1..20 printable UTF-8 bytes')
    if args.command=='led' and any(not 0<=v<=255 for v in args.rgb):p.error('RGB channels must be 0..255')
    if args.command=='log' and not 1<=args.seconds<=3600:p.error('Log duration must be 1..3600 seconds')
    session=Session(args.port)
    neutral_on_exit=False
    try:
        # Emergency stop is sent before the handshake so a lost status response
        # does not delay the explicit stop request.
        if args.command=='stop':
            session.send('!stop');time.sleep(.05)
        state=session.status()
        if args.command=='stop':
            deadline=time.monotonic()+2
            while not (state.get('manual_stop') is True and state.get('driving') is False):
                if time.monotonic()>=deadline:
                    raise RuntimeError('Stop was sent but the stopped state was not confirmed; support the robot and use its power switch')
                time.sleep(.05);state=session.status()
        if args.command in ['status','stop']:
            print(json.dumps(state,ensure_ascii=False,indent=2));return
        if args.command=='policy':
            if not isinstance(state.get('policy'),dict):raise RuntimeError('Installed firmware has no native policy interface')
            if args.mode=='status':print(json.dumps(state['policy'],ensure_ascii=False,indent=2));return
            if state.get('manual_stop') is not True or state.get('driving') is not False:
                raise RuntimeError('Stop explicitly before selecting policy mode')
            if args.mode=='active' and not (state.get('policy',{}).get('trained') is True and state.get('policy',{}).get('model_valid') is True):
                raise RuntimeError('Active policy requires an installed, validated trained model')
        if args.command=='move':
            if not state.get('driving'):raise RuntimeError('Robot is not balancing; inspect status and arm first')
            neutral_on_exit=True;session.send(f'!move {forward} {turn}')
            deadline=time.monotonic()+args.seconds
            while time.monotonic()<deadline:time.sleep(min(.1,max(0,deadline-time.monotonic())))
            session.send('!neutral');neutral_on_exit=False
        elif args.command=='log':
            root=Path(os.path.abspath(__file__)).parents[1];folder=root/'logs';folder.mkdir(exist_ok=True)
            path=folder/(datetime.now().strftime('%Y%m%d-%H%M%S')+'.jsonl')
            deadline=time.monotonic()+args.seconds
            with path.open('w',encoding='utf-8') as output:
                while time.monotonic()<deadline:
                    state=session.status();text=json.dumps(state,ensure_ascii=False);print(text);output.write(text+'\n');output.flush();time.sleep(.5)
            print('Saved '+str(path));return
        elif args.command=='policy':session.send('!policy '+args.mode)
        elif args.command=='cal':session.send('!cal '+args.operation)
        elif args.command=='name':session.send('!name '+args.suffix)
        elif args.command=='led':session.send('!rgb '+str(args.index)+' '+' '.join(map(str,args.rgb)))
        elif hasattr(args,'value'):session.send('!'+args.command+' '+str(args.value))
        else:session.send('!'+args.command)
        time.sleep(.15);state=session.status()
        if args.command=='policy' and state.get('policy',{}).get('mode')!=args.mode:
            raise RuntimeError('Policy mode was not accepted: '+str(state.get('result','unknown reason')))
        print(json.dumps(state,ensure_ascii=False,indent=2))
    finally:
        try:
            if neutral_on_exit:session.send('!neutral')
        finally:session.close()

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('Interrupted; neutral requested for a timed move.')
    except (ValueError,RuntimeError,OSError) as exc:raise SystemExit(str(exc))
