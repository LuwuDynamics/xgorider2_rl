"""Read-only BLE protocol round trips, followed by neutral control frames.

No arm, policy activation, settings write or nonzero movement is sent.
"""
import argparse
import asyncio
import json
from pathlib import Path
import struct
import time
from bleak import BleakClient,BleakScanner

SERVICE='0000fff0-0000-1000-8000-00805f9b34fb'
NOTIFY='0000fff1-0000-1000-8000-00805f9b34fb'
WRITE='0000fff2-0000-1000-8000-00805f9b34fb'


def frame(kind,reg,payload):
    body=bytes([len(payload)+8,kind,reg])+payload
    return b'\x55\x00'+body+bytes([(~sum(body))&255,0,0xaa])


async def run(args):
    found=await BleakScanner.discover(timeout=10,return_adv=True,service_uuids=[SERVICE])
    candidates=[(device,adv) for device,adv in found.values()
        if (adv.local_name or device.name or '').startswith('XGORider_')
        and (not args.name or args.name in (adv.local_name,device.name))]
    if len(candidates)!=1:
        raise RuntimeError('Expected one Rider advertisement; found '+str([(d.address,a.local_name or d.name) for d,a in candidates]))
    device,adv=candidates[0];queue=asyncio.Queue();buffer=bytearray();checksum_errors=0;replies=[]
    def notified(_,data):
        nonlocal checksum_errors
        buffer.extend(data)
        while len(buffer)>=3:
            if buffer[:2]!=b'\x55\x00':del buffer[0];continue
            n=buffer[2]
            if not 8<=n<=64:del buffer[0];continue
            if len(buffer)<n:return
            packet=bytes(buffer[:n]);del buffer[:n]
            if packet[-2:]!=b'\x00\xaa' or packet[-3]!=((~sum(packet[2:-3]))&255):
                checksum_errors+=1;continue
            queue.put_nowait(packet)
    began=time.monotonic()
    async with BleakClient(device,timeout=15) as client:
        services=[dict(uuid=s.uuid,characteristics=[dict(uuid=c.uuid,properties=c.properties) for c in s.characteristics]) for s in client.services if s.uuid==SERVICE]
        await client.start_notify(NOTIFY,notified)
        reads=((0xb1,8),(0xb0,1),(7,10),(1,1),(0x65,24),(2,1))
        if args.rl_only:reads+=((0xb2,16),)
        for index,(reg,count) in enumerate(reads*args.rounds):
            request=frame(2,reg,bytes([count]));sent=time.monotonic()
            await client.write_gatt_char(WRITE,request,response=True)
            packet=await asyncio.wait_for(queue.get(),timeout=4)
            if packet[3]!=0x12 or packet[4]!=reg or len(packet)!=count+8:raise RuntimeError('Unexpected read response: '+packet.hex())
            payload=packet[5:-3];row=dict(round=index//len(reads),register=hex(reg),request_hex=request.hex(),reply_hex=packet.hex(),latency_ms=round((time.monotonic()-sent)*1000,3))
            if reg==0xb1:
                row.update(model_id=int.from_bytes(payload[:4],'little'),action_space=payload[4],trained=bool(payload[5]),model_valid=bool(payload[6]),contract=payload[7])
                if row['model_id']!=args.model_id or not row['trained'] or not row['model_valid'] or row['contract']!=6:
                    raise RuntimeError('BLE model identity/validity mismatch: '+str(row))
            elif reg==0xb0:row['policy_mode']=payload[0]
            elif reg==0xb2:
                row.update(controller_kind=payload[0],wheel_source=payload[1],policy_mode=payload[2],last_stop_reason=payload[3],
                    policy_calls=int.from_bytes(payload[4:8],'little'),policy_failures=int.from_bytes(payload[8:12],'little'),
                    rl_output_ticks=int.from_bytes(payload[12:16],'little'))
                if row['controller_kind']!=1 or row['wheel_source'] not in (0,2):
                    raise RuntimeError('Firmware is not exclusively RL wheel control: '+str(row))
            elif reg==7:row['version']=payload.rstrip(b'\x00').decode(errors='replace')
            elif reg==1:row['battery_percent']=payload[0]
            elif reg==2:
                row['robot_mode']=payload[0]
                if args.expect_stopped and row['robot_mode']!=254:raise RuntimeError('Manual stopped mode was not retained: '+str(row))
            elif reg==0x65:row['acceleration_and_roll_pitch_yaw']=list(struct.unpack('<6f',payload))
            replies.append(row)
            await asyncio.sleep(.05)
        neutral=[]
        for reg in (() if args.read_only else (0x30,0x32)):
            request=frame(0,reg,b'\x80');await client.write_gatt_char(WRITE,request,response=True);neutral.append(request.hex())
        await asyncio.sleep(args.hold_seconds)
        await client.stop_notify(NOTIFY)
    result=dict(passed=True,name=adv.local_name or device.name,device_uuid=device.address,rssi=adv.rssi,
        services=services,replies=replies,checksum_errors=checksum_errors,neutral_frames_written=neutral,
        seconds=time.monotonic()-began,round_trips=len(replies),latency_max_ms=max(row['latency_ms'] for row in replies),motors_armed=False,nonzero_motion_sent=False,
        scope='Mac BLE GATT and original frame protocol; not original phone-app or floor-balance acceptance')
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--name');parser.add_argument('--model-id',type=int,required=True)
    parser.add_argument('--rounds',type=int,choices=range(1,101),default=1)
    parser.add_argument('--hold-seconds',type=float,default=0)
    parser.add_argument('--expect-stopped',action='store_true')
    parser.add_argument('--rl-only',action='store_true',help='Verify RL-only build and actual wheel source via register 0xb2')
    parser.add_argument('--read-only',action='store_true',help='Only send protocol read requests; omit neutral control writes')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if not 0<=args.hold_seconds<=60:parser.error('Hold duration must be 0..60 seconds')
    asyncio.run(run(args))
