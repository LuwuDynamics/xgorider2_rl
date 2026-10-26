#!/usr/bin/env python3
"""Portable offline regression suite. Never opens a serial port.  Report: tests/build/host_validation.json."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    compiler=os.environ.get('CXX') or shutil.which('clang++') or shutil.which('c++')
    if not compiler:raise SystemExit('A host C++17 compiler is required')
    build=ROOT/'tests/build/native';build.mkdir(parents=True,exist_ok=True)
    flags=[compiler,'-std=c++17','-O1','-UNDEBUG','-ffp-contract=off','-Wall','-Wextra','-Werror','-I',str(ROOT/'src')]
    cases=[
        ('test_bus',['src/actuator_bus.cpp'],[]),
        ('test_bus_transport',['src/actuator_bus.cpp'],['-DARDUINO','-I',str(ROOT/'tests/stubs')]),
        ('test_board_bus',['src/actuator_bus.cpp','src/board_support.cpp'],[]),
        ('test_runtime_guard',['src/runtime_guard.cpp'],[]),
        ('test_trace',[],[]),
        ('test_lift_guard',[],[]),
        ('test_dead_zone_comp',[],[]),
        ('test_fall_guard',[],[]),
        ('test_led_status',['src/board_support.cpp','src/runtime_guard.cpp'],[]),
        ('test_motion_controller',['src/motion_controller.cpp','src/control/pid.cpp','src/control/rider_motion.cpp','src/control/rider_leg_imu.cpp'],[]),
        ('test_leg_position_controller',['src/leg_position_controller.cpp','src/motion_controller.cpp','src/control/pid.cpp','src/control/rider_motion.cpp','src/control/rider_leg_imu.cpp'],['-fsanitize=address,undefined']),
        ('test_learned_wheel_drive',['src/learned_wheel_drive.cpp','src/policy/policy.cpp'],['-fsanitize=address,undefined']),
        ('test_protocol',['src/command_protocol.cpp'],[]),
        ('test_protocol_fuzz',['src/command_protocol.cpp'],['-fsanitize=address,undefined']),
        ('test_device_settings',['src/device_settings.cpp'],['-I',str(ROOT/'tests/settings_stubs')]),
        ('test_policy',['src/policy/policy.cpp','src/command_protocol.cpp'],['-fsanitize=address,undefined']),
    ]
    results=[]

    def run(name,command):
        result=subprocess.run(command,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        print(result.stdout,end='',flush=True)
        results.append(dict(name=name,passed=result.returncode==0,output=result.stdout))
        if result.returncode:raise RuntimeError(name+' failed')

    for name,sources,extra in cases:
        executable=build/name
        subprocess.run(flags+extra+[str(ROOT/p) for p in sources]+[str(ROOT/'tests'/f'{name}.cpp'),'-o',str(executable)],check=True)
        run(name,[str(executable)])
    for pattern in ('test_policy_export.py','test_rider_cli.py','test_flash_app.py','test_capture_trace_usb.py','test_incident_snapshot.py','test_start_cycle_test.py','test_standing_report.py','test_dzcomp_ab_test.py','test_replay_fall_guard.py'):
        run(pattern,[sys.executable,'-B','-m','unittest','discover','-s','tests','-p',pattern,'-v'])
    source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                   for folder in ('src','tools','tests') for p in (ROOT/folder).rglob('*')
                   if p.is_file() and p.suffix in ('.h','.cpp','.py') and 'build' not in p.parts and '__pycache__' not in p.parts}
    report=dict(generated_at_utc=datetime.now(timezone.utc).isoformat(),passed=True,
                hardware_tested=False,scope='Native policy, generated actor numerical agreement, legacy portable C++ and mocked CLI/flash tests. No physical stability or MCU timing validation.',
                compiler=subprocess.check_output([compiler,'--version'],text=True).splitlines()[0],
                tests=results,source_sha256=source_hashes)
    path=ROOT/'tests/build/host_validation.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(f'Passed {len(results)} test groups; {path}')


if __name__=='__main__':main()
