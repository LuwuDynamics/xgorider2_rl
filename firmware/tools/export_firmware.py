#!/usr/bin/env python3
"""Export the recovered application with motor output enabled. Offline only."""
import argparse
import configparser
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

# Keep the verified ASCII junction path; resolving it defeats the GCC workaround.
ROOT = Path(os.path.abspath(__file__)).parents[1]
ENVIRONMENT = 'rider2-rl-only'
BUILD = ROOT / '.pio' / 'build' / ENVIRONMENT
OUT = ROOT / 'dist'
# SHA-256 of the stock partition table (3072 bytes, generated from partitions.csv); flash_app.py refuses devices whose table differs
STOCK_PARTITION_TABLE_SHA256 = '8001a0fb6a2969561f406531b6ab499eb3b2a357989a5a0a24aa341e24a0489b'
RL_ONLY_FORBIDDEN_SYMBOLS = (
    'rider::firmware::MotionController::',
    'rider::control::updateBalance(', 'rider::control::updateBalanceBias(',
    'rider::control::addWheelSpeedCorrection(', 'rider::control::addHeadingCorrection(',
    'rider::control::addWheelFriction(', 'rider::control::updatePositionHold(',
    'rider::control::speedPid(', 'rider::control::pitchPid(', 'rider::control::headingPid(')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect_image(path):
    data = Path(path).read_bytes()
    if len(data) < 32 or data[0] != 0xE9:
        raise ValueError(f'Not an ESP application image: {path}')
    if not 1 <= data[1] <= 16 or data[23] not in (0, 1):
        raise ValueError('Invalid ESP segment count or hash-appended flag')
    offset, checksum = 24, 0xEF
    segments = []
    for index in range(data[1]):
        if offset + 8 > len(data):
            raise ValueError('Truncated ESP segment header')
        address, length = struct.unpack_from('<II', data, offset)
        start, end = offset + 8, offset + 8 + length
        if end > len(data):
            raise ValueError('Truncated ESP segment data')
        for value in data[start:end]:
            checksum ^= value
        segments.append(dict(index=index, address=hex(address),
                             file_offset=hex(start), bytes=length))
        offset = end
    checksum_end = (offset + 16) // 16 * 16
    if checksum_end > len(data):
        raise ValueError('Missing ESP checksum')
    hash_appended = bool(data[23])
    if hash_appended and checksum_end + 32 > len(data):
        raise ValueError('Truncated appended SHA-256')
    hash_valid = (hashlib.sha256(data[:checksum_end]).digest() ==
                  data[checksum_end:checksum_end + 32]) if hash_appended else None
    descriptor = {}
    first = int(segments[0]['file_offset'], 16)
    if struct.unpack_from('<I', data, first)[0] == 0xABCD5432:
        for name, relative, size in [('version', 16, 32), ('project', 48, 32),
                                     ('time', 80, 16), ('date', 96, 16),
                                     ('idf_version', 112, 32)]:
            descriptor[name] = data[first + relative:first + relative + size].split(
                b'\0')[0].decode('ascii', errors='replace')
    return dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
                chip_id=struct.unpack_from('<H', data, 12)[0],
                entry=hex(struct.unpack_from('<I', data, 4)[0]),
                spi_mode_code=data[2], flash_size_code=data[3] >> 4,
                flash_frequency_code=data[3] & 15,
                spi_mode={0: 'qio', 1: 'qout', 2: 'dio', 3: 'dout'}.get(data[2], 'unknown'),
                flash_size={0: '1MB', 1: '2MB', 2: '4MB', 3: '8MB', 4: '16MB'}.get(data[3] >> 4, 'unknown'),
                flash_frequency={0: '40MHz', 1: '26MHz', 2: '20MHz', 15: '80MHz'}.get(data[3] & 15, 'unknown'),
                checksum_valid=checksum == data[checksum_end - 1],
                appended_sha256_valid=hash_valid, descriptor=descriptor, segments=segments)


def package_version(path):
    if not path.is_file():
        raise ValueError(f'Missing build package metadata: {path}')
    return json.loads(path.read_text(encoding='utf-8'))['version']


def elf_word(data, address):
    """Resolve a little-endian ELF32 load-address literal, never a text guess."""
    if data[:6] != b'\x7fELF\x01\x01':
        raise ValueError('Expected little-endian ELF32')
    phoff = struct.unpack_from('<I', data, 28)[0]
    entry_size, count = struct.unpack_from('<HH', data, 42)
    for index in range(count):
        typ, offset, virtual, _, size = struct.unpack_from('<IIIII', data, phoff + index * entry_size)
        if typ == 1 and virtual <= address and address + 4 <= virtual + size:
            return struct.unpack_from('<I', data, offset + address - virtual)[0]
    raise ValueError('ELF literal is not backed by a load segment: ' + hex(address))


def ota_defer_call(elf, disassembly, symbols):
    symbol = re.search(r'^([0-9a-fA-F]+)\s+[Tt]\s+verifyRollbackLater$', symbols, re.M)
    if not symbol:
        return None
    target = int(symbol.group(1), 16)
    instructions = [(int(m.group(1), 16), m.group(2).strip()) for m in re.finditer(
        r'^\s*([0-9a-fA-F]+):\s+[0-9a-fA-F]+\s+(.+)$', disassembly, re.M)]
    for index, (pc, instruction) in enumerate(instructions[:-1]):
        literal = None
        direct = re.match(r'call8\s+([0-9a-fA-F]+)\s+<verifyRollbackLater>', instruction)
        if direct:
            if int(direct.group(1), 16) != target:
                continue
        else:
            indirect = re.fullmatch(r'callx8\s+(a\d+)', instruction)
            if not indirect or not index:
                continue
            load = re.match(r'l32r\s+' + indirect.group(1) + r',\s*([0-9a-fA-F]+)\b', instructions[index-1][1])
            if not load:
                continue
            literal = int(load.group(1), 16)
            if elf_word(elf, literal) != target:
                continue
        branch_pc, branch = instructions[index+1]
        branch_match = re.match(r'bnez(?:\.n)?\s+a10,\s*([0-9a-fA-F]+)\b', branch)
        if not branch_match:
            continue
        skip_to = int(branch_match.group(1), 16)
        if not any(branch_pc < addr < skip_to and '<esp_ota_mark_app_valid_cancel_rollback>' in op
                   for addr, op in instructions):
            continue
        return dict(call_address=hex(pc), target=hex(target),
                    literal_address=hex(literal) if literal is not None else None,
                    return_true_branch_skips_ota_confirmation_to=hex(skip_to))
    return None


def main():
    global OUT,ENVIRONMENT,BUILD
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=OUT)
    parser.add_argument('--environment',choices=['rider2-firmware','rider2-trained','rider2-rl-only'],default=ENVIRONMENT)
    args=parser.parse_args()
    ENVIRONMENT=args.environment;BUILD=ROOT/'.pio/build'/ENVIRONMENT
    rl_only=ENVIRONMENT=='rider2-rl-only'
    trained=ENVIRONMENT in ('rider2-trained','rider2-rl-only')
    OUT=args.output_dir.absolute()
    OUT.mkdir(parents=True,exist_ok=True)
    required = ['firmware.bin', 'firmware.elf', 'firmware.map', 'partitions.bin']
    for name in required:
        if not (BUILD / name).is_file():
            raise SystemExit('Missing build artifact; run build.sh first: ' + name)
    image = inspect_image(BUILD / 'firmware.bin')
    if not image['checksum_valid'] or image['appended_sha256_valid'] is not True:
        raise SystemExit('Application image checksum / appended SHA-256 failed')
    if (image['chip_id'] != 0 or image['spi_mode'] != 'dio' or
            image['flash_size'] != '4MB' or image['flash_frequency'] != '80MHz'):
        raise SystemExit('Unexpected application target or flash configuration')
    if image['bytes'] > 0x180000:
        raise SystemExit('Application exceeds the original 0x180000-byte slot')
    partition_matches = sha256(BUILD / 'partitions.bin') == STOCK_PARTITION_TABLE_SHA256
    if not partition_matches:
        raise SystemExit('Generated partition table differs from the stock XGO-Rider2 table')

    core = Path(os.environ.get('PLATFORMIO_CORE_DIR', Path.home() / '.platformio'))
    packages = core / 'packages'
    suffix = '.exe' if os.name == 'nt' else ''
    compiler_dir = packages / 'toolchain-xtensa-esp32' / 'bin'
    compiler_version = subprocess.check_output(
        [str(compiler_dir / ('xtensa-esp32-elf-g++' + suffix)), '--version'],
        text=True, encoding='utf-8').splitlines()[0]
    symbols = subprocess.check_output(
        [str(compiler_dir / ('xtensa-esp32-elf-nm' + suffix)), '-C', str(BUILD / 'firmware.elf')],
        text=True, encoding='utf-8', errors='replace')
    required_symbols = ['rider2::BoardImu::begin(', 'rider2::readCalibration(',
                        'rider::bringup::ActuatorBus::poll(',
                        'rider::firmware::LegPositionController::step(' if rl_only else 'rider::firmware::MotionController::step(',
                        'rider::policy::Controller::step(', 'rider::policy::infer(',
                        'rider2::ActionEngine::tick(', 'rider2::beginBle(',
                        'rider2::protocol::StreamParser::feed(',
                        'rider2::saveLegCalibration(', 'rider::board::BoardSupport::poll(',
                        '__wrap_nvs_flash_init', 'verifyRollbackLater']
    linked_modules = {name: name in symbols for name in required_symbols}
    if rl_only:
        linked_modules['rider::firmware::LearnedWheelDrive::step(']='rider::firmware::LearnedWheelDrive::step(' in symbols
    if not all(linked_modules.values()):
        raise SystemExit('Required application module is missing: ' + str(linked_modules))
    wheel_controller=dict(kind='mixed',legacy_wheel_pid_linked=True)
    if rl_only:
        absent={name:name not in symbols for name in RL_ONLY_FORBIDDEN_SYMBOLS}
        marker=re.search(r'^([0-9a-fA-F]+)\s+[A-Za-z]\s+rider_rl_only_build$',symbols,re.M)
        if not all(absent.values()) or not marker:
            raise SystemExit('RL-only link check failed: '+str(absent))
        if elf_word((BUILD/'firmware.elf').read_bytes(),int(marker.group(1),16))!=0x524c4f31:
            raise SystemExit('Invalid linked RL-only build marker')
        wheel_controller=dict(kind='rl_only',legacy_wheel_pid_linked=False,
            no_fallback=True,legs='position_control',actor_leg_offsets=True,
            explicit_rearm_selects='active',
            forbidden_symbols_absent=absent,linked_build_marker_verified=True)
    if not any(line.split()[-1:] == ['nvs_flash_init'] for line in symbols.splitlines()):
        raise SystemExit('Full BLE firmware requires the real NVS initializer')
    startup_disassembly = {}
    for symbol in ['initArduino', '__wrap_nvs_flash_init', 'verifyRollbackLater']:
        startup_disassembly[symbol] = subprocess.check_output(
            [str(compiler_dir / ('xtensa-esp32-elf-objdump' + suffix)), '-d',
             '--disassemble=' + symbol, 'firmware.elf'],
            cwd=str(BUILD), text=True, encoding='utf-8', errors='replace')
    startup = startup_disassembly['initArduino']
    nvs_guard = startup_disassembly['__wrap_nvs_flash_init']
    ota_guard = startup_disassembly['verifyRollbackLater']
    ota_call = ota_defer_call((BUILD / 'firmware.elf').read_bytes(), startup, symbols)
    guards = dict(
        init_calls_nvs_wrapper='<__wrap_nvs_flash_init>' in startup,
        init_checks_ota_defer_override=ota_call is not None,
        nvs_wrapper_calls_real_initializer='<nvs_flash_init>' in nvs_guard,
        ota_override_returns_true=bool(re.search(r'\bmovi(?:\.n)?\s+a2,\s*1\b', ota_guard)) and
            not re.search(r'\bcall(?:x)?(?:0|4|8|12)\b', ota_guard),
        real_nvs_initializer_present=True)
    if not all(guards.values()):
        raise SystemExit('Startup NVS/OTA preservation disassembly check failed: ' + str(guards))
    sdkconfig = packages / 'framework-arduinoespressif32' / 'tools' / 'sdk' / 'esp32' / 'dio_qspi' / 'include' / 'sdkconfig.h'
    sdktext = sdkconfig.read_text(encoding='utf-8')
    preservation = dict(checks=guards, resolved_ota_defer_call=ota_call, sdkconfig_sha256=sha256(sdkconfig),
                        sdk_bootloader_app_rollback_enabled=bool(re.search(
                            r'^#define CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE 1$', sdktext, re.M)),
                        note='OTA automatic confirmation is deferred. NVS initializes normally for BLE; its two auto-format-triggering errors are mapped to ESP_FAIL, preventing Arduino automatic full-NVS erase. Normal SDK NVS writes are possible.')
    preservation_output=OUT
    (preservation_output / 'startup_preservation.json').write_text(
        json.dumps(dict(**preservation, firmware_elf_sha256=sha256(BUILD / 'firmware.elf'),
                        disassembly=startup_disassembly), indent=2) + '\n', encoding='utf-8')
    config = configparser.ConfigParser()
    config.read(ROOT / 'platformio.ini', encoding='utf-8')
    parameters = dict(config['env:rider2-firmware'])
    parameters.update(dict(config['env:' + ENVIRONMENT]))
    parameters['build_flags']=parameters['build_flags'].replace('${env:rider2-firmware.build_flags}',config['env:rider2-firmware']['build_flags'])
    versions = dict(
        platformio=subprocess.check_output([sys.executable, '-m', 'platformio', '--version'],
                                           text=True, encoding='utf-8').strip(),
        espressif32=package_version(core / 'platforms' / 'espressif32' / 'platform.json'),
        framework_arduinoespressif32=package_version(packages / 'framework-arduinoespressif32' / 'package.json'),
        toolchain_xtensa_esp32=package_version(packages / 'toolchain-xtensa-esp32' / 'package.json'),
        compiler=compiler_version,
        tool_esptoolpy=package_version(packages / 'tool-esptoolpy' / 'package.json'),
        embedded_idf=image['descriptor'].get('idf_version'))
    sources = [ROOT / name for name in ['platformio.ini', 'partitions.csv', 'requirements.txt', 'README.md', 'build.sh'] if (ROOT / name).is_file()]
    for subdir in ['src', 'include', 'tools']:
        sources.extend(p for p in (ROOT / subdir).rglob('*')
                       if p.is_file() and '__pycache__' not in p.parts)
    OUT.mkdir(exist_ok=True)
    artifacts = []
    for name in ['firmware.bin', 'firmware.elf', 'firmware.map']:
        shutil.copy2(BUILD / name, OUT / name)
        artifacts.append(dict(file=name, bytes=(OUT / name).stat().st_size,
                              sha256=sha256(OUT / name)))
    policy_path=ROOT/('src/policy/trained_weights.h' if trained else 'src/policy/policy_weights.h')
    policy_header=policy_path.read_text(encoding='utf-8')
    policy_match=re.search(r'^// policy_export: (.+)$',policy_header,re.M)
    if not policy_match:raise SystemExit('Native policy model metadata is missing')
    policy_info=json.loads(policy_match.group(1))
    from export_policy import CONTRACT
    if policy_info.get('contract')!=CONTRACT:raise SystemExit('Weights use an obsolete observation contract')
    if trained and policy_info.get('trained') is not True:
        raise SystemExit('The trained build requires exported trained weights')
    if rl_only and policy_info.get('action_space')!='wheels_legs':
        raise SystemExit('RL-only requires the trained wheel+leg actor')
    # Verify model metadata from the actual linked target structure, not just a header comment.
    model_symbol=re.search(r'^([0-9a-fA-F]+)\s+[A-Za-z]\s+rider::policy::weights::compiledModel\(\)::model$',symbols,re.M)
    if not model_symbol:raise SystemExit('Linked native model structure is missing')
    model_address=int(model_symbol.group(1),16);elf=(BUILD/'firmware.elf').read_bytes()
    # 32-bit layout: name0,id4,contract8,trained12,action_space13,action_scale16,
    # leg_action_scale20,observation_clip24,pointers28..,yaw gain60,limit64.
    flags=elf_word(elf,model_address+12)
    action_space_value={'residual':0,'wheels':1,'wheels_legs':2}[policy_info['action_space']]
    if (elf_word(elf,model_address+4)!=policy_info['model_id'] or
        elf_word(elf,model_address+8)!=policy_info['contract'] or
        bool(flags&255)!=policy_info['trained'] or
        ((flags>>8)&255)!=action_space_value):
        raise SystemExit('Linked native model metadata differs from the selected weights header')
    for offset,key,default in [(16,'action_scale',0.),(20,'leg_action_scale',.005),
                               (60,'yaw_integral_gain',0.),(64,'yaw_integral_limit',.05)]:
        expected=struct.unpack('<I',struct.pack('<f',policy_info.get(key,default)))[0]
        if elf_word(elf,model_address+offset)!=expected:raise SystemExit('Linked model profile differs from the header: '+key)
    policy_info.update(mode_on_boot='active' if rl_only else 'off',on_device_profiled=False,weights_header_sha256=sha256(policy_path),
        weights_header=str(policy_path.relative_to(ROOT)),linked_model_address=hex(model_address),linked_metadata_verified=True)
    manifest = dict(
        project='Rider2 application', environment=ENVIRONMENT,
        exported_at_utc=datetime.now(timezone.utc).isoformat(),
        telemetry_only=False, physical_motion_enabled=True,
        recovered_application_features=['balance','forward_reverse','turn','height','roll',
            'position_hold','heading_hold','leveling','periodic_motion','six_actions','action_loop',
            'usb_protocol','aux_uart_protocol','ble_protocol','telemetry','rgb_leds',
            'battery_protection','leg_calibration','pitch_trim','settings','native_policy_inference'],
        native_policy=policy_info,
        wheel_controller=wheel_controller,
        artifacts=artifacts, image=image, build_versions=versions, linked_modules=linked_modules,
        startup_preservation=preservation,
        build_parameters=parameters,
        source_sha256={str(p.relative_to(ROOT)).replace('\\', '/'): sha256(p)
                       for p in sorted(sources)},
        partition_table_matches_supplied_file=partition_matches,
        original_partition_table_sha256=STOCK_PARTITION_TABLE_SHA256,
        candidate_app_slots=[dict(label='app0', offset='0x10000', size='0x180000'),
                             dict(label='app1', offset='0x190000', size='0x180000')],
        preserve_unpartitioned_calibration_gap=dict(start='0x310000', end_exclusive='0x330000'),
        live_device_active_ota_slot='unknown; uploader inspects device partition table and otadata first',
        flashing_policy='App-only after confirming the actual device partition table and active OTA slot. Do not erase chip or overwrite NVS, otadata, calibration gap [0x310000,0x330000), SPIFFS, bootloader, or partition table.',
        backup_reads_required=False)
    if rl_only:
        manifest['project']='Rider2 RL-only wheels with leg position control'
        manifest['recovered_application_features']=[name for name in manifest['recovered_application_features']
            if name not in ('balance','position_hold','heading_hold','pitch_trim')]+[
                'rl_only_wheel_balance','leg_position_control','bounded_phone_references','latched_fault_stop']
    (OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'Exported MOTOR-ENABLED application: {image["bytes"]} bytes; image integrity and original partition layout verified.')
    print(f'No hardware was accessed. Export directory: {OUT}')


if __name__ == '__main__':
    main()
