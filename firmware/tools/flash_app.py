"""Inspect only by default. --flash replaces the OTA-selected app, never metadata.

Selection/CRC reference: Espressif ESP-IDF v4.4.2 bootloader_common_loader.c.
This does not read a full backup. It reads only partition/OTA metadata (11 KiB).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import zlib
from export_firmware import inspect_image, RL_ONLY_FORBIDDEN_SYMBOLS

ROOT = Path(os.path.abspath(__file__)).parents[1]
SLOTS = (0x10000, 0x190000)
SLOT_SIZE = 0x180000


def select_slot(ota):
    if len(ota) != 0x2000:
        raise ValueError('OTA metadata must be exactly 8192 bytes')
    candidates = []
    for index, off in enumerate((0, 0x1000)):
        seq, state, crc = struct.unpack_from('<I20xII', ota, off)
        if seq == 0xffffffff or state in (3, 4):
            continue
        # seq=0 with matching CRC is present in the stock boot_app0 second
        # sector. It is a valid older record, but never a flash target by itself.
        if zlib.crc32(ota[off:off+4], 0xffffffff) != crc:
            continue
        if state not in (2, 0xffffffff):
            raise ValueError('OTA pending/first-boot/unknown state; refusing to write')
        candidates.append((seq, -index, state))
    if not candidates:
        raise ValueError('No CRC-valid OTA selection; refusing to guess an app slot')
    seq, neg_index, state = max(candidates)
    if seq == 0:
        raise ValueError('Only sequence zero is valid; refusing to guess an app slot')
    slot = (seq-1) % 2
    return dict(slot=slot, offset=SLOTS[slot], size=SLOT_SIZE,
                sequence=seq, state=state, record=-neg_index)


def check_partition_table(data, expected_hash):
    if len(data) != 0xc00 or hashlib.sha256(data).hexdigest() != expected_hash:
        raise ValueError('Device partition table differs from the stock Rider table; no write')
    found = {}
    for off in range(0, len(data), 32):
        magic, typ, subtype, start, size = struct.unpack_from('<HBBII', data, off)
        if magic != 0x50aa:
            break
        if typ == 0:
            found[subtype] = (start, size)
        if typ == 1 and subtype == 0 and (start, size) != (0xe000, 0x2000):
            raise ValueError('Unexpected OTA metadata location')
    if found != {0x10: (SLOTS[0], SLOT_SIZE), 0x11: (SLOTS[1], SLOT_SIZE)}:
        raise ValueError('Unexpected app layout')


def check_image(image, manifest):
    info = inspect_image(image)
    artifact = next(a for a in manifest['artifacts'] if a['file'] == 'firmware.bin')
    if (info['sha256'] != artifact['sha256'] or info['bytes'] != artifact['bytes'] or
            not info['checksum_valid'] or info['appended_sha256_valid'] is not True):
        raise ValueError('Firmware hash/image checksum mismatch; rebuild/export first')
    if (info['bytes'] > SLOT_SIZE or info['chip_id'] != 0 or info['spi_mode'] != 'dio' or
            info['flash_size'] != '4MB' or info['flash_frequency'] != '80MHz' or
            manifest.get('telemetry_only') is not False or manifest.get('physical_motion_enabled') is not True or
            manifest.get('environment') not in ('rider2-firmware','rider2-trained','rider2-rl-only')):
        raise ValueError('Unexpected firmware type, size, or target')
    if manifest.get('environment') in ('rider2-trained','rider2-rl-only'):
        model=manifest.get('native_policy',{})
        boot_mode='active' if manifest['environment']=='rider2-rl-only' else 'off'
        if (model.get('trained') is not True or model.get('contract')!=6 or
                model.get('mode_on_boot')!=boot_mode or model.get('linked_metadata_verified') is not True):
            raise ValueError('Trained firmware requires a verified native model and the matching boot mode')
    if manifest.get('environment')=='rider2-rl-only':
        controller=manifest.get('wheel_controller',{})
        absent=controller.get('forbidden_symbols_absent',{})
        if (model.get('action_space')!='wheels_legs' or controller.get('kind')!='rl_only' or
                controller.get('legacy_wheel_pid_linked') is not False or
                controller.get('no_fallback') is not True or controller.get('legs')!='position_control' or
                controller.get('actor_leg_offsets') is not True or
                controller.get('explicit_rearm_selects')!='active' or
                controller.get('linked_build_marker_verified') is not True or
                set(absent)!=set(RL_ONLY_FORBIDDEN_SYMBOLS) or any(value is not True for value in absent.values())):
            raise ValueError('RL-only firmware requires the wheel+leg actor and verified removal of legacy wheel control')
    return info


def flash_arguments(selection, image, length):
    start = selection['offset']
    erase_end = start + ((length + 4095) // 4096) * 4096
    if start not in SLOTS or length <= 0 or length > SLOT_SIZE or erase_end > start+SLOT_SIZE or erase_end > 0x310000:
        raise ValueError('Application write/erase range crosses an allowed slot boundary')
    # keep preserves the checked image header. No chip-wide erase or force option.
    return ['write_flash', '--flash_mode', 'keep', '--flash_freq', 'keep',
            '--flash_size', 'keep', hex(start), str(image)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port')
    p.add_argument('--flash', action='store_true', help='Actually replace the OTA-selected application')
    p.add_argument('--plan', action='store_true', help='Offline artifact check; never open a serial port')
    p.add_argument('--esptool', type=Path, required=True)
    p.add_argument('--artifact-dir',type=Path,help='Artifact directory; defaults to dist/')
    args = p.parse_args()
    artifact_dir=args.artifact_dir if args.artifact_dir is not None else ROOT/'dist'
    image = artifact_dir / 'firmware.bin'
    manifest = json.loads((artifact_dir / 'manifest.json').read_text(encoding='utf-8'))
    info = check_image(image, manifest)
    print(f'Validated MOTOR-ENABLED firmware: {info["bytes"]} bytes, SHA256 {info["sha256"]}')
    print('This application can start balancing after sensor calibration. Support the robot before installation.')
    if args.plan:
        if args.flash:
            p.error('--plan cannot be combined with --flash')
        print('OFFLINE ONLY. Device partition table and selected OTA slot are still unknown.')
        print('Possible app slots: 0x10000 / 0x190000. Calibration gap 0x310000..0x32ffff is preserved.')
        return
    if not args.port:
        p.error('--port is required for device inspection or flashing')
    if not args.esptool.is_file():
        p.error('esptool.py was not found')
    common = [sys.executable, str(args.esptool), '--chip', 'esp32', '--port', args.port,
              '--baud', '115200', '--before', 'default_reset']

    def run(command, after='no_reset'):
        subprocess.run(common + ['--after', after] + command, check=True)

    # Reads are intentionally limited to metadata.  Make your own full backup of the stock firmware first (README.md).
    # Keep chip in download mode between calls, then reset after final operation.
    with tempfile.TemporaryDirectory(prefix='rider2_metadata_') as directory:
        table_file = Path(directory) / 'partitions.bin'
        ota_file = Path(directory) / 'otadata.bin'
        run(['read_flash', '0x8000', '0xc00', str(table_file)])
        check_partition_table(table_file.read_bytes(), manifest['original_partition_table_sha256'])
        run(['read_flash', '0xe000', '0x2000', str(ota_file)])
        selection = select_slot(ota_file.read_bytes())
        print('OTA-selected target: ' + json.dumps(selection))
        if args.flash:
            run(flash_arguments(selection, image, info['bytes']))
            run(['verify_flash', hex(selection['offset']), str(image)], after='hard_reset')
            print('Application written and verified. Open this port at 115200 baud.')
        else:
            run(['flash_id'], after='hard_reset')
            print('Inspection finished; NO flash write. Add --flash to install this application.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError, StopIteration, subprocess.CalledProcessError) as exc:
        print('STOP: ' + str(exc), file=sys.stderr)
        print('No further operation will run. If the device remains in download mode, power-cycle it.', file=sys.stderr)
        sys.exit(1)
