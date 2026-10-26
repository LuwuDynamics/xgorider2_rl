"""Offline uploader checks. Every esptool invocation is replaced by a fake.

The synthetic ESP image tests corruption independently of the export manifest;
the exported production image is also checked when it is available.
"""
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import zlib

ROOT = Path(os.path.abspath(__file__)).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import flash_app


def metadata(records):
    data = bytearray(b'\xff' * 0x2000)
    for index, (sequence, state) in enumerate(records):
        checksum = zlib.crc32(struct.pack('<I', sequence), 0xffffffff)
        struct.pack_into('<I20xII', data, index * 0x1000, sequence, state, checksum)
    return data


def synthetic_image(*, chip=0, mode=2, size_frequency=0x2f, appended_hash=True):
    """One segment with a real ESP checksum/hash, never intended for a device."""
    header = bytearray(24)
    header[:4] = bytes((0xe9, 1, mode, size_frequency))
    struct.pack_into('<I', header, 4, 0x40080000)
    struct.pack_into('<H', header, 12, chip)
    header[23] = int(appended_hash)
    payload = bytes(range(16))
    data = header + struct.pack('<II', 0x3ffb0000, len(payload)) + payload
    checksum = 0xef
    for value in payload:
        checksum ^= value
    checksum_end = (len(data) + 16) // 16 * 16
    data.extend(b'\0' * (checksum_end - len(data)))
    data[-1] = checksum
    if appended_hash:
        data.extend(hashlib.sha256(data).digest())
    return bytes(data)


def image_manifest(data, table):
    return dict(environment='rider2-firmware', telemetry_only=False,
                physical_motion_enabled=True,
                original_partition_table_sha256=hashlib.sha256(table).hexdigest(),
                artifacts=[dict(file='firmware.bin', bytes=len(data),
                                sha256=hashlib.sha256(data).hexdigest())])


class UploaderChecks(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='rider2_uploader_test_')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        (self.root / 'dist').mkdir()
        self.table = (ROOT / 'partitions.bin').read_bytes()
        self.image = self.root / 'dist' / 'firmware.bin'
        self.original = synthetic_image()
        self.manifest = image_manifest(self.original, self.table)
        self.write_fixture(self.original, self.manifest)
        self.esptool = self.root / 'fake_esptool.py'
        self.esptool.write_text('# Never executed.\n', encoding='utf-8')

    def write_fixture(self, data, manifest=None):
        self.image.write_bytes(data)
        manifest = manifest if manifest is not None else image_manifest(data, self.table)
        (self.root / 'dist' / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
        return manifest

    def invoke(self, arguments, *, ota=None, table=None, fail_operation=None):
        """Mock subprocess.run before calling main, including --flash paths."""
        commands = []
        ota = metadata([(1, 2), (2, 2)]) if ota is None else ota
        table = self.table if table is None else table

        def fake_run(argv, check):
            self.assertTrue(check)
            marker = argv.index('--after')
            after, operation = argv[marker + 1], argv[marker + 2:]
            commands.append((after, operation))
            if operation[0] == fail_operation:
                raise subprocess.CalledProcessError(2, argv)
            if operation[0] == 'read_flash':
                expected = {'0x8000': ('0xc00', table), '0xe000': ('0x2000', ota)}
                size, data = expected[operation[1]]
                self.assertEqual(operation[2], size)
                Path(operation[3]).write_bytes(data)
            elif operation[0] not in ('write_flash', 'verify_flash', 'flash_id'):
                self.fail('Unexpected external operation: ' + repr(operation))

        self.commands = commands
        with mock.patch.object(flash_app, 'ROOT', self.root), \
                mock.patch.object(sys, 'argv', ['flash_app.py', '--esptool', str(self.esptool)] + arguments), \
                mock.patch.object(flash_app.subprocess, 'run', side_effect=fake_run), \
                mock.patch('sys.stdout', new_callable=io.StringIO), \
                mock.patch('sys.stderr', new_callable=io.StringIO):
            flash_app.main()
        return commands

    def test_stock_boot_metadata_selects_app0(self):
        # the stock boot_app0.bin: record 0 = sequence 1, record 1 = sequence 0, both with undefined state
        selection = flash_app.select_slot(bytes(metadata([(1, 0xffffffff), (0, 0xffffffff)])))
        self.assertEqual(selection['offset'], 0x10000)

    def test_explicit_rl_artifact_directory_and_offline_plan(self):
        target=self.root/'dist-rl';target.mkdir()
        (target/'firmware.bin').write_bytes(self.original)
        (target/'manifest.json').write_text(json.dumps(self.manifest))
        # The explicit directory must win, even if the original dist is corrupt.
        self.image.write_bytes(b'invalid original dist')
        self.assertEqual(self.invoke(['--plan','--artifact-dir',str(target)]),[])

    def test_selection_tracks_newest_valid_sequence_and_ties(self):
        cases = [([(1, 2), (2, 2)], 0x190000, 1),
                 ([(3, 2), (2, 2)], 0x10000, 0),
                 ([(7, 2), (7, 2)], 0x10000, 0),
                 ([(3, 3), (2, 2)], 0x190000, 1),
                 ([(3, 4), (2, 2)], 0x190000, 1),
                 ([(1, 0xffffffff), (0, 0xffffffff)], 0x10000, 0),
                 ([(0xfffffffe, 2), (0xfffffffd, 2)], 0x190000, 0)]
        for records, offset, record in cases:
            with self.subTest(records=records):
                selection = flash_app.select_slot(metadata(records))
                self.assertEqual((selection['offset'], selection['record']), (offset, record))
        corrupt = metadata([(1, 2), (2, 2)])
        corrupt[0x101c] ^= 1
        self.assertEqual(flash_app.select_slot(corrupt)['offset'], 0x10000)

    def test_unknown_pending_empty_or_bad_length_metadata_refused(self):
        variants = [metadata([]), metadata([(0, 2)]), metadata([(0xffffffff, 2)]),
                    metadata([(1, 3), (2, 4)]), b'', bytes(8191), bytes(8193)]
        for state in (0, 1, 99):
            variants.extend([metadata([(1, state)]), metadata([(1, state), (2, 2)])])
        for data in variants:
            with self.subTest(data_length=len(data), prefix=data[:32]):
                with self.assertRaises(ValueError):
                    flash_app.select_slot(data)

    def test_original_table_and_layout_validation(self):
        digest = hashlib.sha256(self.table).hexdigest()
        flash_app.check_partition_table(self.table, digest)
        with self.assertRaises(ValueError):
            flash_app.check_partition_table(self.table[:-1], digest)
        corrupt = bytearray(self.table)
        corrupt[4] ^= 1
        with self.assertRaises(ValueError):
            flash_app.check_partition_table(corrupt, digest)
        # Even a matching manifest hash cannot authorize a changed app/OTA layout.
        for target_type in (0, 1):
            changed = bytearray(self.table)
            for offset in range(0, len(changed), 32):
                magic, typ, subtype = struct.unpack_from('<HBB', changed, offset)
                if magic != 0x50aa:
                    break
                if typ == target_type and (typ == 0 or subtype == 0):
                    struct.pack_into('<I', changed, offset + 4, 0x310000)
                    break
            else:
                self.fail('Fixture does not contain the required partition')
            with self.assertRaises(ValueError):
                flash_app.check_partition_table(changed, hashlib.sha256(changed).hexdigest())

    def test_only_app_slot_ranges_and_sector_boundaries_allowed(self):
        for start in flash_app.SLOTS:
            for size in (1, 4095, 4096, 4097, flash_app.SLOT_SIZE):
                args = flash_app.flash_arguments({'offset': start}, 'firmware.bin', size)
                self.assertEqual(args, ['write_flash', '--flash_mode', 'keep', '--flash_freq', 'keep',
                                        '--flash_size', 'keep', hex(start), 'firmware.bin'])
            for size in (-1, 0, flash_app.SLOT_SIZE + 1):
                with self.assertRaises(ValueError):
                    flash_app.flash_arguments({'offset': start}, 'f.bin', size)
        for start in (0, 0x1000, 0x8000, 0xe000, 0x11000, 0x310000, 0x330000):
            with self.assertRaises(ValueError):
                flash_app.flash_arguments({'offset': start}, 'f.bin', 4096)

    def test_full_firmware_manifest_is_required_with_strict_booleans(self):
        self.assertEqual(flash_app.check_image(self.image, self.manifest)['bytes'], len(self.original))
        invalid = [('telemetry_only', True), ('telemetry_only', 0), ('telemetry_only', None),
                   ('physical_motion_enabled', False), ('physical_motion_enabled', 1),
                   ('physical_motion_enabled', None), ('environment', 'rider2-bringup')]
        for field, value in invalid:
            bad = copy.deepcopy(self.manifest)
            bad[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                flash_app.check_image(self.image, bad)
        for field in ('telemetry_only', 'physical_motion_enabled', 'environment'):
            bad = copy.deepcopy(self.manifest)
            del bad[field]
            with self.assertRaises(ValueError):
                flash_app.check_image(self.image, bad)

    def test_manifest_digest_and_length_must_match(self):
        for field, value in [('sha256', '0' * 64), ('bytes', len(self.original) + 1)]:
            bad = copy.deepcopy(self.manifest)
            bad['artifacts'][0][field] = value
            with self.assertRaises(ValueError):
                flash_app.check_image(self.image, bad)

    def test_rl_only_requires_direct_actor_and_linked_controller_proof(self):
        candidate=copy.deepcopy(self.manifest);candidate['environment']='rider2-rl-only'
        candidate['native_policy']=dict(trained=True,contract=6,mode_on_boot='active',
            linked_metadata_verified=True,action_space='wheels_legs')
        candidate['wheel_controller']=dict(kind='rl_only',legacy_wheel_pid_linked=False,no_fallback=True,
            legs='position_control',actor_leg_offsets=True,explicit_rearm_selects='active',linked_build_marker_verified=True,
            forbidden_symbols_absent={name:True for name in flash_app.RL_ONLY_FORBIDDEN_SYMBOLS})
        flash_app.check_image(self.image,candidate)
        for section,key,value in [
                ('native_policy','mode_on_boot','off'),('native_policy','action_space','residual'),('native_policy','action_space','wheels'),
                ('native_policy','trained',False),('native_policy','linked_metadata_verified',False),
                ('wheel_controller','kind','mixed'),('wheel_controller','legacy_wheel_pid_linked',True),
                ('wheel_controller','no_fallback',False),('wheel_controller','legs','learned'),('wheel_controller','actor_leg_offsets',False),
                ('wheel_controller','explicit_rearm_selects','off'),
                ('wheel_controller','linked_build_marker_verified',False),
                ('wheel_controller','forbidden_symbols_absent',{})]:
            bad=copy.deepcopy(candidate);bad[section][key]=value
            with self.subTest(section=section,key=key),self.assertRaises(ValueError):
                flash_app.check_image(self.image,bad)
        for name in flash_app.RL_ONLY_FORBIDDEN_SYMBOLS:
            bad=copy.deepcopy(candidate);bad['wheel_controller']['forbidden_symbols_absent'][name]=False
            with self.subTest(symbol=name),self.assertRaises(ValueError):flash_app.check_image(self.image,bad)

    @unittest.skipUnless((ROOT/'dist-rl-only/manifest.json').is_file(),'Build RL-only target for artifact validation')
    def test_exported_rl_only_artifact(self):
        manifest=json.loads((ROOT/'dist-rl-only/manifest.json').read_text())
        flash_app.check_image(ROOT/'dist-rl-only/firmware.bin',manifest)

    def test_trained_variant_requires_verified_model_metadata_and_boot_off(self):
        candidate=copy.deepcopy(self.manifest);candidate['environment']='rider2-trained'
        with self.assertRaises(ValueError):flash_app.check_image(self.image,candidate)
        candidate['native_policy']=dict(trained=True,contract=6,mode_on_boot='off',linked_metadata_verified=True)
        self.assertEqual(flash_app.check_image(self.image,candidate)['bytes'],len(self.original))
        for key,value in [('trained',False),('trained',1),('contract',5),('mode_on_boot','active'),('linked_metadata_verified',False)]:
            bad=copy.deepcopy(candidate);bad['native_policy'][key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):flash_app.check_image(self.image,bad)

    def test_target_header_rejected_even_with_valid_hashes(self):
        for options in [dict(chip=2), dict(mode=0), dict(size_frequency=0x1f),
                        dict(size_frequency=0x20), dict(appended_hash=False)]:
            data = synthetic_image(**options)
            manifest = self.write_fixture(data)
            with self.subTest(options=options), self.assertRaises(ValueError):
                flash_app.check_image(self.image, manifest)

    def test_internal_image_corruption_rejected_even_if_manifest_is_rehashed(self):
        variants = [self.original[:8], self.original[:-1], self.original[:48]]
        for offset, value in [(0, 0), (1, 0), (1, 17), (23, 2),
                              (32, self.original[32] ^ 1), (63, self.original[63] ^ 1),
                              (95, self.original[95] ^ 1)]:
            changed = bytearray(self.original)
            changed[offset] = value
            variants.append(changed)
        wrong_length = bytearray(self.original)
        struct.pack_into('<I', wrong_length, 28, 0xffffffff)
        variants.append(wrong_length)
        for index, data in enumerate(variants):
            manifest = self.write_fixture(data)
            with self.subTest(index=index), self.assertRaises(ValueError):
                flash_app.check_image(self.image, manifest)

    @unittest.skipUnless((ROOT / 'dist' / 'manifest.json').is_file(), 'Run export_firmware.py for production artifact validation')
    def test_exported_motor_enabled_artifact(self):
        manifest = json.loads((ROOT / 'dist' / 'manifest.json').read_text(encoding='utf-8'))
        image = ROOT / 'dist' / 'firmware.bin'
        info = flash_app.check_image(image, manifest)
        self.assertEqual(info['bytes'], image.stat().st_size)
        self.assertLessEqual(info['bytes'], flash_app.SLOT_SIZE)

    def test_offline_plan_performs_no_external_operation(self):
        self.assertEqual(self.invoke(['--plan']), [])
        with self.assertRaises(SystemExit):
            self.invoke(['--plan', '--flash'])
        self.assertEqual(self.commands, [])

    def test_default_inspection_reads_only_metadata_and_never_writes(self):
        commands = self.invoke(['--port', 'FAKE_PORT_NEVER_OPEN'])
        self.assertEqual([command[0] for _, command in commands], ['read_flash', 'read_flash', 'flash_id'])
        self.assertEqual(commands[-1][0], 'hard_reset')

    def test_flash_writes_and_verifies_only_the_selected_active_app(self):
        for sequence, expected in [(1, '0x10000'), (2, '0x190000')]:
            with self.subTest(sequence=sequence):
                commands = self.invoke(['--port', 'FAKE_PORT_NEVER_OPEN', '--flash'], ota=metadata([(sequence, 2)]))
                self.assertEqual([command[0] for _, command in commands],
                                 ['read_flash', 'read_flash', 'write_flash', 'verify_flash'])
                self.assertEqual(commands[2][1][-2:], [expected, str(self.image)])
                self.assertEqual(commands[3], ('hard_reset', ['verify_flash', expected, str(self.image)]))
                self.assertTrue(all(after == 'no_reset' for after, _ in commands[:-1]))

    def test_metadata_failure_stops_before_any_write(self):
        with self.assertRaises(ValueError):
            self.invoke(['--port', 'FAKE_PORT_NEVER_OPEN', '--flash'], table=bytes(len(self.table)))
        self.assertEqual(len(self.commands), 1)
        with self.assertRaises(ValueError):
            self.invoke(['--port', 'FAKE_PORT_NEVER_OPEN', '--flash'], ota=metadata([(1, 1)]))
        self.assertEqual([command[0] for _, command in self.commands], ['read_flash', 'read_flash'])

    def test_bad_full_firmware_manifest_rejected_before_external_operations(self):
        for field, value in [('telemetry_only', True), ('physical_motion_enabled', False)]:
            bad = copy.deepcopy(self.manifest)
            bad[field] = value
            self.write_fixture(self.original, bad)
            with self.assertRaises(ValueError):
                self.invoke(['--port', 'FAKE_PORT_NEVER_OPEN', '--flash'])
            self.assertEqual(self.commands, [])

    def test_esptool_failure_prevents_followup_operations(self):
        for operation, expected_count in [('read_flash', 1), ('write_flash', 3), ('verify_flash', 4)]:
            with self.subTest(operation=operation), self.assertRaises(subprocess.CalledProcessError):
                self.invoke(['--port', 'FAKE_PORT_NEVER_OPEN', '--flash'], fail_operation=operation)
            self.assertEqual(len(self.commands), expected_count)


if __name__ == '__main__':
    unittest.main()
