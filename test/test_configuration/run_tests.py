"""Restart real firmware session, command and configuration modules with only the FRAM image retained."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import zlib

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
if not compiler:
    raise SystemExit('Set CC to a host gcc or clang.')
includes = ['test/test_configuration/stubs', 'test/test_position/stubs', 'test/test_time_sync/stubs',
            'include', 'lib/hardware', 'lib/utils/src',
            'lib/libcontrollers/src', 'lib/protocol', 'lib/config_store', 'lib/initialization',
            'lib/connection', 'lib/time_sync', 'lib/command_requirements']
sources = ['test/test_configuration/test_idle_hook.c', 'test/test_configuration/test_configuration.c', 'test/test_configuration/test_sleep_queues.c',
           'lib/utils/src/message_queue.c', 'src/commands_manager.c', 'src/command_handler.c',
           'src/controller_configuration.c', 'src/platform.c', 'src/platform_differential.c',
           'src/platform_omni.c', 'src/platform_mecanum.c', 'src/controllers_manager.c',
           'src/platform_position.c', 'src/encoder_odometry.c', 'src/odometry_manager.c',
           'src/odometry_integrator.c', 'lib/libcontrollers/src/pid_controller.c',
           'lib/libcontrollers/src/loop_frequency.c', 'lib/libcontrollers/src/position_controller.c',
           'lib/command_requirements/command_requirements.c', 'lib/config_store/config_store.c',
           'lib/connection/connection.c', 'lib/time_sync/time_sync.c', 'lib/protocol/protocol.c',
           'lib/initialization/initialization.c']

with tempfile.TemporaryDirectory() as folder:
    out = Path(folder)
    for revision in (3, 4):
        binary = out / f'configuration_v{revision}.exe'
        subprocess.run([compiler, '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        '-Wno-unused-parameter', '-Wno-unused-variable', '-Wno-error=absolute-value',
                        f'-DMP_V{revision}', '-DMOTOR_CONTROL_MODEL_ENABLE_DIRECTION=2',
                        f'-DBOARD_HAS_DRIVER_SLEEP={int(revision == 4)}',
                        f'-DBOARD_HAS_BOARD_IDLE={int(revision == 4)}',
                        f'-DMOTOR_CONTROL_MODEL={2 if revision == 4 else 1}',
                        *['-I' + path for path in includes], *sources, '-lm', '-o', str(binary)],
                       cwd=root, check=True)

        def run(mode, image, platform):
            subprocess.run([str(binary), mode, str(image), str(platform)], check=True, timeout=10)

        if revision == 3:
            run('unsupported', out / 'unused.fram', 1)
            print('PASS v0.3 explicit unsupported save preserves setup and never calls FRAM')
            continue
        for bus in (0, 1):
            run('sleep-queues', out / 'unconfigured.fram', bus)
            run('idle-queues', out / 'unconfigured.fram', bus)
        print('PASS both transport wake directions, backpressure, fragmented USB input and fresh-motion boundary')
        run('idle-policy', out / 'idle.fram', 1)
        run('wake-boot', out / 'idle.fram', 1)
        print('PASS board idle restart, motion cancellation, shallow WFI, failed entry and wake, independent inhibit and retry')
        run('sleep-policy', out / 'sleep.fram', 1)
        run('wake-boot', out / 'sleep.fram', 1)
        print('PASS driver command gating, target cancellation, reconnect, volatile reset and saved configuration')
        for feedback in ('encoder-free', 'mixed'):
            run(feedback + '-save', out / (feedback + '.fram'), 1)
            run(feedback + '-verify', out / (feedback + '.fram'), 1)
            print(f'PASS {feedback} platform has stopped outputs and rejects feedback operations until all wheels are configured')
        for platform in (1, 2, 3):
            fram = out / f'{platform}.fram'
            run('blank', fram, platform)
            run('save', fram, platform)
            run('verify', fram, platform)
            # Damage the only committed setup. For incompatibility give the
            # unknown version a valid CRC so checksum rejection cannot mask it.
            for damage in ('corrupt', 'incompatible'):
                damaged = out / f'{platform}-{damage}.fram'
                data = bytearray(fram.read_bytes())
                if damage == 'corrupt':
                    data[16] ^= 0x80
                else:
                    data[8:10] = (2).to_bytes(2, 'little')
                    data[208:212] = zlib.crc32(data[:208]).to_bytes(4, 'little')
                damaged.write_bytes(data)
                run('blank', damaged, platform)
                run('save', damaged, platform)
                run('verify', damaged, platform)
            replacement = platform % 3 + 1
            run('interrupt', fram, replacement)
            run('verify', fram, platform)
            run('save', fram, replacement)
            run('verify', fram, replacement)
            run('driver-fault', fram, replacement)
            run('encoder-fault', fram, replacement)
            run('bad-frequency', fram, replacement)
            run('blank', fram, replacement)
            print(f'PASS platform {platform}: reboot, live readback, reconnect edits, interrupted replacement, '
                  'replacement, stopped outputs, driver/encoder rollback and invalid-frequency default')
