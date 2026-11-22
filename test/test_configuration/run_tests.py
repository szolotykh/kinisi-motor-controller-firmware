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
    imu_test = out / 'imu_only.exe'
    subprocess.run([compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-Iinclude',
                    'test/test_imu_only/test_imu_only.c', '-lm', '-o', str(imu_test)], cwd=root, check=True)
    subprocess.run([str(imu_test)], check=True)
    for revision in (2, 3, "3_1", 4):
        binary = out / f'configuration_v{revision}.exe'
        usb_objects = []
        usb_flags = []
        if revision == "3_1":
            usb_flags = ['-DKINISI_CONFIGURATION_REAL_USB', '-Itest/test_usb_vbus/stubs']
            # Link the real Nano VBUS lifecycle beside the real command task.
            # USB has its own stack/HAL boundary; the controller fixture uses
            # narrower motor, clock and RTOS doubles.
            for src in ('lib/usb/usb_device.c', 'test/test_configuration/nano_usb_stack.c'):
                obj = out / (Path(src).stem + '.o')
                subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-DMP_V3_1',
                                '-Itest/test_usb_vbus/stubs', '-Ilib/usb', '-Iinclude', '-Ilib/utils/src',
                                '-c', src, '-o', str(obj)], cwd=root, check=True)
                usb_objects.append(str(obj))
        subprocess.run([compiler, '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        '-Wno-unused-parameter', '-Wno-unused-variable', '-Wno-error=absolute-value',
                        f'-DMP_V{revision}', '-DMOTOR_CONTROL_MODEL_ENABLE_DIRECTION=2',
                        f'-DBOARD_HAS_DRIVER_SLEEP={int(revision != 2)}',
                        f'-DBOARD_HAS_DRV8870_SLEEP={int(revision in (3, "3_1"))}',
                        '-include', 'board_revision.h',
                        f'-DMOTOR_CONTROL_MODEL={2 if revision == 4 else 1}',
                        *['-I' + path for path in includes], *usb_flags, *sources, *usb_objects, '-lm', '-o', str(binary)],
                       cwd=root, check=True)

        def run(mode, image, platform):
            subprocess.run([str(binary), mode, str(image), str(platform)], check=True, timeout=10)

        if revision == 4:
            run('imu-tracking', out / 'unconfigured.fram', 1)
            for preset in (1, 2, 3):
                run('fusion', out / 'unconfigured.fram', preset)
            print('PASS fusion mode, source age, frame/bias, matched wheel outputs, position/velocity fail-stop and rearm on all presets')
        if revision != 4:
            run('unsupported', out / 'unused.fram', 1)
            if revision == "3_1":
                run('usb-idle', out / 'unused.fram', 1)
                print('PASS Nano same-session USB idle interrupt/ping/read/wake and VBUS loss/renewed handshake')
            if revision != 2:
                for bus in (0, 1):
                    for mode in ('sleep-queues', 'motor-brake-queues', 'platform-brake-queues',
                                 'idle-queues', 'idle-motor-brake-queues', 'idle-platform-brake-queues'):
                        run(mode, out / 'unconfigured.fram', bus)
                run('brake-policy', out / 'unconfigured.fram', 1)
                run('sleep-policy', out / 'unused.fram', 1)
                run('idle-policy', out / 'unused.fram', 1)
                run('idle-reboot', out / 'unused.fram', 1)
                print(f'PASS MP_V{revision} shallow idle, independent sleep, interrupted wakes and stale drive/brake')
            print(f'PASS MP_V{revision} power capability, transport gates and no FRAM access')
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
