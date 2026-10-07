"""Exercise production position control and platform dispatch without hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
if not compiler:
    raise SystemExit('Set CC to a host gcc or clang executable.')
with tempfile.TemporaryDirectory() as folder:
    binary = Path(folder) / 'platform_position.exe'
    subprocess.run([
        compiler, '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
        '-Wno-error=absolute-value',  # Existing open-loop mecanum code uses abs(double).
        '-Itest/test_position/stubs', '-Itest/test_time_sync/stubs', '-Iinclude',
        '-Ilib/hardware', '-Ilib/libcontrollers/src', '-Ilib/protocol',
        'test/test_position/test_platform_position.c', 'src/platform_position.c',
        'src/platform.c', 'src/platform_differential.c', 'src/platform_omni.c', 'src/platform_mecanum.c',
        'lib/libcontrollers/src/position_controller.c', '-lm', '-o', str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
