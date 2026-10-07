"""Run the production velocity PID directly with a host C compiler."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
if not compiler:
    raise SystemExit('Set CC to a host gcc or clang executable.')
with tempfile.TemporaryDirectory() as directory:
    binary = Path(directory) / 'velocity_pid.exe'
    subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-Ilib/libcontrollers/src', 'test/test_velocity_pid/test_velocity_pid.c',
                    'lib/libcontrollers/src/pid_controller.c', '-lm', '-o', str(binary)],
                   cwd=root, check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
