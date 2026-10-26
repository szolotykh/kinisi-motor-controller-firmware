"""Compile the real current-sense driver with ADC failure injection for all boards."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
if not compiler:
    raise SystemExit('Set CC to a host C compiler.')
with tempfile.TemporaryDirectory() as folder:
    for revision in ('MP_V1', 'MP_V2', 'MP_V3', 'MP_V4'):
        binary = Path(folder) / (revision + '.exe')
        subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-const-variable',
                        '-Itest/test_current_sense/stubs', '-Iinclude', '-Ilib/hardware', '-D' + revision,
                        'test/test_current_sense/test_current_sense.c', 'lib/hardware/hw_current_sense.c',
                        '-o', str(binary)], cwd=root, check=True)
        subprocess.run([str(binary)], check=True, timeout=10)
