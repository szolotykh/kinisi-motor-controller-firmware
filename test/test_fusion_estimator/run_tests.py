"""Compile dependency-free production estimator unit checks without hardware."""
import os
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as temp:
    executable=Path(temp)/('fusion.exe' if os.name=='nt' else 'fusion')
    subprocess.run([os.environ.get('CC','gcc'),'-std=c11','-O2','-Wall','-Wextra','-Werror',
                    '-I'+str(ROOT/'include'),str(Path(__file__).with_name('test_estimator.c')),
                    '-lm','-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
