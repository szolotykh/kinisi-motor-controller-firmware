"""Exercise production motor control and DRV8873 SPI setup using a host HAL fake."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not compiler:
    raise SystemExit("Set CC to a host gcc or clang executable.")
with tempfile.TemporaryDirectory() as folder:
    for revision in ("MP_V1", "MP_V2", "MP_V3", "MP_V4"):
        binary = Path(folder) / (revision + ".exe")
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-Wno-unused-const-variable", "-Wno-unused-variable", "-Wno-unused-function",
            "-Itest/test_motor_control/stubs", "-Iinclude", "-Ilib/hardware", "-D" + revision,
            "test/test_motor_control/test_motor_control.c", "lib/hardware/hw_motor.c",
            "lib/hardware/hw_drv8873.c", "-lm", "-o", str(binary)
        ], cwd=root, check=True)
        print(revision, flush=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        for clock in ("apb2_div1", "apb2_div4", "apb1_div1"):
            subprocess.run([str(binary), clock], check=True, timeout=10)
        print(revision + ": all motor timers verified at 100 kHz across divided/undivided APB clocks")
