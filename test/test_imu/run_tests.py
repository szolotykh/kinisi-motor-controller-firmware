"""Exercise the production BMI088 SPI driver and absence on legacy boards."""
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
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-const-variable",
            "-Itest/test_imu/stubs", "-Iinclude", "-Ilib/hardware", "-D" + revision,
            "test/test_imu/test_imu.c", "lib/hardware/hw_imu.c", "-o", str(binary)
        ], cwd=root, check=True)
        scenarios = ("normal", "wrap", "init_failure", "transfer_failure", "reset_failure", "wrong_id", "bad_config") if revision == "MP_V4" else ("absent",)
        for scenario in scenarios:
            print(revision + ": " + scenario, flush=True)
            subprocess.run([str(binary), scenario], check=True, timeout=10)
