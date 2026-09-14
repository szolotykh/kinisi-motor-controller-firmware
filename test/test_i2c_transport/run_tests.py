# ------------------------------------------------------------
# File name: run_tests.py
# ------------------------------------------------------------
"""Verify I2C IRQ wiring, consecutive master reads, and interrupted reply recovery."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    """Compile the production transport with a focused HAL mock, then run the regressions."""
    root = Path(__file__).resolve().parents[2]
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise SystemExit("Set CC to a host gcc or clang executable.")
    with tempfile.TemporaryDirectory() as folder:
        binary = Path(folder) / "i2c_transport.exe"
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-Wno-unused-parameter", "-Wno-pointer-sign",
            "-Itest/test_i2c_transport/stubs", "-Ilib/utils/src",
            "test/test_i2c_transport/test_i2c_transport.c",
            "lib/utils/src/message_queue.c", "-o", str(binary)
        ], cwd=root, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
