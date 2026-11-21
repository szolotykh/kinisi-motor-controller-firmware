"""Exercise the production FRAM journal with byte-granular interrupted writes."""
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
    binary = Path(folder) / "test_config_store.exe"
    subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-DMP_V4",
                    "-Iinclude", "-Ilib/hardware", "-Ilib/config_store",
                    "test/test_config_store/test_config_store.c", "lib/config_store/config_store.c",
                    "-o", str(binary)], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
