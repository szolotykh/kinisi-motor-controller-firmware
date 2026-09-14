# ------------------------------------------------------------
# File name: run_tests.py
# ------------------------------------------------------------
"""Run isolated INIT protocol tests with a host C compiler, without hardware."""
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
    for revision in ("MP_V1", "MP_V2", "MP_V3"):
        binary = Path(folder) / (revision + ".exe")
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-Iinclude", "-Ilib/initialization", "-Ilib/protocol", "-D" + revision,
            "-DKINISI_BUILD_ID_HIGH=0x12345678U",
            "-DKINISI_BUILD_ID_LOW=0x90abcdefU",
            "test/test_initialization/test_init.c",
            "lib/initialization/initialization.c", "-o", str(binary)
        ], cwd=root, check=True)
        subprocess.run([str(binary)], check=True)
        print(revision + ": INIT wire format, versions, retry, compatibility, capabilities and lengths passed")

    binary = Path(folder) / "protocol.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Iinclude", "-Ilib/protocol", "-Ilib/initialization", "-Ilib/utils/src",
        "test/test_initialization/test_protocol.c", "lib/protocol/protocol.c",
        "lib/initialization/initialization.c", "lib/utils/src/message_queue.c",
        "-o", str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
    print("Shared reply framing, errors, ACKs, argument validation and fragmented requests passed")
