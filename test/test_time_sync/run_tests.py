# ------------------------------------------------------------
# File name: run_tests.py
# ------------------------------------------------------------
"""Exercise time sync and the real protocol/session code using a virtual clock."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys
import json

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not compiler:
    raise SystemExit("Set CC to a host gcc or clang executable.")
with tempfile.TemporaryDirectory() as folder:
    binary = Path(folder) / "controller_stop.exe"
    subprocess.run([
        compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-variable",
        "-Itest/test_time_sync/stubs", "-Iinclude", "-Ilib/hardware",
        "-Ilib/libcontrollers/src", "-Ilib/protocol",
        "test/test_time_sync/test_controller_stop.c", "src/controllers_manager.c",
        "lib/libcontrollers/src/pid_controller.c", "lib/libcontrollers/src/loop_frequency.c",
        "-lm", "-o", str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
    binary = Path(folder) / "requirements.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Iinclude", "-Ilib/protocol", "-Ilib/command_requirements",
        "test/test_time_sync/test_requirements.c", "lib/command_requirements/command_requirements.c",
        "lib/protocol/protocol.c", "-o", str(binary)
    ], cwd=root, check=True)
    output = subprocess.run([str(binary)], check=True, capture_output=True, text=True).stdout
    schema = json.loads((root / "commands.json").read_text())
    names = {e['code']: e['name'] for e in schema['error_codes']}
    commands = {int(c['code'], 16): c for c in schema['commands']}
    for line in output.splitlines():
        command, error = map(int, line.split(','))
        assert names[error] in commands[command]['errors'], line
    print("Resource checks, error framing, ownership precedence and JSON error declarations passed")
    binary = Path(folder) / "time_sync.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Iinclude", "-Ilib/initialization", "-Ilib/protocol",
        "-Ilib/time_sync", "-Ilib/connection",
        "test/test_time_sync/test_time_sync.c", "lib/time_sync/time_sync.c",
        "lib/connection/connection.c", "lib/protocol/protocol.c",
        "lib/initialization/initialization.c", "-o", str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
    binary = Path(folder) / "session_features.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Iinclude", "-Ilib/initialization", "-Ilib/protocol", "-Ilib/time_sync", "-Ilib/connection",
        "test/test_time_sync/test_session_features.c", "lib/time_sync/time_sync.c",
        "lib/connection/connection.c", "lib/protocol/protocol.c", "lib/initialization/initialization.c",
        "-o", str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
    binary = Path(folder) / "odometry_timestamp.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Itest/test_time_sync/stubs", "-Iinclude", "-Ilib/hardware", "-Ilib/libcontrollers/src", "-Ilib/protocol",
        "test/test_time_sync/test_odometry_timestamp.c", "src/odometry_manager.c",
        "src/odometry_integrator.c", "lib/libcontrollers/src/loop_frequency.c",
        "-lm", "-o", str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
    binary = Path(folder) / "hw_clock.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Itest/test_time_sync/stubs", "-Ilib/hardware",
        "test/test_time_sync/test_hw_clock.c", "lib/hardware/hw_clock.c",
        "-o", str(binary)
    ], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)
subprocess.run([sys.executable, "test/test_time_sync/test_error_schema.py"], cwd=root, check=True)
