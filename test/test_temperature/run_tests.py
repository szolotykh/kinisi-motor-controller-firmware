"""Test the production TMP102 driver and legacy boards without hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/board_config_generator"))
import generator

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not compiler:
    raise SystemExit("Set CC to a host gcc or clang executable.")
with tempfile.TemporaryDirectory() as folder:
    for revision in ("MP_V1", "MP_V2", "MP_V3", "MP_V4"):
        binary = Path(folder) / (revision + ".exe")
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-const-variable",
            "-Itest/test_temperature/stubs", "-Iinclude", "-Ilib/hardware", "-D" + revision,
            "test/test_temperature/test_temperature.c", "lib/hardware/hw_temperature.c", "-o", str(binary)
        ], cwd=root, check=True)
        scenarios = ("normal", "wrap", "init_failure", "write_failure", "read_failure", "bad_config", "status_bits") if revision == "MP_V4" else ("absent",)
        for scenario in scenarios:
            print(revision + ": " + scenario, flush=True)
            subprocess.run([str(binary), scenario], check=True, timeout=10)

    # Include two sensors to verify that recovery invalidates the whole shared bus.
    boards = generator.load_boards(root / "boards")
    board = next(board for board in boards if board["board"]["build_define"] == "MP_V4")
    board["temperature_sensors"].append(dict(id=1, type="TMP102", bus="sensor_i2c", address=0x49))
    generator.validate(board)
    for name, content in generator.render(boards).items():
        (Path(folder) / name).write_text(content, encoding="utf-8")
    binary = Path(folder) / "recovery.exe"
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-const-variable",
        "-I" + folder, "-Itest/test_temperature/stubs", "-Iinclude", "-Ilib/hardware", "-DMP_V4",
        "test/test_temperature/test_bus_recovery.c", "lib/hardware/hw_temperature.c", "-o", str(binary)
    ], cwd=root, check=True)
    for scenario in ("busy", "wrap", "sda_release", "sda_stuck", "scl_stuck", "wrap_scl_stuck", "init_failure"):
        print("I2C3 recovery: " + scenario, flush=True)
        subprocess.run([str(binary), scenario], check=True, timeout=10)
