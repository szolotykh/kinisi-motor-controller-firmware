"""Test board validation, generated C mappings and production LED behavior."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import test_generator

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not compiler:
    raise SystemExit("Set CC to a host gcc or clang executable.")
subprocess.run([sys.executable, str(Path(__file__).with_name("test_generator.py"))], cwd=root, check=True)
flags = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-const-variable",
         "-Itest/test_board_config/stubs", "-Iinclude", "-Ilib/hardware"]
with tempfile.TemporaryDirectory() as folder:
    tmp = Path(folder)
    snapshots = []
    for revision in ("MP_V1", "MP_V2", "MP_V3", "MP_V4"):
        for source in ("dump_config.c", "test_status_led.c"):
            binary = tmp / (revision + source + ".exe")
            sources = ["test/test_board_config/" + source]
            if source == "test_status_led.c":
                sources.append("lib/hardware/hw_gpio.c")
            subprocess.run(flags + ["-D" + revision] + sources + ["-o", str(binary)], cwd=root, check=True)
            output = subprocess.check_output([str(binary)], text=True)
            if source == "dump_config.c":
                snapshots.append(output)
            else:
                print(revision + ": " + output.strip())
    expected = (root / "test/test_board_config/expected_mappings.txt").read_text(encoding="utf-8")
    if "".join(snapshots) != expected:
        raise AssertionError("Generated C mappings differ from the pre-generator board mappings")
    print("All four compiled C configurations match the original pin/timer mappings")

    # Test an active-low primary LED and a second LED without changing production profiles.
    boards = test_generator.g.load_boards(root / "boards")
    boards[3]["status_leds"][0]["active_low"] = True
    boards[3]["status_leds"].append(dict(id=1, pin="PE0", active_low=True))
    test_generator.g.validate(boards[3])
    for name, contents in test_generator.g.render(boards).items():
        (tmp / name).write_text(contents, encoding="utf-8")
    binary = tmp / "multiple_leds.exe"
    subprocess.run([compiler, "-I" + str(tmp)] + flags[1:] + ["-DMP_V4",
                   "test/test_board_config/test_status_led.c", "lib/hardware/hw_gpio.c", "-o", str(binary)], cwd=root, check=True)
    subprocess.run([str(binary)], check=True)

    result = subprocess.run(flags + ["-DMP_V1", "-DMP_V4", "-fsyntax-only", "test/test_board_config/dump_config.c"],
                            cwd=root, text=True, capture_output=True)
    assert result.returncode != 0 and "Select exactly one supported board revision" in result.stderr
    print("Conflicting board selections fail compilation")
