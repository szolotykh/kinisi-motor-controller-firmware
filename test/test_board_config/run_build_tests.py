"""Build a synthetic new revision and verify its startup and driver fault paths.

Uses an isolated project under .pio; no synthetic board is added to the real project.
Requires PlatformIO and its ARM toolchain (or --pio / --nm paths).
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2]
    core = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pio", default=shutil.which("pio"))
    parser.add_argument("--nm", default=shutil.which("arm-none-eabi-nm"))
    args = parser.parse_args()
    nm = args.nm or str(core / "packages/toolchain-gccarmnoneeabi/bin" /
                        ("arm-none-eabi-nm.exe" if os.name == "nt" else "arm-none-eabi-nm"))
    if not args.pio:
        parser.error("PlatformIO is not on PATH; pass --pio with its executable path")
    if not Path(nm).is_file():
        parser.error("ARM nm was not found; install the PlatformIO toolchain or pass --nm")

    workspace = (root / ".pio").resolve()
    workspace.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="board-portability-", dir=workspace) as folder:
        project = Path(folder).resolve()
        # Keep both the isolated copy and automatic recursive cleanup inside .pio.
        if project.parent != workspace:
            raise RuntimeError("Unexpected temporary project location")
        for name in ("src", "lib", "include", "tools", "boards"):
            shutil.copytree(root / name, project / name,
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        config = (root / "platformio.ini").read_text(encoding="utf-8")
        board = json.loads((project / "boards/v0.4.json").read_text(encoding="utf-8"))
        # Choose a fresh selector even after real revisions beyond v0.4 are added.
        number = 5
        while f"MP_V{number}" in (project / "include/board_revision.h").read_text(encoding="utf-8"):
            number += 1
        board["board"].update(revision=f"0.{number}.0", build_define=f"MP_V{number}")
        (project / f"boards/v0.{number}.json").write_text(json.dumps(board, indent=2) + "\n", encoding="utf-8")
        config += f"""
[env:future_board_test]
extends = env:v0_4
build_flags =
  ${{env:genericSTM32F405RG.build_flags}}
  -DMP_V{number}
"""
        (project / "platformio.ini").write_text(config, encoding="utf-8")
        subprocess.run([args.pio, "run", "--project-dir", str(project), "-e", "future_board_test"],
                       cwd=root, check=True)
        build = project / ".pio/build/future_board_test"
        # Inspect the real compiled objects, so revision-specific #ifdefs cannot
        # silently remove either call site for the synthetic board.
        for source, symbol in (("main.o", "hw_motor_init"),
                               ("command_handler.o", "hw_drv8873_fault_active")):
            output = subprocess.check_output([nm, "-u", str(build / "src" / source)], text=True)
            if symbol not in {line.split()[-1] for line in output.splitlines() if line.split()}:
                raise AssertionError(f"MP_V{number}: {source} is missing {symbol}")
        print(f"MP_V{number}: full firmware build includes motor startup and command fault handling")


if __name__ == "__main__":
    main()
