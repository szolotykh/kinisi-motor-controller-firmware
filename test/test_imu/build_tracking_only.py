"""Build a private v0.4 profile with tracking enabled and fusion disabled.

The relocations in main.o verify that boot actually installs the physical producer.
This integration test requires the same PlatformIO STM32 toolchain as a target build.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platformio", default="platformio")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory() as folder:
        project = Path(folder)
        for name in ("boards", "include", "lib", "src", "tools"):
            shutil.copytree(root/name, project/name, ignore=shutil.ignore_patterns("__pycache__"))
        shutil.copy2(root/"platformio.ini", project)
        profile = project/"boards/v0.4.json"
        data = json.loads(profile.read_text())
        assert data["capabilities"]["imu_tracking"]
        data["capabilities"]["fusion_experiment"] = False
        profile.write_text(json.dumps(data))
        subprocess.run([args.platformio, "run", "-e", "v0_4"], cwd=project, check=True)
        main_object = project/".pio/build/v0_4/src/main.o"
        symbols = subprocess.check_output([args.nm, str(main_object)], text=True)
        for required in ("imu_tracking_install_producer", "hw_imu_poll_observations", "hw_imu_read_tracking"):
            assert required in symbols, required
        assert "platform_fusion_install_imu" not in symbols and "platform_fusion_imu_epoch" not in symbols
        print("PASS tracking-only target boot links the independent BMI088 acquisition task and continuous reader")


if __name__ == "__main__":
    main()
