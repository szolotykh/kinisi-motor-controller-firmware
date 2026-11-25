"""Compile the target predicate using the installed STM32CubeF4 vendor register definitions."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
packages = Path(os.getenv("PLATFORMIO_PACKAGES_DIR", Path.home()/".platformio/packages"))
drivers = Path(os.getenv("STM32CUBE_F4_DRIVERS", packages/"framework-stm32cubef4/Drivers"))
includes = [root/"include", root/"lib/hardware", drivers/"CMSIS/Include",
            drivers/"CMSIS/Device/ST/STM32F4xx/Include", drivers/"STM32F4xx_HAL_Driver/Inc"]
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
with tempfile.TemporaryDirectory() as folder:
    binary = Path(folder)/"counter.exe"
    subprocess.run([compiler, "-std=c11", "-DSTM32F405xx", "-Wno-pointer-to-int-cast",
                    "-Wno-int-to-pointer-cast", *["-I"+str(i) for i in includes],
                    str(root/"test/test_encoder_hardware/test_counter.c"), "-o", str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
