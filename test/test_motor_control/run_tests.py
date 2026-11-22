"""Exercise production motor control and DRV8873 SPI setup using a host HAL fake."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
if not compiler:
    raise SystemExit("Set CC to a host gcc or clang executable.")
def build(binary, revision, extra=()):
    subprocess.run([
        compiler, "-DKINISI_TEST_PWM_ROLLOVER", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-const-variable", "-Wno-unused-variable", "-Wno-unused-function",
        *extra, "-Itest/test_motor_control/stubs", "-Iinclude", "-Ilib/hardware", "-D" + revision,
        "test/test_motor_control/test_motor_control.c", "lib/hardware/hw_motor.c",
        "lib/hardware/hw_drv8873.c", "-lm", "-o", str(binary)
    ], cwd=root, check=True)


with tempfile.TemporaryDirectory() as folder:
    for revision in ("MP_V1", "MP_V2", "MP_V3", "MP_V3_1", "MP_V4"):
        binary = Path(folder) / (revision + ".exe")
        build(binary, revision)
        print(revision, flush=True)
        subprocess.run([str(binary)], check=True, timeout=10)
        for clock in ("apb2_div1", "apb2_div4", "apb1_div1", "deferred", "driver_sleep"):
            subprocess.run([str(binary), clock], check=True, timeout=10)
        print(revision + ": all motor timers verified at supported PWM frequencies across divided/undivided APB clocks")

    # Neither released DRV8870 board routes a complementary channel. Exercise
    # its distinct timer start and polarity at the same GPIO-to-PWM handoff.
    fixture = Path(folder) / "hw_config.h"
    header = (root / "include/hw_config.h").read_text()
    fixture.write_text(header.replace("{TIM_CHANNEL_1, GPIOA, GPIO_PIN_8, false}",
                                      "{TIM_CHANNEL_1, GPIOB, GPIO_PIN_13, true}"))
    binary = Path(folder) / "complementary.exe"
    build(binary, "MP_V3", ("-I" + folder, "-DTEST_COMPLEMENTARY_ROUTE"))
    subprocess.run([str(binary), "driver_sleep"], check=True, timeout=10)
    print("Synthetic TIM1_CH1N: low GPIO hold and zero-duty, proper-polarity PWM handoff passed")
