"""Run production USB device lifecycle on a native host; no connected board needed."""
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
    for revision in ("MP_V1", "MP_V2", "MP_V3", "MP_V3_1", "MP_V4"):
        binary = Path(folder) / (revision + ".exe")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-parameter",
                        "-D" + revision, "-Itest/test_usb_vbus/stubs", "-Ilib/usb",
                        "-Itest/test_configuration/stubs", "-Itest/test_position/stubs",
                        "-Itest/test_time_sync/stubs", "-Iinclude", "-Ilib/utils/src",
                        "-Ilib/hardware", "-Ilib/connection", "-Ilib/protocol", "-Ilib/time_sync",
                        "-Ilib/libcontrollers/src", "-Ilib/initialization",
                        "test/test_usb_vbus/test_usb_vbus.c", "test/test_usb_vbus/test_quiesce_order.c",
                        "lib/utils/src/message_queue.c", "lib/connection/connection.c",
                        "lib/time_sync/time_sync.c", "lib/protocol/protocol.c",
                        "lib/initialization/initialization.c", "-lm",
                        "lib/usb/usb_device.c", "-o", str(binary)], cwd=root, check=True)
        for boot in ("absent", "present"):
            print(revision, boot, flush=True)
            subprocess.run([str(binary), boot], check=True, timeout=5)
        if revision == "MP_V3_1":
            for failure in ("start-fail", "stop-fail", "flush-fail"):
                subprocess.run([str(binary), "absent", failure], check=True, timeout=5)
