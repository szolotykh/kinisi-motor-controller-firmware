# ------------------------------------------------------------
# File name: build_identity.py
# ------------------------------------------------------------
"""Embed the firmware's Git revision without modifying tracked source files."""
import subprocess

Import("env")

def git(*args):
    """Run Git in the PlatformIO project and return stripped stdout; propagate failures."""
    return subprocess.check_output(
        ["git", *args], cwd=env.subst("$PROJECT_DIR"), text=True
    ).strip()

try:
    revision = git("rev-parse", "HEAD")[:16]
except (OSError, subprocess.CalledProcessError):
    revision = "0000000000000000"

env.Append(CPPDEFINES=[
    ("KINISI_BUILD_ID_HIGH", "0x" + revision[:8] + "U"),
    ("KINISI_BUILD_ID_LOW", "0x" + revision[8:] + "U"),
])
