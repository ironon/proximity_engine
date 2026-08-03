"""PlatformIO pre-build script: stamp the build with its provenance.

Shared by the watch and the anchor (both already symlink this directory as a
library, so referencing it costs nothing):

    extra_scripts = pre:../proximity_engine/tools/inject_sha.py

Defines PROX_FW_SHA as "<firmware>/<engine>", each a 7-char short SHA with a
trailing '+' when that repo has uncommitted changes. BOTH matter and neither
substitutes for the other: the firmware SHA says which wiring produced a
capture, the engine SHA says which scoring did. A corpus that records only one
of them cannot be pooled safely the first time the other moves.

The engine constants are covered separately by prox_capture_cfg_hash(), which is
computed rather than stamped — this is the code, that is the configuration.

Falls back to "dev" for anything it cannot determine, which is itself a useful
signal: a session stamped "dev" was built from something nobody can reconstruct.
"""

import subprocess
from os.path import abspath, dirname, join

Import("env")  # noqa: F821  (injected by PlatformIO/SCons)

# SCons exec's this script, so __file__ is not defined. Locate the engine from
# the project instead: both firmwares sit beside proximity_engine.


def short_sha(repo_dir):
    def git(*args):
        return subprocess.run(
            ["git", "-C", repo_dir, *args],
            capture_output=True, text=True, timeout=5,
        )

    try:
        r = git("rev-parse", "--short=7", "HEAD")
        if r.returncode != 0:
            return "dev"
        sha = r.stdout.strip()
        # A dirty tree means the build does not correspond to any commit. Say so
        # in the stamp rather than let it masquerade as a reproducible build.
        dirty = git("status", "--porcelain")
        if dirty.returncode == 0 and dirty.stdout.strip():
            sha += "+"
        return sha
    except Exception:
        return "dev"


project_dir = env.subst("$PROJECT_DIR")          # noqa: F821
engine_dir = abspath(join(project_dir, "..", "proximity_engine"))

stamp = "{}/{}".format(short_sha(project_dir), short_sha(engine_dir))
stamp = stamp[:15]                                # fw_sha[16], NUL-terminated

env.Append(CPPDEFINES=[("PROX_FW_SHA", env.StringifyMacro(stamp))])  # noqa: F821
print("[capture] PROX_FW_SHA = {}".format(stamp))
