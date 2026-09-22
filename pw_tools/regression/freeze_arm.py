"""Copy a build's libxrslam + CMakeCache into a standalone, immutable "arm".

Why this exists (learned the hard way on 2026-08-23):

Two CMake build directories created by copying one another share their
`_deps` sub-builds by absolute path.  Building either one then relinks the
*other* one's artifacts -- `build-v1/xrslam-interface/libxrslam.dylib` changed
sha256 underneath a measurement run that was using it as the control arm.  A
comparison whose control arm mutates between the A run and the B run is not a
comparison.

So: freeze each arm before measuring.  A frozen arm is just

    <dir>/CMakeCache.txt                 (so the harness can read the options)
    <dir>/xrslam-interface/libxrslam.dylib

which is exactly the shape `xrslam_ffi.find_library` expects, and nothing in
the build system knows the path, so nothing can rewrite it.

Usage:
    python3 freeze_arm.py --build ../../build-v1 --out /tmp/arms/thr_off
    python3 freeze_arm.py --build ../../build-v1 --out ... --verify-option \
        XRSLAM_ENABLE_THREADING=OFF
"""

import argparse
import hashlib
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import xrslam_ffi as ffi  # noqa: E402


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def freeze(build, out):
    lib = ffi.find_library(build)
    cache = os.path.join(build, "CMakeCache.txt")
    if not os.path.isfile(cache):
        raise SystemExit("no CMakeCache.txt in %s" % build)
    dst_dir = os.path.join(out, "xrslam-interface")
    os.makedirs(dst_dir, exist_ok=True)
    dst = os.path.join(dst_dir, os.path.basename(lib))
    shutil.copy2(lib, dst)
    shutil.copy2(cache, os.path.join(out, "CMakeCache.txt"))
    return dst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--verify-option", action="append", default=[],
                    metavar="NAME=ON|OFF",
                    help="fail unless the frozen build really has this option; "
                         "checked against the LIBRARY where possible, not just "
                         "the cache text")
    args = ap.parse_args(argv)

    dst = freeze(args.build, args.out)
    slam = ffi.XRSlam(args.out)
    rc, health = slam.get_health()

    print("frozen  %s" % dst)
    print("sha256  %s" % sha256(dst))
    if rc == ffi.XRSLAM_OK:
        # The library's own view. `threading_enabled` is compiled in, so this
        # is a binary-level fact, not a claim made by a text file.
        print("lib says: threading_enabled=%d inspection_compiled_out=%d"
              % (health.threading_enabled, health.inspection_compiled_out))

    bad = []
    for spec in args.verify_option:
        name, _, want = spec.partition("=")
        want_on = want.strip().upper() in ("ON", "TRUE", "1", "YES")
        got = bool(slam.options.get(name, False))
        if name == "XRSLAM_ENABLE_THREADING" and rc == ffi.XRSLAM_OK:
            lib_says = bool(health.threading_enabled)
            if lib_says != got:
                bad.append("%s: cache says %s but the LIBRARY says %s"
                           % (name, got, lib_says))
            got = lib_says
        if got != want_on:
            bad.append("%s: want %s, got %s" % (name, want_on, got))
    for b in bad:
        print("MISMATCH %s" % b)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
