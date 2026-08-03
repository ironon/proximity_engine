#!/usr/bin/env python3
"""Assert that capture_ingest.py's decoder matches the C capture structs.

The wire format is declared twice: once in C (prox_capture.h, what the devices
write) and once in Python (capture_ingest.py, what reads it back). Nothing but
this check stops the two drifting apart — and drift would not fail loudly, it
would silently mis-decode every session recorded afterwards, which is the worst
possible failure for an archive you intend to trust for months.

Compiles a throwaway C++ program that prints sizeof for each struct and compares
against struct.calcsize of each Python format string. Run by `make schema`.
"""

import importlib.util
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")

PROBE = r"""
#include <cstdio>
#include "prox_capture.h"
int main() {
#define P(s) printf("%s %zu\n", #s, sizeof(s));
  P(ProxCapHeader) P(ProxCapSession) P(ProxCapDev) P(ProxCapScore)
  P(ProxCapMotion) P(ProxCapHmm) P(ProxCapEnforce) P(ProxCapAwayGate)
  P(ProxCapSleep) P(ProxCapBeacon) P(ProxCapFpDev)
  return 0;
}
"""


def load_ingest():
    spec = importlib.util.spec_from_file_location(
        "capture_ingest", os.path.join(HERE, "capture_ingest.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ci = load_ingest()

    with tempfile.TemporaryDirectory() as tmp:
        cpp = os.path.join(tmp, "probe.cpp")
        exe = os.path.join(tmp, "probe")
        with open(cpp, "w") as fh:
            fh.write(PROBE)
        r = subprocess.run(
            ["g++", "-std=c++11", "-I", SRC, "-DPROXIMITY_ROLE_ANCHOR",
             "-o", exe, cpp],
            capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stderr, file=sys.stderr)
            print("schema: FAILED to compile the probe", file=sys.stderr)
            return 2
        out = subprocess.run([exe], capture_output=True, text=True).stdout

    c_sizes = {}
    for line in out.strip().splitlines():
        name, size = line.split()
        c_sizes[name] = int(size)

    py_sizes = {
        "ProxCapHeader":   struct.calcsize(ci.HDR),
        "ProxCapSession":  struct.calcsize(ci.TYPES[0x01][1]),
        "ProxCapDev":      ci.DEV_LEN,
        "ProxCapScore":    struct.calcsize(ci.TYPES[0x04][1]),
        "ProxCapMotion":   struct.calcsize(ci.TYPES[0x05][1]),
        "ProxCapHmm":      struct.calcsize(ci.TYPES[0x06][1]),
        "ProxCapEnforce":  struct.calcsize(ci.TYPES[0x07][1]),
        "ProxCapAwayGate": struct.calcsize(ci.TYPES[0x0C][1]),
        "ProxCapSleep":    struct.calcsize(ci.TYPES[0x0B][1]),
        "ProxCapBeacon":   struct.calcsize(ci.TYPES[0x0A][1]),
        "ProxCapFpDev":    ci.FPDEV_LEN,
    }

    bad = []
    for name in sorted(c_sizes):
        if c_sizes[name] != py_sizes.get(name):
            bad.append("  %-18s C=%-3d python=%s"
                       % (name, c_sizes[name], py_sizes.get(name)))

    # Sizes agreeing is necessary but not sufficient — a field reordering keeps
    # the size. Field COUNT is a second cheap check on the same declaration.
    counts = {0x01: 6, 0x04: 13, 0x05: 4, 0x06: 6, 0x07: 4, 0x0A: 3, 0x0B: 2,
              0x0C: 5}
    for tid, want in counts.items():
        got = len(ci.TYPES[tid][2])
        if got != want:
            bad.append("  type 0x%02X (%s): %d fields declared, expected %d"
                       % (tid, ci.TYPES[tid][0], got, want))

    if bad:
        print("schema: MISMATCH between prox_capture.h and capture_ingest.py")
        print("\n".join(bad))
        print("\nEvery session recorded after this point would decode wrong.")
        return 1

    print("schema: %d capture structs match the Python decoder" % len(c_sizes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
