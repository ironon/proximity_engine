#!/usr/bin/env python3
"""Generate the synthetic session that proves the capture pipeline works.

This is NOT field data and must never be read as such — the fw_sha it carries is
literally "synthetic". Its job is to exercise every stage without hardware:

    device wire format -> decode -> label -> replay-export -> replay harness

so that a break anywhere in the chain fails `make corpus` on a laptop, rather
than being discovered the morning after a night of capture that turns out to be
unreadable.

It also happens to demonstrate §13.4-R2 end to end: two frames from the same
spot, the second under 12 dB of uniform attenuation. The replay should score
them identically, with Signal B's legacy value collapsing and its
offset-invariant value holding.

    ./make_smoke_session.py --out ../corpus/synthetic-e2e.jsonl
"""

import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

T0 = 1754300000          # arbitrary but fixed, so the labels below always match
N_DEV = 24


def mac(i):
    return bytes([0xDE, 0xAD, 0xBE, 0x00, (i >> 8) & 0xFF, i & 0xFF])


def room_rssi(i):
    return -45 - ((i * 37) % 45)


def watch_rssi(i):
    return room_rssi(i) - 6 + (((i * 61) % 17) - 8)


class Writer:
    def __init__(self):
        self.seq = 0
        self.buf = b""

    def rec(self, rtype, payload, wall, role=1):
        hdr = struct.pack("<BBBBIIIHH", 0xC5, 1, rtype, role, self.seq,
                          self.seq * 1000, wall, len(payload), 0)
        self.seq += 1
        self.buf += hdr + payload


def devblock(fn):
    b = struct.pack("<H", N_DEV)
    for i in range(N_DEV):
        b += struct.pack("<6sBb", mac(i), 0, fn(i))
    return b


def build():
    w = Writer()
    w.rec(0x01, struct.pack("<16sII16sBBxx", b"\x11" * 16, 1, 0,
                            b"synthetic", 1, 1), T0)

    fp = struct.pack("<H", N_DEV)
    for i in range(N_DEV):
        fp += struct.pack("<6sBxfff", mac(i), 0, float(watch_rssi(i)), 6.0, 120.0)
    w.rec(0x09, fp, T0)

    # EXPECT values measured from the engine on 2026-08-03, not guessed. That
    # makes this session do double duty: it proves the pipeline is intact, AND
    # it is a synthetic regression canary — change the scoring and `make corpus`
    # will report the drift, which is exactly what should happen.
    #
    # The two frames are the R2 argument in miniature. Same spot, second one
    # under 12 dB of uniform attenuation:
    #     clear    score 234, cm  +0.0, Lb 0.90, Li 0.90
    #     blanket  score 234, cm -12.0, Lb 0.00, Li 0.90   <- legacy collapses
    frames = [
        (T0 + 100, lambda i: watch_rssi(i),      -80, 0x01,
         234,   0.0, 0.0, 2.4, 0.90, 0.90),
        (T0 + 200, lambda i: watch_rssi(i) - 12, -92, 0x09,
         234, -12.0, 0.0, 2.4, 0.00, 0.90),
    ]
    for (wall, fn, self_rssi, flags, score,
         cm, mad, sig, lb, li) in frames:
        w.rec(0x03, devblock(room_rssi), wall)
        w.rec(0x02, devblock(fn), wall)
        w.rec(0x04, struct.pack("<BBBbhHHHfffff", score, flags, 188, self_rssi,
                                48, N_DEV, N_DEV, N_DEV,
                                cm, mad, sig, lb, li), wall)
    return w.buf


LABELS = """\
# <from> <to> <label>. [near]/[away] is what the replay harness scores against;
# everything after it is for the human reading this in six months.
2025-08-04T09:33:00Z 2025-08-04T09:35:30Z  [near] bedroom, clear line of sight
2025-08-04T09:35:30Z 2025-08-04T09:38:00Z  [near] same spot, 12 dB uniform attenuation (duvet)
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    ingest = os.path.join(HERE, "capture_ingest.py")
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        binf = os.path.join(tmp, "s.bin")
        labf = os.path.join(tmp, "s.labels")
        rawf = os.path.join(tmp, "s.jsonl")
        with open(binf, "wb") as fh:
            fh.write(build())
        with open(labf, "w") as fh:
            fh.write(LABELS)
        subprocess.run([sys.executable, ingest, "decode", binf, "--out", rawf],
                       check=True)
        subprocess.run([sys.executable, ingest, "label", rawf,
                        "--from", labf, "--out", args.out], check=True)

    print("wrote %s" % args.out, file=sys.stderr)


if __name__ == "__main__":
    main()
