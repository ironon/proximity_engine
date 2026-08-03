#!/usr/bin/env python3
"""Turn a live capture stream into a replayable corpus session.

    ingest      listen on UDP and write a JSONL session          (normal use)
    decode      convert a raw binary dump to JSONL               (serial capture)
    label       apply a sidecar label file to an existing session
    summary     what is in a session, and what is missing from it

The wire format is defined by proximity_engine/src/prox_capture.h. The struct
formats below MUST match it; `--selfcheck` asserts the sizes it can, and the
record types are append-only on the device side for exactly this reason.

Why JSONL rather than a database: the corpus is the input to the replay harness
and it wants to be greppable, diffable, and readable in six months by someone
who no longer remembers any of this. One line per record, one file per session.

    # capture a night
    ./capture_ingest.py ingest --out corpus/2026-08-04-sunrise.jsonl

    # say what was true, afterwards
    ./capture_ingest.py label corpus/2026-08-04-sunrise.jsonl --from labels.txt
"""

import argparse
import json
import os
import socket
import struct
import sys
import time
from datetime import datetime, timezone

MAGIC = 0xC5
VERSION = 1
DEFAULT_PORT = 49001

HDR = "<BBBBIIIHH"
HDR_LEN = struct.calcsize(HDR)
assert HDR_LEN == 20, HDR_LEN

ROLES = {0: "watch", 1: "anchor"}

# type id -> (name, struct format for the fixed head, field names)
# APPEND ONLY, mirroring prox_capture.h. A renumber here silently reinterprets
# every corpus file ever written.
TYPES = {
    0x01: ("session", "<16sII16sBBxx",
           ["device_uuid", "boot_count", "engine_cfg_hash", "fw_sha",
            "v2_authoritative", "r2_offset_invariant"]),
    0x02: ("vector", "<H", ["count"]),
    0x03: ("anchor_cache", "<H", ["count"]),
    0x04: ("score", "<BBBbhHHHfffff",
           ["score", "flags", "near_thr", "self_rssi", "self_delta",
            "shared", "fp_seen", "vec_count",
            "cm_delta", "cm_mad", "cm_sigma", "L_legacy", "L_invariant"]),
    0x05: ("motion", "<IBBBx", ["burst_var", "cadence", "state", "ints"]),
    0x06: ("hmm", "<iiBBBB",
           ["lambda_q8", "emit_q8", "decision", "motion_state", "neff", "score"]),
    0x07: ("enforce", "<BBBx16s",
           ["event", "criteria", "condition_met", "event_uuid"]),
    0x08: ("label", None, None),        # payload is raw UTF-8 text
    0x09: ("fingerprint", "<H", ["count"]),
    0x0A: ("beacon", "<6sHbxxx", ["mac", "minor", "rssi"]),
    0x0B: ("sleep", "<IBxxx", ["slept_ms", "motion_woke"]),
    0x0C: ("awaygate", "<BBBBI",
           ["armed", "hits", "admits", "motion_state", "still_for_s"]),
}

DEV = "<6sBb"                  # ProxCapDev
DEV_LEN = struct.calcsize(DEV)
FPDEV = "<6sBxfff"             # ProxCapFpDev
FPDEV_LEN = struct.calcsize(FPDEV)

MOTION_STATES = {0: "still", 1: "fidget", 2: "locomotion", 3: "unknown"}
DECISIONS = {0: "near", 1: "away", 2: "ambiguous"}
ENF_EVENTS = {0: "window_open", 1: "window_close", 2: "met", 3: "unmet"}
CRITERIA = {0: "getAway", 1: "stayNear", 2: "getOffWifi", 3: "getOnWifi",
            4: "phoneAway"}


def _mac(b):
    return ":".join("%02X" % x for x in b)


def _cstr(b):
    return b.split(b"\0", 1)[0].decode("utf-8", "replace")


def decode_record(buf):
    """One framed record -> dict, or None if the frame is not usable."""
    if len(buf) < HDR_LEN:
        return None
    magic, ver, rtype, role, seq, t_ms, t_wall, plen, _ = struct.unpack(
        HDR, buf[:HDR_LEN])
    if magic != MAGIC:
        return None
    if ver != VERSION:
        return {"type": "unsupported_version", "ver": ver, "seq": seq}
    payload = buf[HDR_LEN:HDR_LEN + plen]
    if len(payload) < plen:
        return None

    name, fmt, fields = TYPES.get(rtype, (None, None, None))
    rec = {
        "seq": seq,
        "role": ROLES.get(role, role),
        "t_ms": t_ms,
        "type": name or ("unknown_0x%02X" % rtype),
    }
    if t_wall:
        rec["t_wall"] = t_wall
        rec["t_iso"] = datetime.fromtimestamp(
            t_wall, timezone.utc).isoformat().replace("+00:00", "Z")

    if name is None:
        rec["raw"] = payload.hex()
        return rec

    if name == "label":
        rec["text"] = payload.decode("utf-8", "replace")
        return rec

    head_len = struct.calcsize(fmt)
    if len(payload) < head_len:
        rec["error"] = "short payload"
        return rec
    vals = struct.unpack(fmt, payload[:head_len])
    for k, v in zip(fields, vals):
        if k in ("device_uuid",):
            rec[k] = v.hex()
        elif k == "event_uuid":
            rec[k] = v.hex()
        elif k == "fw_sha":
            rec[k] = _cstr(v)
        elif k == "mac":
            rec[k] = _mac(v)
        else:
            rec[k] = v

    body = payload[head_len:]
    if name in ("vector", "anchor_cache"):
        devs = []
        for i in range(0, min(len(body), rec["count"] * DEV_LEN), DEV_LEN):
            mac, dtype, rssi = struct.unpack(DEV, body[i:i + DEV_LEN])
            devs.append([_mac(mac), dtype, rssi])
        rec["devices"] = devs           # [mac, type, rssi]
    elif name == "fingerprint":
        devs = []
        for i in range(0, min(len(body), rec["count"] * FPDEV_LEN), FPDEV_LEN):
            mac, dtype, mu, var, W = struct.unpack(FPDEV, body[i:i + FPDEV_LEN])
            devs.append([_mac(mac), dtype, round(mu, 2), round(var, 3),
                         round(W, 2)])
        rec["devices"] = devs           # [mac, type, mu, var, W]

    # Human-readable mirrors. The numeric field stays authoritative; these exist
    # so a session can be read with `grep` without a decoder ring.
    if "motion_state" in rec:
        rec["motion"] = MOTION_STATES.get(rec["motion_state"])
    if name == "motion":
        rec["motion"] = MOTION_STATES.get(rec["state"])
    if name == "hmm":
        rec["decision_name"] = DECISIONS.get(rec["decision"])
    if name == "enforce":
        rec["event_name"] = ENF_EVENTS.get(rec["event"])
        rec["criteria_name"] = CRITERIA.get(rec["criteria"])
    return rec


class GapTracker:
    """Sequence numbers advance on every ATTEMPTED emit, so a hole in them is
    proof of loss — whether the device dropped the record for want of buffer or
    the network dropped the datagram. Either way the corpus says so explicitly.
    A silently incomplete session is worse than an obviously incomplete one,
    because it will be trusted."""

    def __init__(self):
        self.last = {}
        self.total_missing = 0

    def check(self, rec):
        role, seq = rec.get("role"), rec.get("seq")
        if role is None or seq is None:
            return None
        prev = self.last.get(role)
        self.last[role] = seq
        if prev is None or seq <= prev:
            return None                 # first record, or a device reboot
        missing = seq - prev - 1
        if missing <= 0:
            return None
        self.total_missing += missing
        return {"type": "gap", "role": role, "missing": missing,
                "after_seq": prev, "before_seq": seq}


def cmd_ingest(args):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.settimeout(1.0)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    gaps = GapTracker()
    counts = {}
    n = 0
    started = time.time()
    print("listening on udp/%d -> %s   (ctrl-C to stop)" % (args.port, args.out),
          file=sys.stderr)

    with open(args.out, "a", buffering=1) as fh:
        # A run marker, so a session file that was appended to across several
        # listening sessions can still be cut apart later.
        fh.write(json.dumps({
            "type": "ingest_start",
            "t_iso": datetime.now(timezone.utc).isoformat(),
            "port": args.port,
        }) + "\n")
        try:
            while True:
                try:
                    data, addr = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                rec = decode_record(data)
                if rec is None:
                    continue
                rec["src"] = addr[0]
                gap = gaps.check(rec)
                if gap:
                    fh.write(json.dumps(gap) + "\n")
                    print("  ! gap: %s lost %d record(s)"
                          % (gap["role"], gap["missing"]), file=sys.stderr)
                fh.write(json.dumps(rec) + "\n")
                counts[rec["type"]] = counts.get(rec["type"], 0) + 1
                n += 1
                if args.verbose:
                    print(_oneline(rec), file=sys.stderr)
                elif n % 25 == 0:
                    print("\r  %d records, %d lost" % (n, gaps.total_missing),
                          end="", file=sys.stderr)
        except KeyboardInterrupt:
            pass

    dur = time.time() - started
    print("\n%d records in %.0fs, %d lost" % (n, dur, gaps.total_missing),
          file=sys.stderr)
    for k in sorted(counts):
        print("  %-14s %d" % (k, counts[k]), file=sys.stderr)
    if gaps.total_missing:
        print("NOTE: gaps are recorded in the session as `gap` lines.",
              file=sys.stderr)


def _oneline(rec):
    t = rec.get("t_iso", str(rec.get("t_ms")))
    r = rec.get("role", "?")
    ty = rec.get("type")
    if ty == "score":
        return ("%s %-6s score=%3d cm=%+.1f mad=%.1f sig=%.1f Lb=%.2f Li=%.2f "
                "n=%d" % (t, r, rec["score"], rec["cm_delta"], rec["cm_mad"],
                          rec["cm_sigma"], rec["L_legacy"], rec["L_invariant"],
                          rec["vec_count"]))
    if ty == "vector":
        return "%s %-6s vector n=%d" % (t, r, rec["count"])
    if ty == "awaygate":
        return ("%s %-6s gate armed=%d hits=%d admits=%d still=%ds"
                % (t, r, rec["armed"], rec["hits"], rec["admits"],
                   rec["still_for_s"]))
    if ty == "enforce":
        return "%s %-6s %s %s" % (t, r, rec.get("event_name"),
                                  rec.get("criteria_name"))
    if ty == "label":
        return "%s %-6s LABEL %s" % (t, r, rec["text"])
    return "%s %-6s %s" % (t, r, ty)


def cmd_decode(args):
    """Recover records from a raw byte dump — a serial log, or a pcap payload
    concatenation. Resyncs on the magic byte, so leading junk and interleaved
    human-readable serial output are both survivable."""
    data = open(args.input, "rb").read()
    gaps = GapTracker()
    n = 0
    i = 0
    with open(args.out, "w", buffering=1) as fh:
        while i < len(data) - HDR_LEN:
            if data[i] != MAGIC:
                i += 1
                continue
            plen = struct.unpack("<H", data[i + 16:i + 18])[0]
            total = HDR_LEN + plen
            rec = decode_record(data[i:i + total])
            if rec is None:
                i += 1
                continue
            gap = gaps.check(rec)
            if gap:
                fh.write(json.dumps(gap) + "\n")
            fh.write(json.dumps(rec) + "\n")
            n += 1
            i += total
    print("%d records -> %s (%d lost)" % (n, args.out, gaps.total_missing),
          file=sys.stderr)


def cmd_label(args):
    """Apply labels after the fact.

    Live labels are fine when you are awake and probing deliberately, but the
    runs that matter most happen while you are asleep — so the corpus has to
    accept ground truth written the next morning. Sidecar format, one per line:

        2026-08-04T06:12:00Z 2026-08-04T06:18:00Z  in bed, duvet over wrist
        2026-08-04T06:18:00Z 2026-08-04T06:25:00Z  kitchen, awake

    Ranges are half-open [from, to) and are matched against t_wall.
    """
    spans = []
    for ln, line in enumerate(open(args.source), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 2)
        if len(parts) < 3:
            print("skipping line %d: need <from> <to> <label>" % ln,
                  file=sys.stderr)
            continue
        try:
            a = int(datetime.fromisoformat(
                parts[0].replace("Z", "+00:00")).timestamp())
            b = int(datetime.fromisoformat(
                parts[1].replace("Z", "+00:00")).timestamp())
        except ValueError as e:
            print("skipping line %d: %s" % (ln, e), file=sys.stderr)
            continue
        spans.append((a, b, parts[2]))

    out = args.out or (args.session + ".labelled")
    applied = unlabelled = 0
    with open(args.session) as fin, open(out, "w") as fh:
        for line in fin:
            rec = json.loads(line)
            tw = rec.get("t_wall")
            if tw:
                hits = [s[2] for s in spans if s[0] <= tw < s[1]]
                if hits:
                    rec["labels"] = hits
                    applied += 1
                else:
                    unlabelled += 1
            fh.write(json.dumps(rec) + "\n")
    print("%d records labelled, %d left unlabelled -> %s"
          % (applied, unlabelled, out), file=sys.stderr)
    if unlabelled:
        print("NOTE: unlabelled records are kept but the harness will not score "
              "them — a session is only as good as its ground truth.",
              file=sys.stderr)


def cmd_replay_export(args):
    """JSONL -> the line-oriented format the C++ replay harness reads.

    A derived artifact, regenerable at any time: JSONL stays the archive and the
    source of truth, because it is what a human reads in six months. This exists
    only because parsing JSON in the C++11 test harness would be more code, and
    more risk, than the corpus is worth.

    One frame per scored query: the fingerprint state, the anchor's cache, the
    watch's vector, and what the engine said at the time. Replay re-derives the
    verdict from the first three and checks it against the fourth.
    """
    fp = None                  # most recent fingerprint snapshot
    cache = None               # most recent anchor cache
    vec = None                 # most recent vector
    labels = []
    frames = 0
    out = open(args.out, "w") if args.out else sys.stdout
    try:
        for line in open(args.session):
            rec = json.loads(line)
            t = rec.get("type")
            if t == "session":
                out.write("SESSION fw=%s cfg=%08x v2=%d r2=%d\n" % (
                    rec.get("fw_sha", "?"), rec.get("engine_cfg_hash", 0),
                    rec.get("v2_authoritative", -1),
                    rec.get("r2_offset_invariant", -1)))
            elif t == "fingerprint":
                fp = rec["devices"]
            elif t == "anchor_cache":
                cache = rec["devices"]
            elif t == "vector":
                vec = rec["devices"]
                labels = rec.get("labels", [])
            elif t == "score":
                # A score without its inputs cannot be replayed. Skipping is
                # correct and must be visible, not silent.
                if vec is None or cache is None:
                    out.write("SKIP no-inputs seq=%d\n" % rec.get("seq", -1))
                    continue
                out.write("FRAME t=%d\n" % rec.get("t_wall", rec.get("t_ms", 0)))
                for lab in labels:
                    out.write("LABEL %s\n" % lab)
                if fp:
                    out.write("FP %d\n" % len(fp))
                    for mac, dtype, mu, var, W in fp:
                        out.write("  %s %d %.4f %.4f %.4f\n"
                                  % (mac, dtype, mu, var, W))
                out.write("CACHE %d\n" % len(cache))
                for mac, dtype, rssi in cache:
                    out.write("  %s %d %d\n" % (mac, dtype, rssi))
                out.write("VEC %d\n" % len(vec))
                for mac, dtype, rssi in vec:
                    out.write("  %s %d %d\n" % (mac, dtype, rssi))
                out.write("EXPECT %d %d %.4f %.4f %.4f %.4f %.4f\n" % (
                    rec["score"], rec["flags"], rec["cm_delta"], rec["cm_mad"],
                    rec["cm_sigma"], rec["L_legacy"], rec["L_invariant"]))
                frames += 1
                vec = None      # one frame per vector; do not reuse it
    finally:
        if args.out:
            out.close()
    print("%d frames -> %s" % (frames, args.out or "stdout"), file=sys.stderr)


def cmd_summary(args):
    counts, roles, labels = {}, {}, {}
    first = last = None
    missing = 0
    sessions = []
    for line in open(args.session):
        rec = json.loads(line)
        t = rec.get("type")
        counts[t] = counts.get(t, 0) + 1
        if t == "gap":
            missing += rec.get("missing", 0)
            continue
        if t == "session":
            sessions.append(rec)
        roles[rec.get("role")] = roles.get(rec.get("role"), 0) + 1
        for lab in rec.get("labels", []):
            labels[lab] = labels.get(lab, 0) + 1
        tw = rec.get("t_wall")
        if tw:
            first = tw if first is None else min(first, tw)
            last = tw if last is None else max(last, tw)

    print("session: %s" % args.session)
    if first:
        print("  span:     %s .. %s (%.1f h)" % (
            datetime.fromtimestamp(first, timezone.utc).isoformat(),
            datetime.fromtimestamp(last, timezone.utc).isoformat(),
            (last - first) / 3600.0))
    for s in sessions:
        print("  build:    role=%s fw=%s cfg=%08x v2=%d r2=%d"
              % (s.get("role"), s.get("fw_sha"), s.get("engine_cfg_hash", 0),
                 s.get("v2_authoritative", -1), s.get("r2_offset_invariant", -1)))
    print("  records:  %d total, %d lost to gaps"
          % (sum(counts.values()), missing))
    for k in sorted(counts):
        print("    %-16s %d" % (k, counts[k]))
    print("  labels:   %s" % (", ".join("%s (%d)" % (k, v)
                                        for k, v in sorted(labels.items()))
                              or "NONE — this session cannot be scored"))

    # The checks that decide whether a session is replayable at all.
    problems = []
    if not sessions:
        problems.append("no `session` record: provenance unknown, do not pool "
                        "this with other captures")
    if counts.get("vector", 0) == 0:
        problems.append("no `vector` records: nothing to replay")
    if counts.get("fingerprint", 0) == 0:
        problems.append("no `fingerprint` record: replay has no initial state")
    if len({s.get("engine_cfg_hash") for s in sessions}) > 1:
        problems.append("multiple engine configurations in one session")
    if not labels:
        problems.append("no labels: run `label` before scoring")
    if problems:
        print("  PROBLEMS:")
        for p in problems:
            print("    - %s" % p)
    else:
        print("  ready to replay")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("ingest", help="listen on UDP, write a JSONL session")
    p.add_argument("--out", required=True)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("-v", "--verbose", action="store_true",
                   help="print every record as it arrives")
    p.set_defaults(fn=cmd_ingest)

    p = sub.add_parser("decode", help="convert a raw binary dump to JSONL")
    p.add_argument("input")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_decode)

    p = sub.add_parser("label", help="apply a sidecar label file")
    p.add_argument("session")
    p.add_argument("--from", dest="source", required=True)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_label)

    p = sub.add_parser("replay-export",
                       help="JSONL -> the replay harness's input format")
    p.add_argument("session")
    p.add_argument("--out")
    p.set_defaults(fn=cmd_replay_export)

    p = sub.add_parser("summary", help="what is in a session, and what is not")
    p.add_argument("session")
    p.set_defaults(fn=cmd_summary)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
