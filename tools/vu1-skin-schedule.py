#!/usr/bin/env python3
"""List-schedule the VU1 skinning blend sketch and count its cycles per corner.

An independent cross-check of the hand count in docs/animated-models.md ("Skinning on
VU1"). It reads the sketch file itself (vendor/tyra/engine/src/renderer/3d/pipeline/
static/core/compact/skin_vu1_blend_sketch.vsm.txt), keeps the joint blocks for k
influences (a line naming joint j is dropped when j >= k), and schedules the operations
on the two VU pipes:

  upper pipe: one FMAC / EFU / ITOF operation per cycle
  lower pipe: one load (LQ, ILW) per cycle, in parallel with the upper pipe

Result latencies (ASSUMPTIONS, not measured here; the header of the docs section says so):
  FMAC, ITOF0 ........................ 4 cycles   (the repo's FMAC latency, 4 cycles)
  RSQRT (EFU, into Q) ................ 12 cycles
  LQ, ILW (loads) .................... 4 cycles   (--load-latency changes it)
A dependent operation waits for its source's latency. Write-after-read and
write-after-write on a register are kept (the ACC chain depends on them).

Two schedules are reported for each k:
  isolated  one corner, list-scheduled alone: the latency-bound cost of one corner;
  steady    three corners of a triangle, each copy with its own registers, list-scheduled
            together (software pipelining): cycles per corner = body cycles / 3.
Instruction words: a scheduled cycle is one 64-bit upper and one 64-bit lower word of
micro memory, i.e. two words; a loop body that runs straight-line is stored whole.

The VF count is the peak number of simultaneously live VF values in the steady schedule
(a value is live from its writer's issue to its last reader's issue); the register file
has 32 VF and the VI integer file 16 (VI0 is zero), so the peak is checked against both.

Run from the TyraX root:  python tools/vu1-skin-schedule.py
"""
import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "vendor/tyra/engine/src/renderer/3d/pipeline/static/core/compact/skin_vu1_blend_sketch.vsm.txt"

UPPER_MNEMONICS = {"mula", "madda", "madd", "mul", "add", "itof0", "rsqrt"}
LOWER_MNEMONICS = {"lq", "ilw"}
ACC = "acc"
JOINT_RE = re.compile(r"(?:_|jo)([0-3])\b")
REG_RE = re.compile(r"^([A-Za-z][A-Za-z0-9_]*)")


def latency(mnemonic, args):
    if mnemonic == "rsqrt":
        return LATENCY["rsqrt"]
    if mnemonic in LOWER_MNEMONICS:
        return LATENCY["load"]
    return LATENCY["fmac"]


LATENCY = {"fmac": 4, "rsqrt": 12, "load": 4}


def base(operand):
    """A register name without its lane suffix, or None for an immediate / offset."""
    text = operand.strip()
    if "(" in text:  # offset(base)
        text = text[text.index("(") + 1:text.rindex(")")]
    m = REG_RE.match(text)
    return m.group(1) if m else None


def parse(path):
    """Instructions of the sketch in program order: (line, mnemonic, dst, srcs, unit, joint)."""
    ops = []
    in_body = False
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split(";", 1)[0].rstrip()
        if line.strip().startswith("cornerLoop:"):
            in_body = True
            continue
        if not in_body or not line.strip():
            continue
        if line.strip().startswith("--barrier") or line.strip().startswith("#endvuprog"):
            break
        parts = line.strip().split(None, 1)
        mnemonic = parts[0].split(".")[0]
        if mnemonic in LOWER_MNEMONICS:
            unit = "L"
        elif mnemonic in UPPER_MNEMONICS:
            unit = "U"
        else:
            continue  # a directive or a non-blend instruction
        operands = [o.strip() for o in parts[1].split(",")] if len(parts) > 1 else []
        if mnemonic == "lq" or mnemonic == "ilw":
            dst = base(operands[0]) if mnemonic == "lq" else base(operands[0])
            srcs = [base(operands[1])]
        else:
            dst = base(operands[0])
            srcs = [base(o) for o in operands[1:]]
        srcs = [s for s in srcs if s]
        if mnemonic in ("madda", "madd"):
            srcs.append(ACC)  # the accumulator is an implicit source
        if mnemonic in ("mula", "madda"):
            dst = ACC  # the accumulator is written by the mula / madda forms
        if mnemonic == "madd":
            pass  # madd writes its destination, reads ACC
        joints = [int(j) for j in JOINT_RE.findall(line)]
        joint = max(joints) if joints else None
        ops.append({"line": line.strip(), "mn": mnemonic, "dst": dst, "srcs": srcs, "unit": unit,
                    "joint": joint, "lat": latency(mnemonic, operands)})
    return ops


def variant(ops, k):
    """Keep the shared ops and the blocks of joints below k."""
    return [op for op in ops if op["joint"] is None or op["joint"] < k]


def rename(ops, copy):
    """Give one corner copy its own registers (the constants stay shared)."""
    out = []
    for op in ops:
        def r(name):
            if name is None or name.startswith("vone") or name == ACC:
                return name
            return f"{name}#{copy}"
        out.append({**op, "dst": r(op["dst"]) if op["dst"] != ACC else ACC,
                    "srcs": [r(s) for s in op["srcs"]], "copy": copy,
                    "acc": copy})  # ACC is per copy as well (renamed below)
    for op in out:
        if op["dst"] == ACC:
            op["dst"] = f"acc#{copy}"
        op["srcs"] = [f"acc#{copy}" if s == ACC else s for s in op["srcs"]]
    return out


def schedule(ops, in_order=False):
    """Greedy list schedule of ops (program order is the priority). Returns cycles and times."""
    ready = {}        # register -> cycle its newest value is available
    last_read = {}    # register -> last cycle a reader issued
    t_issue = [None] * len(ops)
    unit_busy = {}    # (cycle, unit) -> taken
    pending = list(range(len(ops)))
    cycle = 0
    floor = 0
    while pending:
        progressed = False
        for idx in list(pending):
            op = ops[idx]
            if in_order and idx != pending[0]:
                break
            earliest = floor
            for s in op["srcs"]:
                if s in ready:
                    earliest = max(earliest, ready[s])
            if op["dst"]:
                earliest = max(earliest, last_read.get(op["dst"], 0))
                if op["dst"] in ready:
                    earliest = max(earliest, ready[op["dst"]] - op["lat"] + 1)
            if earliest <= cycle and not unit_busy.get((cycle, op["unit"])):
                unit_busy[(cycle, op["unit"])] = True
                t_issue[idx] = cycle
                for s in op["srcs"]:
                    last_read[s] = max(last_read.get(s, 0), cycle)
                if op["dst"]:
                    ready[op["dst"]] = cycle + op["lat"]
                pending.remove(idx)
                progressed = True
                if in_order:
                    floor = cycle
        cycle += 1
        if cycle > 100000:
            raise RuntimeError("schedule did not converge")
    end = max(ready.values()) if ready else 0
    return max(t_issue) + 1 if ops else 0, t_issue, end


def peak_live(ops, times):
    """Peak simultaneously live VF and VI values (ACC and constants excluded)."""
    last = {}
    first = {}
    for op, t in zip(ops, times):
        if op["dst"] and op["dst"] != "acc" and not op["dst"].startswith("acc#"):
            first.setdefault(op["dst"], t)
        for s in op["srcs"]:
            last[s] = max(last.get(s, -1), t)
    vf = []
    vi = []
    for name, start in first.items():
        base_name = name.split("#")[0]
        end = last.get(name, start)
        interval = (start, end)
        (vi if base_name.startswith(("jo", "vc")) else vf).append(interval)
    def peak(intervals):
        best = 0
        for c in range(0, max([e for _, e in intervals] + [0]) + 1):
            live = sum(1 for s, e in intervals if s <= c <= e)
            best = max(best, live)
        return best
    return peak(vf), peak(vi)


def words_of(ops):
    return sum(1 for op in ops)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sketch", default=str(SKETCH))
    parser.add_argument("--load-latency", type=int, default=4)
    parser.add_argument("--fmac-latency", type=int, default=4)
    parser.add_argument("--rsqrt-latency", type=int, default=12)
    args = parser.parse_args()
    LATENCY["load"] = args.load_latency
    LATENCY["fmac"] = args.fmac_latency
    LATENCY["rsqrt"] = args.rsqrt_latency
    ops = parse(Path(args.sketch))
    if not ops:
        print("no blend instructions found after the cornerLoop label", file=sys.stderr)
        return 1
    print(f"sketch: {args.sketch}")
    print(f"latencies: FMAC/ITOF {LATENCY['fmac']}, RSQRT {LATENCY['rsqrt']}, LQ/ILW {LATENCY['load']} cycles")
    print(f"{'k':>2} {'upper':>6} {'lower':>6} {'isolated':>9} {'in-order':>9} {'steady1':>8} "
          f"{'steady2':>8} {'steady3':>8} {'VF pk1':>7} {'VF pk2':>7} {'VF pk3':>7} {'VI pk3':>7} "
          f"{'words3':>7}")
    for k in (1, 2, 3, 4):
        body = variant(ops, k)
        upper = sum(1 for o in body if o["unit"] == "U")
        lower = sum(1 for o in body if o["unit"] == "L")
        iso_cycles, iso_times, _ = schedule(rename(body, 0))
        in_order_cycles, _, _ = schedule(rename(body, 0), in_order=True)
        per = {}
        for copies in (1, 2, 3):
            seq = []
            for copy in range(copies):
                seq += rename(body, copy)
            cycles, times, _ = schedule(seq)
            vf, vi = peak_live(seq, times)
            per[copies] = (cycles / copies, vf, vi, cycles)
        print(f"{k:>2} {upper:>6} {lower:>6} {iso_cycles:>9} {in_order_cycles:>9} {per[1][0]:>8.1f} "
              f"{per[2][0]:>8.1f} {per[3][0]:>8.1f} {per[1][1]:>7} {per[2][1]:>7} {per[3][1]:>7} "
              f"{per[3][2]:>7} {2 * per[3][3]:>7}")
    print("steadyN: N corners of one body list-scheduled together, cycles per corner; VF pkN: peak")
    print("live VF values in that schedule (budget 32); words3: 2 x the three-corner body's cycles.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
