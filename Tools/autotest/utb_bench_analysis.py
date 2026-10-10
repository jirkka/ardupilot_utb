#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline completeness and persisted-rate analysis of downloaded benchmark BIN logs."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from pymavlink import mavutil


def analyse(records, start_us=None, end_us=None):
    groups = defaultdict(lambda: defaultdict(list))
    epochs = defaultdict(list)
    for rec in records:
        name = rec.get("mavpackettype")
        if name in {"UTBR", "UTBM", "UTBA", "UTBT", "UBEP"}:
            groups[(int(rec["TimeUS"]), int(rec["Seq"]))][name].append(rec)
    incomplete = 0
    for (stamp, seq), group in groups.items():
        if start_us is not None and stamp < start_us or end_us is not None and stamp >= end_us:
            continue
        rates = group["UTBR"]
        complete = (len(rates) == 3 and sorted(int(r["Ax"]) for r in rates) == [0, 1, 2]
                    and all(len(group[n]) == 1 for n in ["UTBM", "UTBA", "UTBT", "UBEP"]))
        if not complete:
            incomplete += 1
            continue
        epochs[int(group["UBEP"][0]["Epoch"])].append((stamp, seq))
    result = {"incomplete_groups": incomplete, "epochs": {}, "persisted_rate_hz": None,
              "complete_sets": sum(len(v) for v in epochs.values())}
    for epoch, samples in epochs.items():
        samples.sort()
        elapsed = (samples[-1][0] - samples[0][0]) / 1e6
        result["epochs"][str(epoch)] = {
            "complete_sets": len(samples), "first_us": samples[0][0], "last_us": samples[-1][0],
            "interior_rate_hz": (len(samples) - 1) / elapsed if elapsed > 0 else None,
        }
    if start_us is not None and end_us is not None:
        if end_us <= start_us:
            raise ValueError("end must exceed start")
        result["persisted_rate_hz"] = result["complete_sets"] * 1e6 / (end_us - start_us)
        result["window_us"] = [start_us, end_us]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bin", type=Path)
    parser.add_argument("--start-us", type=int)
    parser.add_argument("--end-us", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if (args.start_us is None) != (args.end_us is None):
        parser.error("provide both window endpoints")
    log = mavutil.mavlink_connection(str(args.bin))
    records = []
    while True:
        msg = log.recv_match()
        if msg is None:
            break
        records.append(msg.to_dict())
    result = analyse(records, args.start_us, args.end_us)
    result["notes"] = ("Interior rate excludes unrecorded leading/trailing interval; supply full window for bench throughput. "
                       "Flash-overwritten intervals without records cannot be reconstructed. Zero confirmed collisions "
                       "does not exclude blocking. Successful backend writes are not persistence evidence.")
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + "\n")


if __name__ == "__main__":
    main()
