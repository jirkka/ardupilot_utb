#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Expand lossless UTBC v1 packets into legacy field views; never connects to hardware."""
import argparse
import json
from pathlib import Path
import struct

AXIS = struct.Struct("<9fBB")
TAIL = struct.Struct("<10fBBB4fBBBBfIHIB")


def decode_packed(record):
    """Return all seven logical companions; reject truncated or unsupported payloads."""
    words = []
    for name in ("D0", "D1", "D2"):
        block = record[name]
        if len(block) != 32:
            raise ValueError("UTBC array must contain 32 int16 words")
        words.extend(block)
    payload = struct.pack("<96h", *words)
    if payload[-1] != 1:
        raise ValueError(f"Unsupported UTBC version {payload[-1]}")
    common = {key: record[key] for key in ("TimeUS", "Seq", "Md")}
    result = []
    for axis in range(3):
        values = AXIS.unpack_from(payload, axis * AXIS.size)
        rate = dict(zip(("Des", "Meas", "Err", "P", "I", "D", "Raw", "Out", "AP", "Val", "Sat"), values))
        result.append(dict(common, mavpackettype="UTBR", Ax=axis, **rate))
    values = TAIL.unpack_from(payload, 3 * AXIS.size)
    motor = dict(zip(("R", "P", "Y", "T", "M1", "M2", "M3", "M4", "S", "Shift", "Val", "Lo", "Hi"), values[:13]))
    result.append(dict(common, mavpackettype="UTBM", **motor))
    achieved = dict(zip(("R", "P", "Y", "T", "Pos", "Neg", "Scale", "Shift"), values[13:21]))
    result.append(dict(common, mavpackettype="UTBA", **achieved))
    timing = {key: record[key] for key in ("APUS", "APS", "APM", "Cap", "RCms", "Eff")}
    timing.update(Dt=values[21], Ex=values[22], Req=values[23], Max=values[24],
                  Drop=record["Dr"], Miss=record["Ms"])
    result.append(dict(common, mavpackettype="UTBT", **timing))
    result.append(dict(TimeUS=record["TimeUS"], Seq=record["Seq"], Epoch=record["Epoch"], mavpackettype="UBEP"))
    return result


def expand_record(record):
    return decode_packed(record) if record.get("mavpackettype") == "UTBC" else [record]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bin", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="new JSONL file containing logical UTB records")
    args = parser.parse_args()
    from pymavlink import mavutil
    log = mavutil.mavlink_connection(str(args.bin))
    with args.output.open("x") as output:
        while True:
            msg = log.recv_match()
            if msg is None:
                break
            if msg.get_type().startswith(("UTB", "UBEP")):
                for record in expand_record(msg.to_dict()):
                    output.write(json.dumps(record) + "\n")


if __name__ == "__main__":
    main()
