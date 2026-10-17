#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Functional benchmark SITL regression; no hardware connection or H7 timing claim."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_bench.py
# @brief      BENCH metriky, thread failure a functional SITL regrese; nikoli H743 timing.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import json
from pathlib import Path
import tempfile
import time
from pymavlink import mavutil
from test_utb_skeleton import SITL
from utb_bench_analysis import analyse


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Collect.
#
# @param[in]  sim Owned local simulator connection.
# @param[in]  seconds Seconds.
# @return     Returned values: values.
#
# @details    Global module function. Calls: time.monotonic, sim.rc, sim.link.recv_match, msg.get_type.
# --------------------------------------------------------------------------------------------------------------------
def collect(sim, seconds):
    end = time.monotonic() + seconds
    values = {}
    while time.monotonic() < end:
        sim.rc()
        msg = sim.link.recv_match(type=["NAMED_VALUE_INT", "HEARTBEAT"], blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "HEARTBEAT":
            assert not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        elif msg.name.startswith("UB"):
            values[int(msg.name[2:])] = int(msg.value) & 0xffffffff
    return values


# --------------------------------------------------------------------------------------------------------------------
# @brief      Scenario.
#
# @param[in]  binary Explicit local SITL executable.
# @param[in]  root Root.
# @param[in]  output Explicit destination for new experiment results.
# @param[in]  name Name.
# @param[in]  enabled Enabled.
# @param[in]  extra Extra.
# @return     Returned values: result.
#
# @details    Global module function. Calls: tempfile.TemporaryDirectory, Path, sorted, any.
# --------------------------------------------------------------------------------------------------------------------
def scenario(binary, root, output, name, enabled, extra):
    with tempfile.TemporaryDirectory(prefix="utb-bench-") as temp:
        work = Path(temp)
        with (output / (name + ".log")).open("w") as console:
            sim = SITL(binary, root / "Tools/autotest/default_params/copter.parm", work, console, 1, enabled,
                       extra_params="FRAME_TYPE 12\nLOG_BACKEND_TYPE 1\nLOG_DISARMED 1\n" + extra,
                       model="bfx", speedup=1)
            try:
                values = collect(sim, 12)
                assert values[0] == (1 if not enabled else 3 if "UTB_SHADOW 1" in extra else 2), values
                assert values[34] == (1 if enabled else 0), values
                assert values[35] == (1 if "UTB_B_FAIL 1" in extra else 0), values
                assert values[36] == 1, values
                assert values[18] == 3, values  # independent exporter RUNNING
                if not enabled:
                    assert values[10] == 0 and values[11] == 0, values
                elif "UTB_B_FAIL 1" in extra:
                    assert values[10] == 2 and values[11] == 0, values
                else:
                    assert values[10] == 3 and values[11] > 0, values
                    before = values[11]
                    sim.param("LOG_DISARMED", 0)
                    unavailable = collect(sim, 4)
                    assert unavailable[11] > before and unavailable[36] == 0, unavailable
                    sim.param("LOG_DISARMED", 1)
                    recovered = collect(sim, 4)
                    assert recovered[11] > unavailable[11] and recovered[36] == 1, recovered
                    if "UTB_SHADOW 1" in extra:
                        sim.param("UTB_LOG_RATE", 400)
                        fast = collect(sim, 5)
                        assert fast[0] == 4 and fast[2] == 400, fast
                        values["400"] = fast
                sim.param("UTB_SHADOW", 0)
                collect(sim, 2)
            finally:
                sim.close()
        records = []
        for path in sorted((work / "logs").glob("*.BIN")):
            (output / (name + "-" + path.name)).write_bytes(path.read_bytes())
            log = mavutil.mavlink_connection(str(path))
            while True:
                msg = log.recv_match()
                if msg is None:
                    break
                records.append(msg.to_dict())
        assert any(x.get("mavpackettype") == "UBST" for x in records), name
        assert any(x.get("mavpackettype") == "UBMT" and x["Met"] == 0 and x["N"] > 0 for x in records), name
        result = {"telemetry": values, "persisted": analyse(records)}
        if "UTB_SHADOW 1" in extra and "UTB_B_FAIL 1" not in extra:
            assert result["persisted"]["complete_sets"] > 0, result
        return result


# --------------------------------------------------------------------------------------------------------------------
# @brief      Analysis regression.
#
# @param[in]  None
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: analyse, records.extend, range.
# --------------------------------------------------------------------------------------------------------------------
def analysis_regression():
    records = []
    for seq, stamp, epoch in [(1, 100000, 1), (2, 105000, 1), (3, 500000, 2)]:
        records.extend({"mavpackettype": "UTBR", "TimeUS": stamp, "Seq": seq, "Ax": ax} for ax in range(3))
        records.extend({"mavpackettype": name, "TimeUS": stamp, "Seq": seq, "Epoch": epoch}
                       for name in ["UTBM", "UTBA", "UTBT", "UBEP"])
    result = analyse(records, 0, 1000000)
    assert result["complete_sets"] == 3 and result["persisted_rate_hz"] == 3
    assert result["epochs"]["1"]["interior_rate_hz"] == 200 and result["epochs"]["2"]["complete_sets"] == 1
    assert analyse([r for r in records if r["mavpackettype"] != "UBEP"])["complete_sets"] == 0
    duplicate = records + [records[0]]
    assert analyse(duplicate)["complete_sets"] == 2 and analyse(duplicate)["incomplete_groups"] == 1
    return {name: "PASS" for name in ["separate_epochs", "full_window_rate", "missing_companion", "duplicate_record"]}


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args,
#             args.output.mkdir.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    args.output.mkdir(parents=True, exist_ok=True)
    results = {"analysis": analysis_regression()}
    for name, enabled, extra in [("A", 0, ""), ("B", 1, ""),
                                 ("C-D", 1, "UTB_SHADOW 1\nUTB_LOG_RATE 200\n"),
                                 ("failure", 1, "UTB_B_FAIL 1\n")]:
        results[name] = scenario(args.binary.resolve(), root, args.output.resolve(), name, enabled, extra)
        (args.output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        print("PASS", name, flush=True)


if __name__ == "__main__":
    main()

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Data type, constant, macro definition
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private function prototypes
# ====================================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private functions
