#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""BF_X_REV regression on an owned local SITL, never on a physical controller."""

import argparse
import json
import math
from pathlib import Path

from pymavlink import mavutil

from test_utb_shadow import BASE, assemble_sets, launch, source_audit


COLUMNS = ((-0.5, -0.5, 0.5, 0.5), (-0.5, 0.5, -0.5, 0.5), (0.5, -0.5, -0.5, 0.5))


def check_math(records):
    complete = [v for v in assemble_sets(records).values() if len(v) == 6 and v["UTBM"]["Val"]]
    assert len(complete) > 500, len(complete)
    maximum = 0.0
    for sample in complete:
        mixer = sample["UTBM"]
        # Recover demand from PID records, independently of logged mixer commands.
        command = [max(-1, min(1, sample[f"R{i}"]["Raw"])) for i in range(3)]
        motors = [mixer[f"M{i}"] for i in range(1, 5)]
        assert all(math.isfinite(v) and 0 <= v <= 1 for v in motors)
        for motor in range(4):
            expected = mixer["T"] + mixer["Shift"] + mixer["S"] * sum(
                COLUMNS[axis][motor] * command[axis] for axis in range(3))
            maximum = max(maximum, abs(motors[motor] - expected))
        for axis, name in enumerate(("R", "P", "Y")):
            inverse = sum(COLUMNS[axis][motor] * motors[motor] for motor in range(4))
            assert abs(sample["UTBA"][name] - inverse) < 1e-6
            assert abs(inverse - mixer["S"] * command[axis]) < 1e-6
            rate = sample[f"R{axis}"]
            assert abs(rate["P"] - 0.1 * (rate["Des"] - rate["Meas"])) < 1e-6
        assert abs(sum(motors) / 4 - sample["UTBA"]["T"]) < 1e-6
    assert maximum < 1e-6, maximum
    assert any(abs(v["R2"]["Raw"]) > 0.01 for v in complete), "No yaw excitation"
    assert all(s["Calc"] != 6 for s in records["UTBS"]), "Unexpected FRAME_MISMATCH for type18"
    rates = {}
    for rate in (200, 400):
        segment = sorted((v["UTBT"] for v in complete if v["UTBT"]["Req"] == rate), key=lambda m: m["TimeUS"])
        assert len(segment) > 400, (rate, len(segment))
        observed = (len(segment) - 1) * 1e6 / (segment[-1]["TimeUS"] - segment[0]["TimeUS"])
        assert abs(observed - rate) < rate * 0.04, (rate, observed)
        rates[rate] = {"complete_sets": len(segment), "interior_hz": observed}
    # Independent static RC map with the explicit calibration below.
    expected = (17 * math.pi / 47, -7 * math.pi / 47, 7 * math.pi / 94)
    for axis in range(3):
        assert max(abs(v[f"R{axis}"]["Des"] - expected[axis]) for v in complete[-100:]) < 1e-6
    return {"valid_sets": len(complete), "allocation_max_error": maximum, "rates": rates,
            "reference_and_pid_nonzero": True, "hardware_rate_claimed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True, exist_ok=True)
    base = BASE.replace("FRAME_TYPE 12", "FRAME_TYPE 18")
    base += "UTB_RAT_RLL_I 0\nUTB_RAT_RLL_D 0\nLOG_BACKEND_TYPE 1\nFLTMODE1 1\n"
    base += "ACRO_RP_RATE 180\nACRO_Y_RATE 90\nACRO_RP_EXPO 0\nACRO_Y_EXPO 0\n"
    for channel in (1, 2, 3, 4):
        base += (f"RC{channel}_MIN 1000\nRC{channel}_MAX 2000\nRC{channel}_TRIM 1500\n"
                 f"RC{channel}_DZ 30\nRC{channel}_REVERSED 0\n")
    measured = {}
    for shadow in (0, 1):
        def action(sim):
            sim.roll, sim.pitch, sim.yaw, sim.throttle = 1700, 1400, 1600, 1500
            sim.duration(3)  # allow startup RC mode selection to settle before the GCS request
            sim.mode(1, 1)
            if shadow:
                sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
            sim.duration(6)
            outputs = []
            for _ in range(20):
                msg = sim.wait("SERVO_OUTPUT_RAW")
                outputs.append(tuple(getattr(msg, f"servo{i}_raw") for i in range(1, 5)))
            # No successful arm is attempted: these requests test the UTB_ACRO prohibition only.
            sim.param("UTB_LOG_RATE", 400)
            sim.duration(6)
            sim.mode(40, 40)
            for force in (False, True):
                assert sim.arm(force=force) != mavutil.mavlink.MAV_RESULT_ACCEPTED
                sim.observe(40, False)
            sim.param("RC7_OPTION", 153)
            sim.rc7 = 2000
            sim.observe(40, False)
            sim.rc7 = 1000
            sim.param("RC7_OPTION", 0)
            sim.mode(1, 1)
            measured[shadow] = outputs
        records, _ = launch(args.binary.resolve(), root, args.output.resolve(), f"bfxrev-{shadow}",
                            base + f"UTB_SHADOW {shadow}\n", action, model="bfxrev")
        assert records.get("RCOU"), "Missing physical AP output diagnostics"
        if shadow:
            result = check_math(records)
    assert measured[0] == measured[1], measured
    result["motor_isolation"] = {"mode": "ACRO DISARMED", "samples_per_run": 20,
                                 "steady_output_differences": 0, "flight_equivalence_claimed": False}
    result["arming_guard"] = "ordinary/forced/RC rejected in UTB_ACRO; simulator stays DISARMED"

    def transition(sim):
        sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
        sim.duration(2)
        for frame in (12, 18):
            sim.param("FRAME_TYPE", frame)
            sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
            sim.duration(2)
    changed, _ = launch(args.binary.resolve(), root, args.output.resolve(), "geometry-transition",
                        base + "UTB_SHADOW 1\n", transition, model="bfxrev")
    assert all(status["Calc"] != 6 for status in changed["UTBS"])
    epochs = {sample["Epoch"] for sample in changed.get("UBEP", [])}
    assert len(epochs) >= 3, epochs
    result["geometry_transition"] = {"epochs": sorted(epochs), "healthy_recovery": True}
    result["source_audit"] = source_audit(root)
    (args.output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
