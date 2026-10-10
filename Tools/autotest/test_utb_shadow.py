#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Local shadow-only SITL regression. This script never connects to hardware."""

import argparse
import json
import math
from pathlib import Path
import re
import time

from pymavlink import mavutil

from test_utb_skeleton import SITL


class ShadowSITL(SITL):
    def __init__(self, *args, **kwargs):
        self.statuses = []
        self.roll = self.pitch = self.yaw = 1500
        self.throttle = 1000
        super().__init__(*args, **kwargs)

    def rc(self):
        self.link.mav.rc_channels_override_send(1, 1, self.roll, self.pitch, self.throttle, self.yaw,
                                                self.rc5, 1000, self.rc7, 1000)

    def wait(self, kind, predicate=lambda msg: True, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rc()
            msg = self.link.recv_match(blocking=True, timeout=0.1)
            if msg is not None:
                if msg.get_type() == "STATUSTEXT" and msg.text.startswith("UTB flags="):
                    values = re.search(r"flags=(\d+) calc=(\d+) obs=(\d+) policy=(\d+)", msg.text)
                    if values:
                        self.statuses.append(tuple(map(int, values.groups())))
                if msg.get_type() == kind and predicate(msg):
                    return msg
            if self.process.poll() is not None:
                raise RuntimeError("SITL exited; inspect console log")
        raise AssertionError(f"Timeout waiting for {kind}")

    def duration(self, seconds):
        start = self.wait("SYSTEM_TIME").time_boot_ms
        while self.wait("SYSTEM_TIME").time_boot_ms - start < seconds * 1000:
            pass

    def health(self, predicate, timeout=45):
        start = len(self.statuses)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.wait("SYSTEM_TIME")
            for status in self.statuses[start:]:
                if predicate(*status):
                    return status
        raise AssertionError(f"Health state absent; recent: {self.statuses[-12:]}")


def read_logs(directory):
    records = {}
    for path in sorted((directory / "logs").glob("*.BIN")):
        reader = mavutil.mavlink_connection(str(path))
        while True:
            msg = reader.recv_match()
            if msg is None:
                break
            kind = msg.get_type()
            if kind in {"UTBS", "UTBR", "UTBM", "UTBA", "UTBT", "UTBQ", "RCOU", "PARM", "PIDR", "RATE", "PM", "SIDD", "UBEP"}:
                records.setdefault(kind, []).append(msg.to_dict())
    return records


def launch(binary, root, output, name, extras, action, enabled=1, model="bfx"):
    directory = output / name
    directory.mkdir(parents=True, exist_ok=False)
    with (directory / "console.log").open("w") as console:
        sim = ShadowSITL(binary, root / "Tools/autotest/default_params/copter.parm", directory,
                         console, True, enabled, extra_params=extras, model=model, speedup=1)
        try:
            action(sim)
            sim.duration(2)  # permit normal backend flushing
            statuses = sim.statuses.copy()
        finally:
            sim.close()
    return read_logs(directory), statuses


BASE = ("FRAME_TYPE 12\nLOG_DISARMED 1\nLOG_BITMASK 5225\nUTB_SHADOW 1\nUTB_LOG_RATE 200\n"
        "UTB_RAT_RLL_P 0.1\nUTB_RAT_PIT_P 0.1\nUTB_RAT_YAW_P 0.1\n"
        "UTB_RAT_RLL_I 0.05\nUTB_RAT_RLL_IMAX 0.2\nUTB_RAT_RLL_D 0.001\nUTB_RAT_RLL_D_HZ 20\n"
        "SIM_GYR1_RND 0\nSIM_GYR2_RND 0\nSIM_ACC1_RND 0\nSIM_ACC2_RND 0\nSIM_MAG_RND 0\n"
        "ATC_RAT_RLL_FLTT 0\nACRO_TRAINER 0\n")


def exercise(sim):
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.duration(6)
    sim.param("UTB_LOG_RATE", 400)
    sim.duration(6)
    for param, value in (("FRAME_TYPE", 1), ("FRAME_CLASS", 2)):
        sim.param(param, value)
        sim.health(lambda flags, calc, obs, policy: calc == 6 and not flags & 64)
        sim.param(param, 12 if param == "FRAME_TYPE" else 1)
        sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.param("FSTRATE_ENABLE", 1)
    sim.health(lambda flags, calc, obs, policy: calc == 7 and not flags & 64)
    sim.param("FSTRATE_ENABLE", 0)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.param("UTB_RAT_RLL_P", 6)
    sim.health(lambda flags, calc, obs, policy: calc == 5 and not flags & 16)
    sim.param("UTB_RAT_RLL_P", 0.1)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.param("LOG_DISARMED", 0)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256) and not flags & 512)
    sim.duration(1)
    sim.param("LOG_DISARMED", 1)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    for mode in (1, 2, 0):
        sim.mode(mode, mode)
        sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.mode(40, 40)
    for force in (False, True):
        assert sim.arm(force=force) != mavutil.mavlink.MAV_RESULT_ACCEPTED
        sim.observe(40, False)
    sim.param("RC7_OPTION", 153)
    sim.rc7 = 2000
    sim.observe(40, False, seconds=2)
    sim.rc7 = 1000
    sim.param("RC7_OPTION", 0)
    sim.mode(0, 0)
    sim.param("UTB_SHADOW", 0)
    sim.duration(2)
    sim.param("UTB_SHADOW", 1)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.duration(2)


def assemble_sets(records):
    sets = {}
    for kind in ("UTBR", "UTBM", "UTBA", "UTBT"):
        for msg in records.get(kind, []):
            key = (msg["Seq"], msg["TimeUS"])
            item = sets.setdefault(key, {})
            subkey = f"R{msg['Ax']}" if kind == "UTBR" else kind
            assert subkey not in item, ("Duplicate sample", key, subkey)
            item[subkey] = msg
    return sets


def analyse(records):
    sets = assemble_sets(records)
    complete = [v for v in sets.values() if set(v) == {"R0", "R1", "R2", "UTBM", "UTBA", "UTBT"}]
    assert len(complete) > 1000, len(complete)
    rates = {}
    for requested in (200, 400):
        samples = sorted((v["UTBT"] for v in complete if v["UTBT"]["Req"] == requested), key=lambda v: v["TimeUS"])
        # Measure the full baseline interval, including startup gaps/backend losses.
        # Native worker service is measured at real-time SITL speedup=1, not accelerated time.
        end_us = samples[-1]["TimeUS"] + 1 if requested == 200 else samples[0]["TimeUS"] + 6000000
        segment = [v for v in samples if v["TimeUS"] < end_us]
        assert len(segment) > 300, (requested, len(segment))
        actual = (len(segment) - 1) * 1e6 / (segment[-1]["TimeUS"] - segment[0]["TimeUS"])
        assert abs(actual - requested) < requested * 0.04, (requested, actual)
        rates[str(requested)] = {"complete_sets": len(segment), "observed_hz": actual,
                                 "duration_s": (segment[-1]["TimeUS"] - segment[0]["TimeUS"]) / 1e6}
    for sample in complete:
        timing = sample["UTBT"]
        if timing["Cap"]:
            assert timing["APS"] == timing["Seq"]
            assert timing["APUS"] <= timing["TimeUS"]
            assert timing["APM"] == timing["Md"]
        motor = sample["UTBM"]
        for i in range(1, 5):
            assert math.isfinite(motor[f"M{i}"]) and 0 <= motor[f"M{i}"] <= 1
        if not motor["Val"]:
            assert all(motor[f"M{i}"] == 0 for i in range(1, 5))
    assert max(v["UTBT"]["Drop"] for v in complete) > 0, "Queue overflow was not exercised"
    assert any(s["Calc"] == 6 for s in records["UTBS"])
    assert any(s["Calc"] == 7 for s in records["UTBS"])
    assert any(s["Calc"] == 5 for s in records["UTBS"])
    assert records.get("UTBQ"), "No worker pipeline counters"
    pipeline = records["UTBQ"]
    assert max(p["HWM"] for p in pipeline) == 4
    assert max(p["Drop"] for p in pipeline) > 0
    assert all(p["EnqN"] <= p["ReqN"] and p["WrN"] <= p["EnqN"] for p in pipeline)
    # Remove the final metadata packet from one real set and ensure it is excluded.
    chosen = complete[-1]["UTBT"]
    truncated = {k: list(v) for k, v in records.items()}
    truncated["UTBT"] = [v for v in records["UTBT"] if (v["Seq"], v["TimeUS"]) != (chosen["Seq"], chosen["TimeUS"])]
    assert len([v for v in assemble_sets(truncated).values() if len(v) == 6]) == len(complete) - 1
    execution = [v["UTBT"]["Ex"] for v in complete]
    return {"rates": rates, "complete_sets": len(complete), "partial_sets": len(sets) - len(complete),
            "queue_drops": max(v["UTBT"]["Drop"] for v in complete),
            "shadow_execution_us_max": max(execution),
            "dt_s_min": min(v["UTBT"]["Dt"] for v in complete if v["UTBT"]["Dt"] > 0),
            "dt_s_max": max(v["UTBT"]["Dt"] for v in complete), "shadow_execution_us_mean": sum(execution) / len(execution),
            "pipeline_last": pipeline[-1], "truncated_set_rejected": True,
            "pm": records.get("PM", [])}


def systemid(sim):
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    for name, value in (("SID_AXIS", 7), ("SID_MAGNITUDE", 3), ("SID_F_START_HZ", 2),
                        ("SID_F_STOP_HZ", 4), ("SID_T_FADE_IN", 0), ("SID_T_REC", 12), ("SID_T_FADE_OUT", 0)):
        sim.param(name, value)
    while sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        sim.duration(1)
    sim.throttle = 1600
    sim.wait("GLOBAL_POSITION_INT", lambda m: m.relative_alt > 5000, timeout=40)
    sim.throttle = 1500
    sim.mode(25, 25, armed=True)
    sim.duration(5)
    sim.throttle = 1000
    sim.arm(force=True, disarm=True)


def analyse_sysid(records):
    rates = sorted((r for r in records["UTBR"] if r["Md"] == 25 and r["Ax"] == 0), key=lambda r: r["TimeUS"])
    assert rates and records.get("SIDD"), "No active SYSID experiment"
    assert max(abs(r["AP"]) for r in rates) > 0.01, "No SYSID target excitation captured"
    captures = {t["Seq"]: t["APUS"] for t in records["UTBT"] if t["Md"] == 25 and t["Cap"]}
    targets = {captures[r["Seq"]]: r["AP"] for r in rates if r["Seq"] in captures}
    differences = [abs(pid["Tar"] - targets[pid["TimeUS"]]) for pid in records.get("PIDR", [])
                   if pid["TimeUS"] in targets]
    assert len(differences) > 30, len(differences)
    assert max(differences) < 1e-6, max(differences)
    return {"matched_pid_samples": len(differences), "max_target_difference_rads": max(differences),
            "alignment": "exact PIDR.TimeUS == capture.APUS; AP target filter disabled for this SITL test"}


def primary_gyro(binary, root, output):
    def switch(sim):
        sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
        sim.duration(10)
        sim.param("EK3_PRIMARY", 1)
        sim.duration(10)
        sim.param("EK3_PRIMARY", 0)
        sim.duration(5)

    records, _ = launch(binary, root, output, "primary-gyro", BASE + "EK3_IMU_MASK 3\n", switch)
    indices = {v["Gy"] for v in records.get("UTBQ", []) if v["Gy"] != 255}
    assert indices >= {0, 1}, indices
    changed = [v for v in records["UTBS"] if v["Calc"] == 11]
    assert changed, "No PRIMARY_GYRO_CHANGED reset event"
    assert all(not v["Flg"] & (16 | 64) for v in changed)
    assert any(v["Calc"] == 0 and v["TimeUS"] > changed[0]["TimeUS"] for v in records["UTBS"])
    return {"indices": sorted(indices), "prime_events": len(changed), "recovery": True}


def reference_map(binary, root, output):
    extras = ("RCMAP_ROLL 2\nRCMAP_PITCH 1\nRC1_MIN 1000\nRC1_MAX 2000\nRC1_TRIM 1500\nRC1_DZ 100\n"
              "RC2_MIN 1000\nRC2_MAX 2000\nRC2_TRIM 1500\nRC2_DZ 100\nRC2_REVERSED 1\n"
              "RC4_MIN 1000\nRC4_MAX 2000\nRC4_TRIM 1500\nRC4_DZ 100\n"
              "ACRO_RP_RATE 180\nACRO_Y_RATE 90\nACRO_RP_EXPO 0.5\nACRO_Y_EXPO 0.5\n")

    def action(sim):
        sim.roll, sim.pitch, sim.yaw = 1500, 1700, 1300
        sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
        sim.duration(3)

    records, _ = launch(binary, root, output, "reference", BASE + extras, action)
    expected = [-math.pi / 7, 0, -math.pi / 14]
    for axis in range(3):
        samples = [r for r in records["UTBR"] if r["Ax"] == axis and r["Val"]][-100:]
        assert len(samples) == 100, (axis, len(samples))
        assert max(abs(r["Des"] - expected[axis]) for r in samples) < 1e-6
    return {"RCMAP": "roll RC2, pitch RC1", "RC2_reversed": True, "deadzone_pwm": 100,
            "expected_reference_rads": expected, "samples_verified_per_axis": 100}


def isolation(binary, root, output):
    measured = {}
    for shadow in (0, 1):
        def action(sim):
            sim.roll, sim.pitch, sim.yaw = 1700, 1400, 1600
            sim.duration(5)
            disarmed = []
            for _ in range(20):
                msg = sim.wait("SERVO_OUTPUT_RAW")
                disarmed.append(tuple(getattr(msg, f"servo{i}_raw") for i in range(1, 5)))
            while sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                sim.duration(1)
            sim.observe(0, True)
            sim.duration(3)
            idle = []
            for _ in range(20):
                msg = sim.wait("SERVO_OUTPUT_RAW")
                idle.append(tuple(getattr(msg, f"servo{i}_raw") for i in range(1, 5)))
            sim.mode(40, 0, armed=True)
            sim.arm(force=True, disarm=True)
            measured[shadow] = {"disarmed": disarmed, "armed_idle": idle}
        records, _ = launch(binary, root, output, f"isolation-{shadow}",
                            BASE + f"UTB_SHADOW {shadow}\n", action)
        assert records.get("RCOU"), "Missing output log"
        if shadow:
            assert any(abs(m["Raw"]) > 0.01 and m["Val"] for m in records.get("UTBR", [])), "No UTB demand"
    for phase in ("disarmed", "armed_idle"):
        assert measured[0][phase] == measured[1][phase], (phase, measured)
    return {"scenario": "STABILIZE, same fixed RC, disarmed and armed ground idle; no takeoff",
            "samples_per_phase_per_run": 20, "steady_rcou_differences": 0,
            "outputs": measured[0], "bitwise_flight_or_transient_equivalence_claimed": False}


def source_audit(root):
    bridge = (root / "ArduCopter/utb.cpp").read_text()
    producer = bridge.split("void Copter::utb_log_init()", 1)[0]
    assert "logger." not in producer and "GCS_SEND_TEXT" not in producer
    scheduler = (root / "ArduCopter/Copter.cpp").read_text()
    assert "FAST_TASK(utb_log_update)" not in scheduler
    assert "SCHED_TASK(utb_status_update" not in scheduler
    assert bridge.count("utb_log_update()") == 2  # worker call and definition
    assert bridge.count("utb_status_update(") == 2
    assert "PRIORITY_IO, 0" in bridge
    assert "if (utb.enabled())" in bridge.split("void Copter::utb_log_thread", 1)[0]
    manager = (root / "libraries/AP_UTB/AP_UTB.cpp").read_text()
    assert "take_blocking" not in manager and "WITH_SEMAPHORE" not in manager
    assert "logger." not in manager
    assert "ATOMIC_INT_LOCK_FREE == 2" in manager
    assert "GCS_SEND_TEXT" not in (root / "ArduCopter/mode_utb_acro.cpp").read_text()
    for path in (root / "libraries/AP_UTB").glob("AP_UTB*.*"):
        if path.suffix not in {".cpp", ".h"}:
            continue
        source = path.read_text()
        for forbidden in ("AP_Motors", "SRV_Channels", "hal.rcout", "set_throttle(", "set_roll(",
                          "set_pitch(", "set_yaw(", "AP_HAL_ChibiOS", "send_dshot_command("):
            assert forbidden not in source, (path.name, forbidden)
    return {"utb_motor_writer_dependencies": 0, "producer_backend_calls": 0, "producer_waits": 0,
            "worker_priority": "PRIORITY_IO,0",
            "queue_capacity": 4, "synchronisation": "AP SPSC external ByteBuffer; status HAL try-lock; atomic feedback"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    binary = args.binary.resolve()
    records, statuses = launch(binary, root, output, "shadow", BASE, exercise)
    result = {"source_audit": source_audit(root), "shadow": analyse(records), "health_states": statuses}

    def suppressed(sim):
        sim.health(lambda flags, calc, obs, policy: policy == 1 and not flags & (8 | 32))
        sim.param("LOG_DISARMED", 1)
        sim.health(lambda flags, calc, obs, policy: bool(flags & 256))

    _, states = launch(binary, root, output, "suppressed", BASE + "LOG_DISARMED 0\n", suppressed)
    result["suppressed_health_states"] = states
    sysid, _ = launch(binary, root, output, "systemid", BASE + "UTB_LOG_RATE 400\n", systemid)
    result["sysid"] = analyse_sysid(sysid)
    result["primary_gyro"] = primary_gyro(binary, root, output)
    result["reference_map"] = reference_map(binary, root, output)
    result["motor_isolation"] = isolation(binary, root, output)
    (output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
