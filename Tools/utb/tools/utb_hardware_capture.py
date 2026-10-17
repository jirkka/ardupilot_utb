#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Capture DISARMED packed C/D windows, without reboot, download, erase or motor commands."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_hardware_capture.py
# @brief      DISARMED packed C/D capture guardy; skutečný sériový nástroj, v 2E se nespouští.
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
import re
import time

from pymavlink import mavutil


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class Monitor:
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self):
        self.partial = {}
        self.metrics = None
        self.health = None
        self.health_at = 0
        self.metrics_at = 0
        self.boot_ms = 0

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Feed.
    #
    # @param[in]  record Record.
    # @param[in]  now Now.
    # @return     None.
    #
    # @details    Public class method. Calls: record.get, RuntimeError, re.fullmatch, re.search.
    # --------------------------------------------------------------------------------------------------------------------
    def feed(self, record, now):
        kind = record.get("mavpackettype")
        if kind == "HEARTBEAT" and record["base_mode"] & 128:
            raise RuntimeError("STOP: armed heartbeat")
        if kind == "NAMED_VALUE_INT":
            match = re.fullmatch(r"UB(\d{2})", record["name"])
            if match:
                key = int(match[1])
                self.boot_ms = record["time_boot_ms"]
                if key == 0:
                    self.partial = {}
                self.partial[key] = record["value"] & 0xffffffff
                if key == 36:
                    if set(self.partial) == set(range(37)):
                        self.metrics = dict(self.partial)
                        self.metrics_at = now
                    self.partial = {}
        if kind == "STATUSTEXT":
            text = record["text"]
            match = re.search(r"UTB flags=(\d+) calc=(\d+) obs=(\d+) policy=(\d+)", text)
            if match:
                self.health = dict(zip(("flags", "calc", "obs", "policy"), map(int, match.groups())))
                self.health_at = now
            if any(word in text.lower() for word in
                   ("config error:", "chip full", "stack overflow", "thread failed", "internal error", "watchdog")):
                raise RuntimeError("STOP: " + text)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Ready.
    #
    # @param[in]  now Now.
    # @return     Computed result of this operation.
    #
    # @details    Public class method. Calls: all, dict, {10: 3, 18: 3, 33: 4, 35: 0, 36: 1}.items.
    # --------------------------------------------------------------------------------------------------------------------
    def ready(self, now):
        return (self.health is not None and now - self.health_at < 7 and
                self.health == dict(flags=1023, calc=0, obs=0, policy=0) and
                self.metrics is not None and now - self.metrics_at < 3 and
                all(self.metrics[k] == v for k, v in {10: 3, 18: 3, 33: 4, 35: 0, 36: 1}.items()) and
                self.metrics[16] not in (0, 0xffffffff))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Check window.
    #
    # @param[in]  baseline Baseline.
    # @param[in]  now Now.
    # @return     None.
    #
    # @details    Public class method. Calls: self.ready, RuntimeError, max.
    # --------------------------------------------------------------------------------------------------------------------
    def check_window(self, baseline, now):
        if not self.ready(now):
            raise RuntimeError("STOP: invalid or stale shadow/logger diagnostics")
        for key in (22, 23, 26, 35):
            if (self.metrics[key] - baseline[key]) & 0xffffffff:
                raise RuntimeError(f"STOP: pipeline/lifecycle counter changed: UB{key:02}")
        # AP PerfInfo slow-loop threshold at 400 Hz is 1.2 * 2500 us.
        # This is an experiment stop rule, not a certified hardware safety bound.
        if self.metrics[29] > max(3000, baseline[29]):
            raise RuntimeError("STOP: new maximum period beyond AP slow-loop threshold")


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class Connection:
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  port Port.
    # @param[in]  output Explicit destination for new experiment results.
    # @return     None.
    #
    # @details    Public class method. Calls: mavutil.mavlink_connection, (output / 'capture.tlog').open, Monitor,
    #             time.monotonic.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self, port, output):
        self.link = mavutil.mavlink_connection(port, baud=115200, source_system=254)
        self.link.logfile = (output / "capture.tlog").open("xb")
        self.monitor = Monitor()
        self.heartbeat_at = time.monotonic()
        self.sent_at = 0
        self.records = []
        self.events = []
        self.latest = {}

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Receive.
    #
    # @param[in]  None
    # @return     Returned values: msg.
    #
    # @details    Public class method. Calls: time.monotonic, self.link.recv_match, self.link.mav.heartbeat_send,
    #             msg.to_dict.
    # --------------------------------------------------------------------------------------------------------------------
    def receive(self):
        now = time.monotonic()
        if now - self.sent_at >= 1:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.sent_at = now
        msg = self.link.recv_match(blocking=True, timeout=0.1)
        now = time.monotonic()
        if msg:
            record = msg.to_dict()
            self.latest[msg.get_type()] = record
            if msg.get_type() == "HEARTBEAT":
                self.heartbeat_at = now
            if msg.get_type() in ("NAMED_VALUE_INT", "STATUSTEXT", "EKF_STATUS_REPORT", "HEARTBEAT"):
                self.records.append(record)
            self.monitor.feed(record, now)
        if now - self.heartbeat_at > 5:
            raise RuntimeError("STOP: stale FC heartbeat")
        return msg

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Wait.
    #
    # @param[in]  predicate Predicate.
    # @param[in]  timeout Timeout.
    # @return     None.
    #
    # @details    Public class method. Calls: RuntimeError, time.monotonic, self.receive, predicate.
    # --------------------------------------------------------------------------------------------------------------------
    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.receive()
            if predicate():
                return
        raise RuntimeError("STOP: readiness timeout")

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Parameter.
    #
    # @param[in]  name Name.
    # @param[in]  value Value supplied to this operation.
    # @return     Parameter value read from the explicitly connected peer.
    #
    # @details    Public class method. Calls: range, RuntimeError, self.link.mav.param_request_read_send, self.events.append.
    # --------------------------------------------------------------------------------------------------------------------
    def parameter(self, name, value=None):
        for _ in range(3):
            if value is None:
                self.link.mav.param_request_read_send(self.link.target_system, self.link.target_component, name.encode(), -1)
            else:
                self.events.append(dict(action="PARAM_SET", name=name, value=value, wall=time.time()))
                self.link.mav.param_set_send(self.link.target_system, self.link.target_component, name.encode(), value,
                                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            end = time.monotonic() + 3
            while time.monotonic() < end:
                msg = self.receive()
                if msg and msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                    if value is None or abs(msg.param_value - value) < 0.0001:
                        return msg.param_value
        raise RuntimeError("Parameter readback failed: " + name)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Request.
    #
    # @param[in]  msgid Msgid.
    # @return     None.
    #
    # @details    Public class method. Calls: self.link.mav.command_long_send.
    # --------------------------------------------------------------------------------------------------------------------
    def request(self, msgid):
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component,
                                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, msgid, 0, 0, 0, 0, 0, 0)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Window.
    #
    # @param[in]  rate Rate.
    # @param[in]  duration Duration.
    # @return     Returned values: result.
    #
    # @details    Public class method. Calls: self.parameter, self.wait, dict, print.
    # --------------------------------------------------------------------------------------------------------------------
    def window(self, rate, duration):
        self.parameter("UTB_LOG_RATE", rate)
        self.monitor.health = None
        self.monitor.metrics = None
        self.wait(lambda: self.monitor.ready(time.monotonic()) and self.monitor.metrics[2] == rate, 12)
        # Keep logger-start transients out of PM's preceding 10-second interval.
        warmup_end = time.monotonic() + 15
        warmup_base = dict(self.monitor.metrics)
        while time.monotonic() < warmup_end:
            self.receive()
            self.monitor.check_window(warmup_base, time.monotonic())
        start_ms = ((self.monitor.boot_ms + 1999) // 1000) * 1000
        end_ms = start_ms + duration * 1000
        baseline = dict(self.monitor.metrics)
        deadline = time.monotonic() + duration + 6
        while self.monitor.boot_ms < end_ms + 1000:
            self.receive()
            self.monitor.check_window(baseline, time.monotonic())
            if time.monotonic() > deadline:
                raise RuntimeError("STOP: benchmark deadline")
        result = dict(rate=rate, window_us=[start_ms * 1000, end_ms * 1000],
                      baseline=baseline, final=dict(self.monitor.metrics), health=self.monitor.health)
        print(json.dumps(result), flush=True)
        return result


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
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
    parser.add_argument("--port", required=True)
    parser.add_argument("--firmware", required=True, help="expected 8-character AUTOPILOT_VERSION git identity")
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    parser.add_argument("--props-removed", action="store_true", required=True)
    parser.add_argument("--duration", type=int, default=20, choices=range(5, 31))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(wall_start=time.time(), windows=[], success=False)
    connection = None
    verified = False
    try:
        connection = Connection(args.port, args.output)
        connection.wait(lambda: "HEARTBEAT" in connection.latest, 8)
        connection.request(148)
        connection.wait(lambda: "AUTOPILOT_VERSION" in connection.latest, 5)
        identity = bytes(connection.latest["AUTOPILOT_VERSION"]["flight_custom_version"]).decode("ascii").strip("\0")
        if identity != args.firmware:
            raise RuntimeError("Unexpected firmware: " + identity)
        required = dict(UTB_ENABLE=1, UTB_SHADOW=0, LOG_DISARMED=0, FRAME_CLASS=1, FRAME_TYPE=18,
                        FSTRATE_ENABLE=0, SCHED_LOOP_RATE=400, LOG_BACKEND_TYPE=4, UTB_LOG_FMT=1,
                        LOG_BITMASK=180222, LOG_BLK_RATEMAX=0, LOG_DARM_RATEMAX=0, LOG_FILE_BUFSIZE=172)
        result["readback"] = {name: connection.parameter(name) for name in required}
        if result["readback"] != required:
            raise RuntimeError("Unexpected benchmark configuration: " + str(result["readback"]))
        verified = True
        # Check the actual STATE_VALID guard before any DataFlash experiment.
        connection.parameter("UTB_SHADOW", 1)
        connection.monitor.health = None
        connection.wait(lambda: connection.monitor.health is not None and
                        connection.monitor.health["flags"] & 4 and
                        connection.monitor.health["policy"] == 1, 12)
        result["state_gate"] = connection.monitor.health
        connection.parameter("UTB_SHADOW", 0)
        connection.request(65)
        connection.wait(lambda: "RC_CHANNELS" in connection.latest, 5)
        rc = connection.latest["RC_CHANNELS"]
        if rc["chancount"] < 4 or not all(800 <= rc[f"chan{i}_raw"] <= 2200 for i in range(1, 5)):
            raise RuntimeError("Invalid RC readback")
        result["rc"] = rc
        # Start logger and its formats first; do not download between C and D.
        connection.parameter("LOG_DISARMED", 1)
        connection.wait(lambda: connection.monitor.metrics is not None and connection.monitor.metrics[36] == 1, 12)
        until = time.monotonic() + 3
        while time.monotonic() < until:
            connection.receive()
        connection.parameter("UTB_LOG_RATE", 200)
        connection.parameter("UTB_SHADOW", 1)
        for rate in (200, 400):
            result["windows"].append(connection.window(rate, args.duration))
        result["success"] = True
    except Exception as exc:
        result["error"] = str(exc)
        raise
    finally:
        if connection:
            if verified:
                try:
                    result["freeze"] = {name: connection.parameter(name, 0) for name in ("UTB_SHADOW", "LOG_DISARMED")}
                except Exception as exc:
                    result["freeze_error"] = str(exc)
            result.update(events=connection.events, telemetry=connection.records, latest=connection.latest)
            if connection.link.logfile:
                connection.link.logfile.close()
                connection.link.logfile = None
            connection.link.close()
        result["wall_end"] = time.time()
        (args.output / "capture.json").write_text(json.dumps(result, indent=2, default=list) + "\n")


if __name__ == "__main__":
    main()

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
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
