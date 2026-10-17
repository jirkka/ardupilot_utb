#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Phase-0 regression test; launches its own local SITL, never connects to hardware."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_skeleton.py
# @brief      FÁZE 0 enable/disable/mode40/arming izolace; historický runner.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------


# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
from pathlib import Path
import socket
import subprocess
import tempfile
import time

from pymavlink import mavutil


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class SITL:
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  binary Explicit local SITL executable.
    # @param[in]  defaults Defaults.
    # @param[in]  workdir Workdir.
    # @param[in]  log Log.
    # @param[in]  compiled Compiled.
    # @param[in]  enabled Enabled.
    # @param[in]  extra_params Extra params.
    # @param[in]  model Model.
    # @param[in]  speedup Speedup.
    # @return     None.
    #
    # @details    Public class method. Calls: params.write_text, subprocess.Popen, socket.socket, sock.bind.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self, binary, defaults, workdir, log, compiled, enabled, extra_params="", model="quad", speedup=5):
        params = workdir / "utb.parm"
        params.write_text("FRAME_CLASS 1\nFRAME_TYPE 1\nFS_THR_ENABLE 0\nFS_GCS_ENABLE 0\n"
                          "FLTMODE_CH 5\nFLTMODE1 0\nRC7_OPTION 0\nDISARM_DELAY 0\n"
                          + (f"UTB_ENABLE {enabled}\n" if compiled else "") + extra_params)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        self.process = subprocess.Popen(
            [str(binary), "--model", model, "--speedup", str(speedup), "--wipe",
             "--home", "50,14,200,0", "--serial0", f"tcp:{port}",
             "--defaults", f"{defaults},{params}"], cwd=workdir, stdout=log, stderr=subprocess.STDOUT)
        self.link = None
        self.rc5 = 1000
        self.rc7 = 1000
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("SITL exited at startup; inspect its log")
            try:
                self.link = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=255)
                break
            except OSError:
                time.sleep(0.1)
        if self.link is None:
            self.close()
            raise RuntimeError("SITL connection timeout")
        try:
            self.wait("HEARTBEAT")
            self.link.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
        except Exception:
            self.close()
            raise

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Release the resources owned by this instance.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method. Calls: self.process.terminate, self.link.close, self.process.wait, self.process.kill.
    # --------------------------------------------------------------------------------------------------------------------
    def close(self):
        if self.link is not None:
            self.link.close()
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Rc.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method. Calls: self.link.mav.rc_channels_override_send.
    # --------------------------------------------------------------------------------------------------------------------
    def rc(self):
        self.link.mav.rc_channels_override_send(1, 1, 1500, 1500, 1000, 1500,
                                                self.rc5, 1000, self.rc7, 1000)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Wait.
    #
    # @param[in]  kind Kind.
    # @param[in]  predicate Predicate.
    # @param[in]  timeout Timeout.
    # @return     Returned values: msg.
    #
    # @details    Public class method. Calls: AssertionError, time.monotonic, self.rc, self.link.recv_match.
    # --------------------------------------------------------------------------------------------------------------------
    def wait(self, kind, predicate=lambda msg: True, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rc()
            msg = self.link.recv_match(type=kind, blocking=True, timeout=0.2)
            if msg is not None and predicate(msg):
                return msg
            if self.process.poll() is not None:
                raise RuntimeError("SITL exited during test")
        raise AssertionError(f"Timeout waiting for {kind}")

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Param.
    #
    # @param[in]  name Name.
    # @param[in]  value Value supplied to this operation.
    # @return     None.
    #
    # @details    Public class method. Calls: self.link.mav.param_set_send, self.wait, name.encode, abs.
    # --------------------------------------------------------------------------------------------------------------------
    def param(self, name, value):
        self.link.mav.param_set_send(1, 1, name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.wait("PARAM_VALUE", lambda m: m.param_id == name and abs(m.param_value - value) < 0.001)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Observe.
    #
    # @param[in]  mode Mode.
    # @param[in]  armed Armed.
    # @param[in]  seconds Seconds.
    # @return     None.
    #
    # @details    Public class method. Calls: time.monotonic, self.wait, bool.
    # --------------------------------------------------------------------------------------------------------------------
    def observe(self, mode, armed, seconds=2):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            heartbeat = self.wait("HEARTBEAT")
            assert heartbeat.custom_mode == mode, (heartbeat.custom_mode, mode)
            actual = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            assert actual == armed, (actual, armed)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Mode.
    #
    # @param[in]  requested Requested.
    # @param[in]  expected Expected.
    # @param[in]  armed Armed.
    # @return     None.
    #
    # @details    Public class method. Calls: self.link.mav.set_mode_send, self.wait, self.observe.
    # --------------------------------------------------------------------------------------------------------------------
    def mode(self, requested, expected, armed=False):
        self.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, requested)
        self.wait("HEARTBEAT", lambda m: m.custom_mode == expected)
        self.observe(expected, armed)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Arm.
    #
    # @param[in]  force Force.
    # @param[in]  disarm Disarm.
    # @return     Returned values: self.wait('COMMAND_ACK', lambda m: m.command == command).result.
    #
    # @details    Public class method. Calls: self.link.mav.command_long_send, self.wait.
    # --------------------------------------------------------------------------------------------------------------------
    def arm(self, force=False, disarm=False):
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self.link.mav.command_long_send(1, 1, command, 0, 0 if disarm else 1,
                                        21196 if force else 0, 0, 0, 0, 0, 0)
        return self.wait("COMMAND_ACK", lambda m: m.command == command).result


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Run case.
#
# @param[in]  binary Explicit local SITL executable.
# @param[in]  defaults Defaults.
# @param[in]  log_dir Log dir.
# @param[in]  compiled Compiled.
# @param[in]  enabled Enabled.
# @return     None.
#
# @details    Global module function. Calls: tempfile.TemporaryDirectory, (log_dir / f'{name}.log').open, SITL, int.
# --------------------------------------------------------------------------------------------------------------------
def run_case(binary, defaults, log_dir, compiled, enabled):
    name = f"compiled-{int(compiled)}-enabled-{enabled}"
    with tempfile.TemporaryDirectory(prefix="utb-sitl-") as directory, (log_dir / f"{name}.log").open("w") as log:
        sim = SITL(binary, defaults, Path(directory), log, compiled, enabled)
        try:
            sim.mode(0, 0)
            if not compiled or not enabled:
                sim.mode(40, 0)
                if compiled:
                    sim.param("UTB_ENABLE", 1)
                    sim.mode(40, 0)  # enable is latched at boot
                else:
                    sim.link.mav.param_request_read_send(1, 1, b"UTB_ENABLE", -1)
                    try:
                        sim.wait("PARAM_VALUE", lambda m: m.param_id == "UTB_ENABLE", timeout=2)
                    except AssertionError:
                        pass
                    else:
                        raise AssertionError("UTB parameter exists in compiled-out build")
                for original_mode in (1, 2, 0):
                    sim.mode(original_mode, original_mode)
                print(f"PASS {name}: mode rejected; enable/compile-out respected; AP manual modes available", flush=True)
                return

            baseline = sim.wait("SERVO_OUTPUT_RAW")
            outputs = tuple(getattr(baseline, f"servo{i}_raw") for i in range(1, 5))
            sim.mode(40, 40)
            actual = sim.wait("SERVO_OUTPUT_RAW")
            assert tuple(getattr(actual, f"servo{i}_raw") for i in range(1, 5)) == outputs
            assert max(outputs) <= 1000, outputs
            for force in (False, True):
                assert sim.arm(force=force) != mavutil.mavlink.MAV_RESULT_ACCEPTED
                sim.observe(40, False)
            sim.param("UTB_ENABLE", 0)
            assert sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED
            sim.observe(40, False)  # changing the parameter cannot remove the guard

            # Exercise the existing RC mode switch and RC arm option as well.
            sim.param("FLTMODE_CH", 5)
            sim.param("FLTMODE1", 0)
            sim.param("FLTMODE6", 40)
            sim.param("RC7_OPTION", 153)
            sim.rc5 = 1000
            sim.mode(0, 0)
            sim.rc5 = 2000
            sim.wait("HEARTBEAT", lambda m: m.custom_mode == 40)
            sim.observe(40, False)
            sim.rc7 = 2000
            sim.wait("RC_CHANNELS", lambda m: m.chan7_raw == 2000)
            sim.observe(40, False, seconds=3)
            sim.rc7 = 1000
            sim.rc5 = 1000
            sim.wait("HEARTBEAT", lambda m: m.custom_mode == 0)
            sim.observe(0, False)

            # Prove the rejection is mode-specific, not merely an unarmable simulator.
            deadline = time.monotonic() + 30
            while sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                if time.monotonic() >= deadline:
                    raise AssertionError("Cannot arm standard STABILIZE for control test")
                sim.observe(0, False, seconds=1)
            sim.wait("HEARTBEAT", lambda m: bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            sim.observe(0, True)
            sim.mode(40, 0, armed=True)
            sim.rc5 = 2000
            sim.observe(0, True)
            assert sim.arm(force=True, disarm=True) == mavutil.mavlink.MAV_RESULT_ACCEPTED
            sim.wait("HEARTBEAT", lambda m: not (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            print(f"PASS {name}: GCS/RC entry, normal/forced/RC arm rejection, armed entry rejection", flush=True)
        finally:
            sim.close()


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args,
#             args.log_dir.mkdir.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--compiled", type=int, choices=(0, 1), required=True)
    parser.add_argument("--log-dir", type=Path, required=True)
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[3]
    for enabled in ((0, 1) if args.compiled else (0,)):
        run_case(args.binary.resolve(), root / "Tools/autotest/default_params/copter.parm",
                 args.log_dir.resolve(), bool(args.compiled), enabled)


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
