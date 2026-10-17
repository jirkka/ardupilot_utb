#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Hardware capture stop conditions and complete telemetry frame regressions."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_hardware_capture.py
# @brief      Unit regrese: Hardware capture stop conditions and complete telemetry frame regressions.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from utb_hardware_capture import Monitor  # noqa: E402


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Healthy monitor.
#
# @param[in]  None
# @return     Returned values: monitor.
#
# @details    Global module function. Calls: Monitor, range, monitor.feed, dict.
# --------------------------------------------------------------------------------------------------------------------
def healthy_monitor():
    monitor = Monitor()
    values = {10: 3, 18: 3, 33: 4, 35: 0, 36: 1, 16: 1784, 29: 2666}
    for key in range(37):
        monitor.feed(dict(mavpackettype="NAMED_VALUE_INT", name=f"UB{key:02}",
                          time_boot_ms=40000, value=values.get(key, 0)), 10)
    monitor.feed(dict(mavpackettype="STATUSTEXT", text="UTB flags=1023 calc=0 obs=0 policy=0"), 10)
    return monitor


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class CaptureTest(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify complete frame required.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: Monitor, self.assertIsNone, healthy_monitor, self.assertTrue.
    # --------------------------------------------------------------------------------------------------------------------
    def test_complete_frame_required(self):
        monitor = Monitor()
        for key in (0, 10, 18, 36):
            monitor.feed(dict(mavpackettype="NAMED_VALUE_INT", name=f"UB{key:02}", time_boot_ms=1, value=1), 10)
        self.assertIsNone(monitor.metrics)
        monitor = healthy_monitor()
        self.assertTrue(monitor.ready(11))
        for key in range(1, 37):
            monitor.feed(dict(mavpackettype="NAMED_VALUE_INT", name=f"UB{key:02}", time_boot_ms=45000, value=0), 14)
        self.assertEqual(monitor.metrics_at, 10)
        self.assertFalse(monitor.ready(14))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify stale state and frame rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: healthy_monitor, self.assertFalse, monitor.ready.
    # --------------------------------------------------------------------------------------------------------------------
    def test_stale_state_and_frame_rejected(self):
        monitor = healthy_monitor()
        self.assertFalse(monitor.ready(14))
        monitor.metrics_at = 17
        self.assertFalse(monitor.ready(18))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify invalid shadow rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: healthy_monitor, dict, monitor.feed, self.assertRaises.
    # --------------------------------------------------------------------------------------------------------------------
    def test_invalid_shadow_rejected(self):
        monitor = healthy_monitor()
        baseline = dict(monitor.metrics)
        monitor.feed(dict(mavpackettype="STATUSTEXT", text="UTB flags=643 calc=2 obs=0 policy=0"), 11)
        with self.assertRaises(RuntimeError):
            monitor.check_window(baseline, 11)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify drop and wrapped counter rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: healthy_monitor, dict, self.assertRaises, monitor.check_window.
    # --------------------------------------------------------------------------------------------------------------------
    def test_drop_and_wrapped_counter_rejected(self):
        for key in (22, 23, 26, 35):
            monitor = healthy_monitor()
            monitor.metrics[key] = 0xffffffff
            baseline = dict(monitor.metrics)
            monitor.metrics[key] = 0
            with self.assertRaises(RuntimeError):
                monitor.check_window(baseline, 11)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unknown stack and new slow period rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: healthy_monitor, dict, self.assertFalse, monitor.ready.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unknown_stack_and_new_slow_period_rejected(self):
        monitor = healthy_monitor()
        baseline = dict(monitor.metrics)
        monitor.metrics[16] = 0xffffffff
        self.assertFalse(monitor.ready(11))
        monitor.metrics[16] = 1784
        monitor.metrics[29] = 3001
        with self.assertRaises(RuntimeError):
            monitor.check_window(baseline, 11)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify armed rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertRaises, Monitor().feed, dict, Monitor.
    # --------------------------------------------------------------------------------------------------------------------
    def test_armed_rejected(self):
        with self.assertRaises(RuntimeError):
            Monitor().feed(dict(mavpackettype="HEARTBEAT", base_mode=128), 10)


if __name__ == "__main__":
    unittest.main()

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
