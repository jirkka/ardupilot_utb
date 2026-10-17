#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Pure host guards; no serial connection or physical commands."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_synthetic_hardware_capture.py
# @brief      Unit regrese: Pure host guards; no serial connection or physical commands.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from utb_synthetic_hardware_capture import BenchConnection, completion_candidate, state_ready  # noqa: E402
from utb_hardware_capture import Connection  # noqa: E402


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class SyntheticHardwareGuards(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify actual state and freshness.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: SimpleNamespace, self.assertTrue, self.assertFalse, state_ready.
    # --------------------------------------------------------------------------------------------------------------------
    def test_actual_state_and_freshness(self):
        connection = SimpleNamespace(monitor=SimpleNamespace(health=dict(flags=4, calc=0, policy=1),
                                                             health_at=time.monotonic()))
        self.assertTrue(state_ready(connection))
        connection.monitor.health['calc'] = 2
        self.assertFalse(state_ready(connection))
        connection.monitor.health['calc'] = 0
        connection.monitor.health_at -= 8
        self.assertFalse(state_ready(connection))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify terminal is only candidate.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: SimpleNamespace, self.assertFalse, self.assertTrue, completion_candidate.
    # --------------------------------------------------------------------------------------------------------------------
    def test_terminal_is_only_candidate(self):
        connection = SimpleNamespace(monitor=SimpleNamespace(health=dict(flags=4, calc=4, policy=0)))
        self.assertFalse(completion_candidate(connection, 5))
        self.assertTrue(completion_candidate(connection, 21))
        connection.monitor.health['policy'] = 2
        self.assertFalse(completion_candidate(connection, 21))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unapproved write rejected before send.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: object.__new__, self.assertRaises, connection.write.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unapproved_write_rejected_before_send(self):
        connection = object.__new__(BenchConnection)
        with self.assertRaises(ValueError):
            connection.write('FS_THR_ENABLE', 0)
        with self.assertRaises(ValueError):
            connection.write('FRAME_TYPE', 12)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify no rc and ekf failures stop.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: object.__new__, SimpleNamespace, patch.object, self.assertIs.
    # --------------------------------------------------------------------------------------------------------------------
    def test_no_rc_and_ekf_failures_stop(self):
        connection = object.__new__(BenchConnection)
        connection.stopping = False
        for kind, fields in [('RC_CHANNELS', dict(chancount=16)), ('EKF_STATUS_REPORT', dict(flags=0))]:
            message = SimpleNamespace(get_type=lambda: kind, **fields)
            with patch.object(Connection, 'receive', return_value=message):
                with self.assertRaises(RuntimeError):
                    connection.receive()
        connection.stopping = True
        message = SimpleNamespace(get_type=lambda: 'EKF_STATUS_REPORT', flags=0)
        with patch.object(Connection, 'receive', return_value=message):
            self.assertIs(connection.receive(), message)  # Safe diagnostic shutdown can receive acknowledgements.


if __name__ == '__main__':
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
