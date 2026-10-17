#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Source attribution must stop at explicit terminal metadata, not extrapolate through faults."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_synthetic_decoder.py
# @brief      Unit regrese: Source attribution must stop at explicit terminal metadata, not extrapolate through
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
from test_utb_synthetic_reference import identity, reference  # noqa: E402


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class SourceAttribution(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify terminal boundary and preroll.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertIsNone, self.assertEqual, dict, identity.
    # --------------------------------------------------------------------------------------------------------------------
    def test_terminal_boundary_and_preroll(self):
        for terminal in (3, 4):
            anchors = [dict(Epoch=1, Seq=500, Cycle=0, Test=2, Prof=1),
                       dict(Epoch=1, Seq=800, Cycle=300, Test=terminal, Prof=1)]
            self.assertIsNone(identity(dict(Epoch=1, Seq=499), anchors))
            self.assertEqual(identity(dict(Epoch=1, Seq=799), anchors), (1, 299))
            self.assertIsNone(identity(dict(Epoch=1, Seq=800), anchors))
            self.assertIsNone(identity(dict(Epoch=1, Seq=900), anchors))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify epoch and missing identity.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertIsNone, self.assertEqual, dict, identity.
    # --------------------------------------------------------------------------------------------------------------------
    def test_epoch_and_missing_identity(self):
        anchors = [dict(Epoch=2, Seq=100, Cycle=5, Test=2, Prof=1)]
        self.assertIsNone(identity(dict(Epoch=1, Seq=100), anchors))
        self.assertIsNone(identity(dict(Epoch=2, Seq=100), []))
        self.assertEqual(identity(dict(Epoch=2, Seq=100), anchors), (1, 5))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify phase edges and terminal zero.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertEqual, self.assertGreater, self.assertLess, reference.
    # --------------------------------------------------------------------------------------------------------------------
    def test_phase_edges_and_terminal_zero(self):
        self.assertEqual(reference(199, 1)[1], [0, 0, 0])
        self.assertGreater(reference(200, 1)[1][0], 0)
        self.assertLess(reference(1800, 1)[1][0], 0)
        self.assertEqual(reference(8000, 1)[1], [0, 0, 0])


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
