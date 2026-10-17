#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Wire-format and completeness regressions independent of a running vehicle."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_log_decode.py
# @brief      Unit regrese: Wire-format and completeness regressions independent of a running vehicle.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from utb_log_decode import decode_packed  # noqa: E402
from utb_bench_analysis import analyse  # noqa: E402


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Packet.
#
# @param[in]  seq Sample sequence identifier.
# @param[in]  stamp Stamp.
# @param[in]  epoch Epoch.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: b''.join, struct.pack, struct.unpack, dict.
# --------------------------------------------------------------------------------------------------------------------
def packet(seq=0xffffffff, stamp=2**40, epoch=17):
    data = b"".join(struct.pack("<9fBB", 1.25 + i, -2.5 - i, 3.75, 0.1, -0.2, -0.0,
                                -0.125, 0.25, 5.5, 1, 1 << i) for i in range(3))
    data += struct.pack("<10fBBB4fBBBBfIHIB", 0.125, -0.25, 0.5, 0.625,
                        0.125, 0.25, 0.5, 0.75, 0.875, -0.125, 1, 5, 10,
                        0.1, -0.2, 0.3, 0.6, 9, 6, 1, 1, 0.0025, 37, 400, 0xffffffff, 1)
    words = struct.unpack("<96h", data)
    return dict(mavpackettype="UTBC", TimeUS=stamp, Seq=seq, Epoch=epoch, Md=2,
                APUS=stamp-20, APS=seq, APM=2, Cap=1, RCms=123456, Dr=13, Ms=19, Eff=399.5,
                D0=words[:32], D1=words[32:64], D2=words[64:])


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class PackedLogTest(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify values alignment and identity.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: decode_packed, self.assertEqual, range, packet.
    # --------------------------------------------------------------------------------------------------------------------
    def test_values_alignment_and_identity(self):
        result = decode_packed(packet())
        self.assertEqual(len(result), 7)
        for axis in range(3):
            self.assertEqual(result[axis]["Ax"], axis)
            self.assertEqual(result[axis]["Des"], 1.25 + axis)
            self.assertEqual(result[axis]["Meas"], -2.5 - axis)
            self.assertEqual(result[axis]["Sat"], 1 << axis)
            self.assertEqual(struct.pack("<f", result[axis]["D"]), b"\0\0\0\x80")
        self.assertEqual(result[3]["Hi"], 10)
        self.assertEqual(result[3]["M4"], 0.75)
        self.assertEqual(result[4]["Pos"], 9)
        timing = result[5]
        self.assertEqual((timing["Req"], timing["Ex"], timing["Max"], timing["Drop"], timing["Miss"]),
                         (400, 37, 0xffffffff, 13, 19))
        self.assertEqual(timing["APUS"], 2**40 - 20)
        self.assertEqual(timing["Seq"], 0xffffffff)
        self.assertEqual(timing["Eff"], 399.5)
        self.assertEqual(result[6]["Epoch"], 17)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify truncated array rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: packet, self.assertRaises, decode_packed.
    # --------------------------------------------------------------------------------------------------------------------
    def test_truncated_array_rejected(self):
        rec = packet()
        rec["D1"] = rec["D1"][:-1]
        with self.assertRaises(ValueError):
            decode_packed(rec)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unknown version rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: packet, list, self.assertRaises, decode_packed.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unknown_version_rejected(self):
        rec = packet()
        rec["D2"] = list(rec["D2"])
        rec["D2"][-1] = 512
        with self.assertRaises(ValueError):
            decode_packed(rec)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify out of range words rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: packet, self.assertRaises, decode_packed.
    # --------------------------------------------------------------------------------------------------------------------
    def test_out_of_range_words_rejected(self):
        rec = packet()
        rec["D0"] = [32768] * 32
        with self.assertRaises(struct.error):
            decode_packed(rec)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify full window loss and mixed formats.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: decode_packed, analyse, self.assertEqual, packet.
    # --------------------------------------------------------------------------------------------------------------------
    def test_full_window_loss_and_mixed_formats(self):
        legacy = decode_packed(packet(2, 10000))
        records = [packet(1, 5000), *legacy, packet(3, 15000, 18)]
        result = analyse(records, 0, 20000)
        self.assertEqual(result["complete_sets"], 3)
        self.assertEqual(result["persisted_rate_hz"], 150)
        self.assertEqual(result["epochs"]["17"]["interior_rate_hz"], 200)
        self.assertEqual(analyse(records + [records[0]])["complete_sets"], 2)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify missing packet is not reconstructed.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: analyse, self.assertEqual, packet.
    # --------------------------------------------------------------------------------------------------------------------
    def test_missing_packet_is_not_reconstructed(self):
        result = analyse([packet(1, 5000), packet(3, 15000)], 0, 20000)
        self.assertEqual(result["complete_sets"], 2)
        self.assertEqual(result["persisted_rate_hz"], 100)


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
