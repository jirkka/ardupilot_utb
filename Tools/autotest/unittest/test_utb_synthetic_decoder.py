#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Source attribution must stop at explicit terminal metadata, not extrapolate through faults."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_utb_synthetic_reference import identity, reference  # noqa: E402


class SourceAttribution(unittest.TestCase):
    def test_terminal_boundary_and_preroll(self):
        for terminal in (3, 4):
            anchors = [dict(Epoch=1, Seq=500, Cycle=0, Test=2, Prof=1),
                       dict(Epoch=1, Seq=800, Cycle=300, Test=terminal, Prof=1)]
            self.assertIsNone(identity(dict(Epoch=1, Seq=499), anchors))
            self.assertEqual(identity(dict(Epoch=1, Seq=799), anchors), (1, 299))
            self.assertIsNone(identity(dict(Epoch=1, Seq=800), anchors))
            self.assertIsNone(identity(dict(Epoch=1, Seq=900), anchors))

    def test_epoch_and_missing_identity(self):
        anchors = [dict(Epoch=2, Seq=100, Cycle=5, Test=2, Prof=1)]
        self.assertIsNone(identity(dict(Epoch=1, Seq=100), anchors))
        self.assertIsNone(identity(dict(Epoch=2, Seq=100), []))
        self.assertEqual(identity(dict(Epoch=2, Seq=100), anchors), (1, 5))

    def test_phase_edges_and_terminal_zero(self):
        self.assertEqual(reference(199, 1)[1], [0, 0, 0])
        self.assertGreater(reference(200, 1)[1][0], 0)
        self.assertLess(reference(1800, 1)[1][0], 0)
        self.assertEqual(reference(8000, 1)[1], [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
