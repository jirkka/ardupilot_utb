#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Independent checks of exported angle metrics and evidence rejection."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utb_attitude_sitl import angle_metrics, analyse  # noqa: E402


class AttitudeEvidence(unittest.TestCase):
    def test_known_first_order_response(self):
        rows = [dict(t=i*.0025, dt=.0025, angle_ref0=0 if i < 400 else .05,
                     angle0=0 if i < 400 else .05*(1-math.exp(-2*(i*.0025-1)))) for i in range(2400)]
        result = angle_metrics(rows, 0)
        step = result['steps'][0]
        self.assertAlmostEqual(step['rise_10_90_s'], math.log(9)/2, delta=.003)
        self.assertAlmostEqual(step['settling_2pct_s'], -math.log(.02)/2, delta=.003)
        self.assertEqual(step['overshoot_pct'], 0)
        self.assertAlmostEqual(result['iae_rad_s'], .05/2, delta=.0001)

    def test_unreached_target_has_no_rise_or_settling(self):
        rows = [dict(t=i*.0025, dt=.0025, angle_ref0=0 if i < 400 else .05, angle0=0) for i in range(1600)]
        step = angle_metrics(rows, 0)['steps'][0]
        self.assertIsNone(step['rise_10_90_s'])
        self.assertIsNone(step['settling_2pct_s'])
        self.assertAlmostEqual(step['steady_error_rad'], -.05)

    def test_signed_return_is_a_measured_transition(self):
        for sign in [-1, 1]:
            rows = [dict(t=i*.0025, dt=.0025, angle_ref0=sign*.05 if i < 400 else 0,
                         angle0=sign*.05 if i < 400 else sign*.05*math.exp(-2*(i*.0025-1))) for i in range(2400)]
            step = angle_metrics(rows, 0)['steps'][0]
            self.assertEqual(step['target_rad'], 0)
            self.assertAlmostEqual(step['rise_10_90_s'], math.log(9)/2, delta=.003)

    def test_incomplete_evidence_is_rejected(self):
        with self.assertRaisesRegex(AssertionError, 'Incomplete finite experiment'):
            analyse(dict(UARX=[], UAMT=[], UAAT=[], UAST=[]), Path('/unused'), 1, 1)


if __name__ == '__main__':
    unittest.main()
