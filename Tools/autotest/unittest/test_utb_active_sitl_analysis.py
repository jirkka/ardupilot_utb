#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Measurement definitions reject misleading or incomplete actual SITL evidence."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utb_active_sitl_analysis import alignment, check_rows, step_metrics  # noqa: E402


def rows():
    return [dict(t=i*.0025, dt=.0025, seq=i+1, motor_calls=i+1, ap_calls=0, utb_calls=i+1,
                 **{k+str(a): 0. for a in range(3) for k in ['ref', 'target', 'gyro']}) for i in range(3200)]


class MeasurementTest(unittest.TestCase):
    def test_missing_duplicate_nonfinite_and_competing_authority(self):
        baseline = rows()
        check_rows(baseline, 1)
        with self.assertRaises(ValueError):
            check_rows(baseline[:-1], 1)
        for field, value in [('seq', 4), ('motor_calls', 500), ('ap_calls', 1), ('utb_calls', 2),
                             ('gyro0', math.nan), ('dt', 0.)]:
            bad = copy.deepcopy(baseline)
            bad[5][field] = value
            with self.assertRaises(ValueError, msg=field):
                check_rows(bad, 1)

    def test_shared_target_and_time_alignment(self):
        a = rows()
        self.assertEqual(alignment(a, a)['max_target_difference_rads'], 0.)
        for field, value in [('target0', .1), ('ref0', .1), ('t', .005)]:
            b = copy.deepcopy(a)
            b[0][field] = value
            with self.assertRaises(ValueError, msg=field):
                alignment(a, b)

    def test_signed_step_and_unreached_rise(self):
        plateau = [dict(t=i*.01, ref0=-.15, target0=-.15, gyro0=-.15) for i in range(200)]
        result = step_metrics(plateau, 0)
        self.assertEqual(result['sign'], -1)
        self.assertEqual(result['overshoot_percent'], 0)
        self.assertEqual(result['settle_2percent_s'], 0)
        for row in plateau:
            row['gyro0'] = -.1
        result = step_metrics(plateau, 0)
        self.assertIsNone(result['rise_10_90_s'])
        self.assertIsNone(result['settle_2percent_s'])
        self.assertAlmostEqual(result['steady_error_rads'], -.05)

    def test_sine_is_not_a_step_and_final_sample_is_not_settling(self):
        plateau = [dict(t=i*.01, ref0=.15, target0=.15, gyro0=.1) for i in range(200)]
        plateau[-1]['gyro0'] = .15
        self.assertIsNone(step_metrics(plateau, 0)['settle_2percent_s'])
        plateau[-1]['ref0'] = .14
        with self.assertRaises(ValueError):
            step_metrics(plateau, 0)


if __name__ == '__main__':
    unittest.main()
