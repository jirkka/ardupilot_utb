#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# @file test_utb_pilot_rates.py
# @brief Independent Actual Rates endpoint, curvature, domain and unit regressions.
# @author Jiri Kovar
# @version 2026-10-19
# @date 2026-10-19
# GNU General Public License version 3 or later.
# Include Files
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from utb_pilot_rates_sitl import actual, inverse_actual, manual_throttle_pwm  # noqa: E402
# End Include Files
# Data type, constant, macro definition


class ActualRatesTest(unittest.TestCase):
    # Public functions
    # @brief Verify physical endpoint units independently of expo.
    # @param[in] self Test instance.
    # @return None.
    # @details No inference of physical tracking performance.
    def test_endpoints_and_signs(self):
        for expo in (0, .3, 1):
            for sign in (-1, 1):
                self.assertAlmostEqual(actual(sign, 160, 670, expo), sign * math.radians(670))
                self.assertEqual(actual(0, 160, 670, expo), 0)

    # @brief Check center slope and independently calculated middle-stick values.
    # @param[in] self Test instance.
    # @return None.
    # @details Center slope is not the maximum rate.
    def test_center_and_expo(self):
        self.assertAlmostEqual(math.degrees(actual(1e-7, 160, 670, .3)) / 1e-7, 160, places=3)
        self.assertAlmostEqual(math.degrees(actual(.5, 160, 670, 0)), 207.5)
        self.assertAlmostEqual(math.degrees(actual(.5, 160, 670, .3)), 171.640625)
        self.assertAlmostEqual(math.degrees(actual(.5, 160, 670, 1)), 87.96875)

    # @brief Reject out-of-domain and nonfinite configuration.
    # @param[in] self Test instance.
    # @return None.
    # @details Prevents invalid evidence from masquerading as a bounded reference.
    def test_invalid(self):
        for values in ((math.nan, 160, 670, .3), (1.1, 160, 670, .3), (0, 671, 670, .3),
                       (0, 160, 671, .3), (0, 160, 670, -1), (0, 160, 670, math.inf)):
            with self.assertRaises(ValueError):
                actual(*values)

    # @brief Check invertible reference selection and reject impossible requests.
    # @param[in] self Test instance.
    # @return None.
    # @details PWM quantization is evaluated from BIN rather than hidden in the inverse.
    def test_inverse_domain(self):
        self.assertAlmostEqual(inverse_actual(171.640625, 160, 670), .5, places=10)
        for desired in (-1, 671, math.nan):
            with self.assertRaises(ValueError):
                inverse_actual(desired, 160, 670)

    # @brief Verify fixture PWM against the native cubic and reject invalid thrust.
    # @param[in] self Test instance.
    # @return None.
    # @details Checks quantized inverse separately from the experimental controller.
    def test_manual_throttle_fixture(self):
        for hover in (.39, .5, .7):
            for thrust in (0, .01, .39, .8, 1):
                pwm = manual_throttle_pwm(thrust, hover)
                x = (pwm - 1000) / 1000
                expo = max(-.5, min(1, -(hover - .5) / .375))
                self.assertLessEqual(abs(x * (1 - expo) + expo * x**3 - thrust), .0016)
        for thrust, hover in ((-1, .39), (2, .39), (math.nan, .39), (.4, 0), (.4, math.inf)):
            with self.assertRaises(ValueError):
                manual_throttle_pwm(thrust, hover)
    # End Public functions
# End Data type, constant, macro definition


if __name__ == '__main__':
    unittest.main()
