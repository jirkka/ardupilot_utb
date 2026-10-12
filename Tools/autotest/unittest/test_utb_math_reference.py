#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Analytical checks of the independent model, separate from C++ comparison."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utb_math_reference import Bound, Reference, allocate, f32, geometry  # noqa: E402


def row(sample, target=0, measurement=0, gains=(0, 0, 0, 0, 0), dt=.0025, action=0):
    return dict(sample=sample, target=[target, 0, 0], measurement=[measurement, 0, 0],
                gains=[gains, (0, 0, 0, 0, 0), (0, 0, 0, 0, 0)], dt=dt, thrust=.5, frame=18, action=action)


class AnalyticalReference(unittest.TestCase):
    def test_proportional_hand_values_and_linearity(self):
        for kp in (.125, .25):
            model = Reference()
            model.step(row(0, gains=(kp, 0, 0, 0, 0)))
            for n, target in enumerate((2, -2, 0), 1):
                result = model.step(row(n, target=target, gains=(kp, 0, 0, 0, 0)))
                self.assertEqual(result['p0'].value, kp * target)
                self.assertEqual(result['out0'].value, kp * target)
                self.assertEqual(result['p1'].value, 0)

    def test_integral_closed_form_and_imax(self):
        for sign in (-1, 1):
            model = Reference()
            gains = (0, 1, 0, .03125, 0)
            model.step(row(0, gains=gains))
            elapsed = 0
            for n in range(1, 81):
                dt = .00125 if n % 2 else .0025
                elapsed += dt
                result = model.step(row(n, target=sign, gains=gains, dt=dt))
                self.assertAlmostEqual(result['i0'].value, sign * min(elapsed, .03125), places=14)
            result = model.step(row(81, gains=gains, action=2))
            self.assertEqual(result['i0'].value, 0)
            self.assertEqual(result['out0'].value, 0)
            self.assertTrue(result['prime'])

    def test_measurement_step_geometric_decay(self):
        # One step in measurement produces beta*A/dt then a geometric tail.
        for hz in (5, 20, 40):
            model = Reference()
            gains = (0, 0, .01, 0, hz)
            model.step(row(0, gains=gains))
            beta = .0025 / (.0025 + 1 / (2 * math.pi * hz))
            for n in range(1, 81):
                result = model.step(row(n, measurement=.1, gains=gains))
                expected = -.01 * beta * .1 / .0025 * (1 - beta) ** (n - 1)
                self.assertAlmostEqual(result['d0'].value, expected, places=14)

    def test_ramp_closed_form_and_no_reference_kick(self):
        gains = (0, 0, .02, 0, 20)
        model = Reference()
        model.step(row(0, measurement=3, gains=gains))
        result = model.step(row(1, target=100, measurement=3, gains=gains))
        self.assertEqual(result['d0'].value, 0)
        beta = .0025 / (.0025 + 1 / (2 * math.pi * 20))
        for n in range(1, 81):
            result = model.step(row(n + 1, measurement=3 + n * .0025 * .5, gains=gains))
            expected = -.02 * .5 * (1 - (1 - beta) ** n)
            self.assertAlmostEqual(result['d0'].value, expected, places=12)

    def test_directional_sustained_saturation(self):
        for sign in (-1, 1):
            model = Reference()
            gains = (1, 1, 0, .5, 0)
            model.step(row(0, gains=gains))
            for n in range(1, 101):
                result = model.step(row(n, target=2 * sign, gains=gains))
                self.assertEqual(result['i0'].value, sign * .005)
                self.assertEqual(result['blocked0'], int(n > 1))
            result = model.step(row(101, target=-sign, gains=gains))
            self.assertFalse(result['blocked0'])
            self.assertEqual(result['i0'].value, sign * .0025)

    def test_geometry_cross_product_and_hand_motor_values(self):
        self.assertEqual(geometry(18), [(-.5, -.5, .5), (-.5, .5, -.5),
                                        (.5, -.5, -.5), (.5, .5, .5)])
        # Positive roll lifts left pair, pitch lifts front pair, yaw lifts CCW pair.
        cases = [([.25, 0, 0], [.375, .375, .625, .625]),
                 ([0, .25, 0], [.375, .625, .375, .625]),
                 ([0, 0, .25], [.625, .375, .375, .625]),
                 ([1, 1, 1], [0, 0, 0, 1])]
        for command, expected in cases:
            result = allocate(list(map(Bound, command)), .5)
            self.assertEqual([result[f'm{i}'].value for i in range(4)], expected)
        result = allocate([Bound(1)] * 3, .5)
        self.assertEqual(result['scale'].value, .5)
        self.assertEqual(result['shift'].value, -.25)
        self.assertEqual(result['at'].value, .25)
        self.assertEqual([result[f'a{i}'].value for i in range(3)], [.5] * 3)

    def test_collective_projection(self):
        for thrust, expected in ((.99, .75), (.01, .25)):
            result = allocate([Bound(.5), Bound(0), Bound(0)], thrust)
            self.assertEqual(result['at'].value, expected)
            self.assertEqual(result['a0'].value, .5)
            self.assertEqual(result['scale'].value, 1)

    def test_float32_roundoff_bound_cancellation(self):
        a, b = Bound(f32(.1)), Bound(f32(.10000001))
        result = (a - b) * f32(.3)
        actual = f32(f32(a.value - b.value) * f32(.3))
        self.assertLessEqual(abs(result.value - actual), result.error)


if __name__ == '__main__':
    unittest.main()
