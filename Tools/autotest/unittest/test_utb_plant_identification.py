#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""1H-E physical model/measurement tests; every dataset is demonstrative."""
import copy
import dataclasses
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utb_closed_loop import (DT, Model, advance, derivative, forces_and_torque,  # noqa: E402
                             interpolate_curve, poles, simulate, validate_tensor)
from utb_plant_identification import bifilar, cad_mass_properties, load_dataset, pendulum_period  # noqa: E402
from utb_plant_sensitivity import torque_bounds  # noqa: E402


def dataset():
    return json.loads((Path(__file__).resolve().parents[1]/'utb_1he_demo_plant.json').read_text())


class IdentificationTests(unittest.TestCase):
    def test_bifilar_known_inertia_and_fixture(self):
        inertia, mass, fixture = .002, .65, .05
        target_fixture = .0002
        t = 2*math.pi*math.sqrt((inertia+target_fixture)*.6/((mass+fixture)*9.80665*.1*.1))
        result = bifilar(mass, fixture, .1, .1, .6, t, fixture_inertia=target_fixture)
        self.assertAlmostEqual(result['inertia_kgm2'], inertia, places=14)

    def test_bifilar_offset_correction(self):
        a = bifilar(.65, 0, .1, .1, .6, 1.)
        b = bifilar(.65, 0, .1, .1, .6, 1., cg_offset=.01)
        self.assertAlmostEqual(a['inertia_kgm2']-b['inertia_kgm2'], .65*.01**2, places=14)

    def test_damped_period_correction(self):
        a = bifilar(.65, 0, .1, .1, .6, 1.)
        b = bifilar(.65, 0, .1, .1, .6, 1., damping_ratio=.1)
        self.assertAlmostEqual(b['inertia_kgm2']/a['inertia_kgm2'], .99, places=14)

    def test_bifilar_uncertainty_period_factor_two(self):
        r = bifilar(.65, 0, .1, .1, .6, 1., uncertainties={'period': .001})
        self.assertAlmostEqual(r['standard_uncertainty_kgm2']/r['inertia_kgm2'], .002, places=14)

    def test_correlated_spacing_uncertainty(self):
        u = {'top_half_spacing': .001, 'bottom_half_spacing': .001}
        independent = bifilar(.65, 0, .1, .1, .6, 1., uncertainties=u)
        correlated = bifilar(.65, 0, .1, .1, .6, 1., uncertainties=u,
                             covariances={'top_half_spacing|bottom_half_spacing': 1e-6})
        self.assertAlmostEqual(correlated['standard_uncertainty_kgm2']/independent['standard_uncertainty_kgm2'], math.sqrt(2))

    def test_duplicate_or_nonpositive_semidefinite_covariance_rejected(self):
        u = {'top_half_spacing': .001, 'bottom_half_spacing': .001, 'vertical_height': .001}
        for covariance in [dict([('top_half_spacing|bottom_half_spacing', 1e-6),
                                 ('bottom_half_spacing|top_half_spacing', 1e-6)]),
                           {'top_half_spacing|bottom_half_spacing': -1e-6,
                            'top_half_spacing|vertical_height': -1e-6,
                            'bottom_half_spacing|vertical_height': -1e-6}]:
            with self.assertRaises(ValueError):
                bifilar(.65, 0, .1, .1, .6, 1., uncertainties=u, covariances=covariance)

    def test_invalid_pendulum_geometry(self):
        for kwargs in [dict(vertical_height=0), dict(damping_ratio=1), dict(fixture_inertia=1)]:
            values = dict(drone_mass=.65, fixture_mass=0, top_half_spacing=.1,
                          bottom_half_spacing=.1, vertical_height=.6, period=1.)
            values.update(kwargs)
            with self.assertRaises(ValueError):
                bifilar(**values)

    def test_period_extraction(self):
        t = [i*.005 for i in range(3000)]
        angles = [.06*math.exp(-.02*x)*math.sin(2*math.pi*x/.9+.2) for x in t]
        r = pendulum_period(t, angles)
        self.assertAlmostEqual(r['period_s'], .9, places=5)
        self.assertGreater(r['cycles'], 10)

    def test_period_requires_monotonic_clock_and_enough_cycles(self):
        with self.assertRaises(ValueError):
            pendulum_period([0]*20, [.1]*20)
        with self.assertRaises(ValueError):
            pendulum_period(list(range(20)), [.1]*20)

    def test_dimensional_exponents(self):
        values = dict(drone_mass=.65, fixture_mass=0, top_half_spacing=.1,
                      bottom_half_spacing=.1, vertical_height=.6, period=1.)
        original = bifilar(**values)['inertia_kgm2']
        for field, ratio in [('drone_mass', 2), ('top_half_spacing', 2), ('bottom_half_spacing', 2),
                             ('vertical_height', .5), ('period', 4)]:
            changed = dict(values)
            changed[field] *= 2
            self.assertAlmostEqual(bifilar(**changed)['inertia_kgm2']/original, ratio)
        # Changing the length unit scale multiplies r cross F by that scale.
        a = Model()
        b = dataclasses.replace(a, arm=a.arm*2, thrust_max=a.thrust_max*2)
        _, before = forces_and_torque([1, 0, 0, 0], a)
        _, after = forces_and_torque([1, 0, 0, 0], b)
        for axis in [0, 1]:
            self.assertAlmostEqual(after[axis]/before[axis], 4)

    def test_cad_center_and_parallel_axis(self):
        intrinsic = [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]]
        components = [dict(mass_kg=1., position_m=[x, 0, 0], body_aligned_local_inertia_kgm2=intrinsic) for x in [-.1, .1]]
        r = cad_mass_properties(components)
        self.assertEqual(r['cg_m'], [0, 0, 0])
        self.assertAlmostEqual(r['inertia_kgm2'][0][0], .0002)
        self.assertAlmostEqual(r['inertia_kgm2'][1][1], .0202)


class CalibrationTests(unittest.TestCase):
    def test_demo_provenance_and_equivalence(self):
        model, report = load_dataset(dataset())
        self.assertIn('DEMONSTRATION', report['label'])
        a, _ = simulate(Model(), 'axes', seconds=2.)
        b, _ = simulate(model, 'axes', seconds=2.)
        self.assertLess(max(abs(x[f'w{k}']-y[f'w{k}']) for x, y in zip(a, b) for k in range(3)), 1e-12)

    def test_wrong_unit_rejected(self):
        d = dataset()
        d['parameters']['inertia']['unit'] = 'g*cm^2'
        with self.assertRaises(ValueError):
            load_dataset(d)

    def test_unknown_and_missing_rejected(self):
        for change in ['missing', 'unknown']:
            d = dataset()
            if change == 'missing':
                del d['parameters']['cg']
            else:
                d['parameters']['mass']['status'] = 'unknown'
            with self.assertRaises(ValueError):
                load_dataset(d)

    def test_demo_cannot_claim_measured(self):
        d = dataset()
        d['parameters']['mass']['status'] = 'measured'
        with self.assertRaises(ValueError):
            load_dataset(d)

    def test_center_of_mass_geometry_conversion(self):
        d = dataset()
        original = copy.deepcopy(d['parameters']['motor_positions']['value'])
        d['position_reference'] = 'body_origin'
        d['parameters']['cg']['value'] = [.01, .02, .03]
        model, _ = load_dataset(d)
        for a, b in zip(model.motor_positions, original):
            self.assertEqual(a, tuple(x-c for x, c in zip(b, [.01, .02, .03])))

    def test_fractional_delay_quantization_is_explicit(self):
        d = dataset()
        d['parameters']['actuator_delay']['value'] = .001
        model, report = load_dataset(d)
        self.assertEqual(model.actuator_cycles, 1)
        self.assertGreater(report['delay_quantization']['actuator_delay']['rounding_s'], .001)

    def test_exact_nominal_cycle_delay_does_not_gain_an_extra_cycle(self):
        for delay, expected in [(.0025, 1), (.005, 2), (.00501, 3)]:
            d = dataset()
            d['parameters']['sensor_delay']['value'] = delay
            model, report = load_dataset(d)
            self.assertEqual(model.sensor_cycles, expected)
            self.assertGreaterEqual(report['delay_quantization']['sensor_delay']['rounding_s'], 0)

    def test_asymmetric_and_nonphysical_tensor_rejected(self):
        for tensor in [[[1, 0, 0], [1, 1, 0], [0, 0, 1]],
                       [[1, 2, 0], [2, 1, 0], [0, 0, 1]],
                       [[1, 0, 0], [0, 1, 0], [0, 0, 3]]]:
            with self.assertRaises(ValueError):
                validate_tensor(tensor)

    def test_general_tensor_preserves_energy(self):
        tensor = ((.002, .0002, 0), (.0002, .0025, 0), (0, 0, .004))
        model = dataclasses.replace(Model(), inertia_tensor=tensor, damping=(0.,)*3)
        state = [1., 2., 3.]+[0.]*4

        def energy(x):
            return sum(x[i]*sum(tensor[i][j]*x[j] for j in range(3)) for i in range(3))/2
        before = energy(state)
        for _ in range(400):
            state = advance(state, [0.]*4, DT, model)
        self.assertAlmostEqual(energy(state), before, places=12)

    def test_general_geometry_motor_signs(self):
        model = dataclasses.replace(Model(), motor_positions=((-1, 2, .1), (1, 2, .1), (-1, -2, .1), (1, -2, .1)))
        _, torque = forces_and_torque([1, 0, 0, 0], model)
        self.assertEqual(torque[:2], [-6, -3])

    def test_curve_deadzone_and_saturation(self):
        curve = ((0, 0), (.2, 0), (.8, 3), (1, 3))
        self.assertEqual(interpolate_curve(curve, .1), 0)
        self.assertEqual(interpolate_curve(curve, .9), 3)
        with self.assertRaises(ValueError):
            dataclasses.replace(Model(), thrust_curves=(((0, 0), (1, -1)),)*4)

    def test_voltage_and_torque_bounds(self):
        a = torque_bounds(Model())
        b = torque_bounds(dataclasses.replace(Model(), voltage_ratio=.9))
        for lo, hi in zip(a['max_Nm'], b['max_Nm']):
            self.assertAlmostEqual(hi/lo, .81)

    def test_unimplemented_general_poles_are_not_reported_as_valid(self):
        model, _ = load_dataset(dataset())
        with self.assertRaises(ValueError):
            poles(model, 0)

    def test_individual_motor_lag_zero(self):
        model = dataclasses.replace(Model(), motor_tau_per_motor=(0, .01, .02, .03))
        state = advance([0.]*7, [.5]*4, DT, model)
        self.assertEqual(state[3], .5)
        self.assertTrue(state[4] > state[5] > state[6])

    def test_common_scale_not_identifiable(self):
        a = Model()
        b = dataclasses.replace(a, inertia=tuple(j*1.5 for j in a.inertia), damping=tuple(j*1.5 for j in a.damping),
                                thrust_max=a.thrust_max*1.5)
        da = derivative([1, 2, 3]+[.5]*4, [.5]*4, a)
        db = derivative([1, 2, 3]+[.5]*4, [.5]*4, b)
        self.assertLess(max(abs(x-y) for x, y in zip(da, db)), 1e-12)

    def test_integration_convergence_general_tensor(self):
        model, _ = load_dataset(dataset())
        a, _ = simulate(dataclasses.replace(model, substeps=2), 'axes', seconds=2)
        b, _ = simulate(dataclasses.replace(model, substeps=8), 'axes', seconds=2)
        self.assertLess(max(abs(x[f'w{k}']-y[f'w{k}']) for x, y in zip(a, b) for k in range(3)), 2e-7)


if __name__ == '__main__':
    unittest.main()
