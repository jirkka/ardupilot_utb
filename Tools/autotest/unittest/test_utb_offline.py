#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline model/transfer tests; no connections, simulator vehicle or firmware writes."""
import dataclasses
import json
import math
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utb_closed_loop import (DT, GAINS, Model, advance, forces_and_torque, linear_step, metrics, simulate,  # noqa: E402
                             validate_gains)
from utb_download_audit import TransferPlan, changes, utc  # noqa: E402
from utb_math_reference import Bound, Reference, allocate  # noqa: E402


class TransferTests(unittest.TestCase):
    def test_out_of_order_duplicate_and_hash(self):
        data = bytes(range(250))*11
        plan = TransferPlan(('id', 'utc', len(data)), len(data))
        while (request := plan.request(plan.identity)) is not None:
            start, size = request
            packets = [(i, data[i:min(i+90, start+size)]) for i in range(start, start+size, 90)]
            for offset, payload in reversed(packets):
                plan.acknowledge(plan.identity, offset, payload)
            self.assertIsNone(plan.pending)
        self.assertEqual(plan.data, data)

    def test_missing_packet_retry_is_bounded(self):
        plan = TransferPlan('a', 180, chunk=180, retries=1)
        self.assertEqual(plan.request('a'), (0, 180))
        plan.acknowledge('a', 90, b'x'*90)
        self.assertEqual(plan.request('a'), (0, 180))
        with self.assertRaises(TimeoutError):
            plan.request('a')

    def test_identity_change_rejected(self):
        plan = TransferPlan('a', 100)
        with self.assertRaises(ValueError):
            plan.request('b')

    def test_zero_count_is_not_eof(self):
        plan = TransferPlan('a', 100)
        plan.request('a')
        with self.assertRaises(ValueError):
            plan.acknowledge('a', 0, b'')
        self.assertFalse(all(plan.seen))

    def test_conflicting_duplicate_rejected(self):
        plan = TransferPlan('a', 180)
        plan.request('a')
        plan.acknowledge('a', 0, b'x'*90)
        with self.assertRaises(ValueError):
            plan.acknowledge('a', 0, b'y'*90)

    def test_offset_outside_request_rejected(self):
        plan = TransferPlan('a', 180, chunk=90)
        plan.request('a')
        with self.assertRaises(ValueError):
            plan.acknowledge('a', 90, b'x'*90)

    def test_size_validation(self):
        for size, chunk in [(0, 90), (10, 0), (10, 91)]:
            with self.assertRaises(ValueError):
                TransferPlan('a', size, chunk=chunk)

    def test_markers_preserve_first_not_just_terminal(self):
        rows = [{'t': 1, 'v': 0}, {'t': 2, 'v': 0}, {'t': 3, 'v': 5}, {'t': 4, 'v': 2}, {'t': 5, 'v': 3}]
        self.assertEqual([r['t'] for r in changes(rows, 'v')], [1, 3, 4, 5])
        self.assertEqual(utc(0), '1970-01-01T00:00:00+00:00')


class PlantTests(unittest.TestCase):
    def test_zero_and_equal_thrust_cancel_torque(self):
        for q in [0, .5, 1]:
            force, torque = forces_and_torque([q]*4, Model())
            self.assertEqual(force, [3*q]*4)
            self.assertEqual(torque, [0, 0, 0])

    def test_individual_motor_signs(self):
        expected = [(-1, -1, 1), (-1, 1, -1), (1, -1, -1), (1, 1, 1)]
        for a, signs in enumerate(expected):
            q = [0]*4
            q[a] = 1
            _, torque = forces_and_torque(q, Model())
            self.assertEqual(tuple(1 if v > 0 else -1 for v in torque), signs)

    def test_allocator_to_physical_gain_and_cross_axes(self):
        model = Model()
        for a in range(3):
            command = [Bound(.1 if i == a else 0) for i in range(3)]
            result = allocate(command, .5, 18)
            _, torque = forces_and_torque([result['m'+str(i)].value for i in range(4)], model)
            gains = [math.sqrt(2)*model.arm*model.thrust_max]*2 + [2*model.reaction_length*model.thrust_max]
            for i in range(3):
                self.assertAlmostEqual(torque[i], gains[i]*(.1 if i == a else 0), places=14)

    def test_analytic_constant_torque(self):
        model = dataclasses.replace(Model(), motor_tau=0, damping=(0.,)*3, coupled=False)
        state = [0.]*7
        command = [.4, .4, .6, .6]
        _, torque = forces_and_torque(command, model)
        actual = advance(state, command, .01, model)
        self.assertAlmostEqual(actual[0], torque[0]/model.inertia[0]*.01, places=13)

    def test_analytic_damping(self):
        model = dataclasses.replace(Model(), motor_tau=0, coupled=False)
        state = advance([1., 0., 0.]+[.5]*4, [.5]*4, .01, model)
        self.assertAlmostEqual(state[0], math.exp(-.003/.002*.01), places=12)

    def test_analytic_motor_lag(self):
        model = dataclasses.replace(Model(), substeps=8)
        state = advance([0.]*7, [.5]*4, .0025, model)
        self.assertAlmostEqual(state[3], .5*(1-math.exp(-.0025/.03)), places=10)

    def test_torque_free_energy_and_momentum_norm(self):
        model = dataclasses.replace(Model(), damping=(0.,)*3)
        state = [1., 2., 3.]+[0.]*4

        def energy(x):
            return sum(j*w*w/2 for j, w in zip(model.inertia, x))

        def momentum(x):
            return sum((j*w)**2 for j, w in zip(model.inertia, x))

        before = state[:]
        for _ in range(400):
            state = advance(state, [0.]*4, DT, model)
        self.assertAlmostEqual(energy(before), energy(state), places=12)
        self.assertAlmostEqual(momentum(before), momentum(state), places=14)

    def test_passive_damping_energy_decreases(self):
        model = Model()
        state = [1., 2., 3.]+[0.]*4

        def energy(x):
            return sum(j*w*w/2 for j, w in zip(model.inertia, x))

        before = energy(state)
        for _ in range(100):
            state = advance(state, [0.]*4, DT, model)
        self.assertLess(energy(state), before)

    def test_invalid_inertias_rejected(self):
        for j in [(0, .002, .003), (-1, 1, 1), (.001, .001, .004), (float('nan'), .002, .003)]:
            with self.assertRaises(ValueError):
                Model(inertia=j)

    def test_invalid_delay_and_dt(self):
        for kwargs in [dict(sensor_cycles=-1), dict(compute_cycles=.5), dict(step=.02), dict(jitter_fraction=1),
                       dict(actuator_max=(1, 1, 1, -.1)), dict(motor_tau=-1)]:
            with self.assertRaises(ValueError):
                Model(**kwargs)

    def test_nonfinite_or_zero_plant_dt_rejected(self):
        for dt in [0, -1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                advance([0.]*7, [0.]*4, dt, Model())

    def test_local_exact_map_matches_rk4_plant(self):
        model = Model()
        x = [.01, .02, .003, .04, -.001]
        out = linear_step(x, model, 0)
        p, i, d, _, hz = GAINS[0]
        acc = (1-DT/(1/(2*math.pi*hz)+DT))*x[3]+(x[0]-x[4])/(1/(2*math.pi*hz)+DT)
        u = -p*x[0]+x[2]-i*x[0]*DT-d*acc
        allocated = allocate([Bound(u), Bound(0), Bound(0)], .5, 18)
        initial = allocate([Bound(x[1]), Bound(0), Bound(0)], .5, 18)
        state = [x[0], 0, 0]+[initial['m'+str(a)].value for a in range(4)]
        physical = advance(state, [allocated['m'+str(a)].value for a in range(4)], DT, model)
        self.assertAlmostEqual(out[0], physical[0], places=10)


class LoopTests(unittest.TestCase):
    def test_fully_failed_actuators(self):
        rows, _ = simulate(dataclasses.replace(Model(), actuator_max=(0.,)*4), seconds=1.1)
        self.assertTrue(all(r['q'+str(a)] == 0 for r in rows for a in range(4)))
        self.assertTrue(all(r['w'+str(a)] == 0 for r in rows for a in range(3)))
        self.assertGreater(rows[-1]['i0'], 0)
        self.assertEqual(rows[-1]['blocked0'], 0)  # Diagnostic feedback does not see physical failure.

    def test_zero_gains_do_not_move_model(self):
        rows, _ = simulate(seconds=1.1, gains=[(0, 0, 0, 0, 0)]*3)
        self.assertTrue(all(r['w0'] == r['w1'] == r['w2'] == 0 for r in rows))

    def test_zero_equilibrium_and_priming(self):
        rows, _ = simulate(kind='zero', seconds=.02)
        self.assertEqual(rows[0]['valid'], 0)
        self.assertEqual(rows[0]['prime'], 1)
        self.assertTrue(all(r['valid'] for r in rows[1:]))
        self.assertTrue(all(r['w0'] == r['w1'] == r['w2'] == 0 for r in rows))

    def test_reproducible_noise_and_jitter(self):
        model = dataclasses.replace(Model(), noise_sigma=.02, jitter_fraction=.2)
        self.assertEqual(simulate(model, seconds=.05), simulate(model, seconds=.05))
        self.assertNotEqual(simulate(model, seconds=.05, seed=2), simulate(model, seconds=.05, seed=3))

    def test_pid_reuses_reference_exactly(self):
        rows, inputs = simulate(seconds=1.05)
        reference = Reference()
        for row, inp in zip(rows, inputs):
            result = reference.step(inp)
            for name in ['p', 'i', 'd', 'raw', 'out']:
                for a in range(3):
                    self.assertEqual(row[name+str(a)], result[name+str(a)].value)

    def test_integrator_and_motor_bounds(self):
        rows, _ = simulate(kind='saturation', seconds=5)
        self.assertTrue(any(r['scaled'] for r in rows))
        self.assertTrue(any(r['blocked0'] for r in rows))
        self.assertTrue(all(abs(r['i'+str(a)]) <= GAINS[a][3] for r in rows for a in range(3)))
        self.assertTrue(all(0 <= r['q'+str(a)] <= 1 for r in rows for a in range(4)))

    def test_numerical_substep_convergence(self):
        base = dataclasses.replace(Model(), substeps=2)
        a, _ = simulate(base, kind='axes', seconds=2)
        b, _ = simulate(dataclasses.replace(base, substeps=8), kind='axes', seconds=2)
        self.assertLess(max(abs(x['w'+str(k)]-y['w'+str(k)]) for x, y in zip(a, b) for k in range(3)), 2e-7)

    def test_sine_has_no_false_step_metrics(self):
        rows, _ = simulate(kind='sine_2', seconds=2)
        result = metrics(rows, 'sine_2')
        self.assertIsNone(result['axes'][0]['rise_s'])
        self.assertIsNone(result['axes'][0]['settle_2percent_s'])

    def test_production_gain_guards(self):
        for g in [(6, 0, 0, 0, 0), (0, 0, .1, 0, 41), (0, 0, .1, 0, 0), (0, 0, 0, 2, 0)]:
            with self.assertRaises(ValueError):
                validate_gains([g]*3)
        validate_gains([(0, 0, 0, 0, 0)]*3)


@unittest.skipUnless(os.environ.get('UTB_AUDIT_EVIDENCE'), 'Set UTB_AUDIT_EVIDENCE to new offline evidence.json')
class ArchiveRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evidence = json.loads(Path(os.environ['UTB_AUDIT_EVIDENCE']).read_text())

    def test_unhealthy_message_during_full_download(self):
        log = self.evidence['tlogs']['full-download.tlog']
        first, last = log['data']['first'][0], log['data']['last'][0]
        unhealthy = [m for m in log['statustext'] if 'EKF3 core 0 unhealthy' in m['text']]
        self.assertTrue(any(first < m['host_s'] < last for m in unhealthy))

    def test_startup_losses_precede_running(self):
        log = self.evidence['bins']['SYNTHETIC_FULL_400.bin']
        running = next(r['TimeUS'] for r in log['UTBX_status_changes'] if r['Test'] == 2)
        for key, maximum in [('22', 402), ('23', 130), ('29', 6011)]:
            rows = log['UBST_changes'][key]
            self.assertTrue(all(r['TimeUS'] < running for r in rows))
            self.assertEqual(rows[-1]['Val'], maximum)

    def test_complete_marker_matches_original_report(self):
        rows = self.evidence['bins']['SYNTHETIC_FULL_400.bin']['UTBX_status_changes']
        complete = next(r for r in rows if r['Test'] == 3)
        self.assertEqual((complete['TimeUS'], complete['Cycle']), (218037671, 8000))

    def test_recovered_payload_total(self):
        logs = self.evidence['tlogs']
        self.assertEqual(sum(logs[n]['data']['received_payload_bytes'] for n in
                             ['short-download.tlog', 'short-download-recovery.tlog']), 2569496)

    def test_coldboot_restores_health_and_no_transmitted_requests_fabricated(self):
        logs = self.evidence['tlogs']
        self.assertEqual(logs['final-readback.tlog']['ekf_last']['flags'], 1024)
        self.assertEqual(logs['final-coldboot-readback.tlog']['ekf_last']['flags'], 167)
        self.assertEqual(logs['full-download.tlog']['counts'].get('LOG_REQUEST_DATA', 0), 0)


if __name__ == '__main__':
    unittest.main()
