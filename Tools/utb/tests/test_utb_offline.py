#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline model/transfer tests; no connections, simulator vehicle or firmware writes."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_offline.py
# @brief      Unit regrese: Offline model/transfer tests; no connections, simulator vehicle or firmware writes.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import dataclasses
import json
import math
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from utb_closed_loop import (DT, GAINS, Model, advance, forces_and_torque, linear_step, metrics, simulate,  # noqa: E402
                             validate_gains)
from utb_download_audit import TransferPlan, changes, utc  # noqa: E402
from utb_math_reference import Bound, Reference, allocate  # noqa: E402


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class TransferTests(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify out of order duplicate and hash.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: TransferPlan, self.assertEqual, bytes, len.
    # --------------------------------------------------------------------------------------------------------------------
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

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify missing packet retry is bounded.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: TransferPlan, self.assertEqual, plan.acknowledge, plan.request.
    # --------------------------------------------------------------------------------------------------------------------
    def test_missing_packet_retry_is_bounded(self):
        plan = TransferPlan('a', 180, chunk=180, retries=1)
        self.assertEqual(plan.request('a'), (0, 180))
        plan.acknowledge('a', 90, b'x'*90)
        self.assertEqual(plan.request('a'), (0, 180))
        with self.assertRaises(TimeoutError):
            plan.request('a')

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify identity change rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: TransferPlan, self.assertRaises, plan.request.
    # --------------------------------------------------------------------------------------------------------------------
    def test_identity_change_rejected(self):
        plan = TransferPlan('a', 100)
        with self.assertRaises(ValueError):
            plan.request('b')

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify zero count is not eof.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: TransferPlan, plan.request, self.assertFalse, self.assertRaises.
    # --------------------------------------------------------------------------------------------------------------------
    def test_zero_count_is_not_eof(self):
        plan = TransferPlan('a', 100)
        plan.request('a')
        with self.assertRaises(ValueError):
            plan.acknowledge('a', 0, b'')
        self.assertFalse(all(plan.seen))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify conflicting duplicate rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: TransferPlan, plan.request, plan.acknowledge, self.assertRaises.
    # --------------------------------------------------------------------------------------------------------------------
    def test_conflicting_duplicate_rejected(self):
        plan = TransferPlan('a', 180)
        plan.request('a')
        plan.acknowledge('a', 0, b'x'*90)
        with self.assertRaises(ValueError):
            plan.acknowledge('a', 0, b'y'*90)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify offset outside request rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: TransferPlan, plan.request, self.assertRaises, plan.acknowledge.
    # --------------------------------------------------------------------------------------------------------------------
    def test_offset_outside_request_rejected(self):
        plan = TransferPlan('a', 180, chunk=90)
        plan.request('a')
        with self.assertRaises(ValueError):
            plan.acknowledge('a', 90, b'x'*90)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify size validation.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertRaises, TransferPlan.
    # --------------------------------------------------------------------------------------------------------------------
    def test_size_validation(self):
        for size, chunk in [(0, 90), (10, 0), (10, 91)]:
            with self.assertRaises(ValueError):
                TransferPlan('a', size, chunk=chunk)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify markers preserve first not just terminal.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertEqual, utc, changes.
    # --------------------------------------------------------------------------------------------------------------------
    def test_markers_preserve_first_not_just_terminal(self):
        rows = [{'t': 1, 'v': 0}, {'t': 2, 'v': 0}, {'t': 3, 'v': 5}, {'t': 4, 'v': 2}, {'t': 5, 'v': 3}]
        self.assertEqual([r['t'] for r in changes(rows, 'v')], [1, 3, 4, 5])
        self.assertEqual(utc(0), '1970-01-01T00:00:00+00:00')


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class PlantTests(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify zero and equal thrust cancel torque.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: forces_and_torque, self.assertEqual, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_zero_and_equal_thrust_cancel_torque(self):
        for q in [0, .5, 1]:
            force, torque = forces_and_torque([q]*4, Model())
            self.assertEqual(force, [3*q]*4)
            self.assertEqual(torque, [0, 0, 0])

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify individual motor signs.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: enumerate, forces_and_torque, self.assertEqual, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_individual_motor_signs(self):
        expected = [(-1, -1, 1), (-1, 1, -1), (1, -1, -1), (1, 1, 1)]
        for a, signs in enumerate(expected):
            q = [0]*4
            q[a] = 1
            _, torque = forces_and_torque(q, Model())
            self.assertEqual(tuple(1 if v > 0 else -1 for v in torque), signs)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify allocator to physical gain and cross axes.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: Model, range, allocate, forces_and_torque.
    # --------------------------------------------------------------------------------------------------------------------
    def test_allocator_to_physical_gain_and_cross_axes(self):
        model = Model()
        for a in range(3):
            command = [Bound(.1 if i == a else 0) for i in range(3)]
            result = allocate(command, .5, 18)
            _, torque = forces_and_torque([result['m'+str(i)].value for i in range(4)], model)
            gains = [math.sqrt(2)*model.arm*model.thrust_max]*2 + [2*model.reaction_length*model.thrust_max]
            for i in range(3):
                self.assertAlmostEqual(torque[i], gains[i]*(.1 if i == a else 0), places=14)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify analytic constant torque.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dataclasses.replace, forces_and_torque, advance, self.assertAlmostEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_analytic_constant_torque(self):
        model = dataclasses.replace(Model(), motor_tau=0, damping=(0.,)*3, coupled=False)
        state = [0.]*7
        command = [.4, .4, .6, .6]
        _, torque = forces_and_torque(command, model)
        actual = advance(state, command, .01, model)
        self.assertAlmostEqual(actual[0], torque[0]/model.inertia[0]*.01, places=13)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify analytic damping.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dataclasses.replace, advance, self.assertAlmostEqual, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_analytic_damping(self):
        model = dataclasses.replace(Model(), motor_tau=0, coupled=False)
        state = advance([1., 0., 0.]+[.5]*4, [.5]*4, .01, model)
        self.assertAlmostEqual(state[0], math.exp(-.003/.002*.01), places=12)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify analytic motor lag.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dataclasses.replace, advance, self.assertAlmostEqual, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_analytic_motor_lag(self):
        model = dataclasses.replace(Model(), substeps=8)
        state = advance([0.]*7, [.5]*4, .0025, model)
        self.assertAlmostEqual(state[3], .5*(1-math.exp(-.0025/.03)), places=10)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify torque free energy and momentum norm.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dataclasses.replace, range, self.assertAlmostEqual, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_torque_free_energy_and_momentum_norm(self):
        model = dataclasses.replace(Model(), damping=(0.,)*3)
        state = [1., 2., 3.]+[0.]*4

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # End Public functions
        # ===================================================================================================================
        # Private functions
        # -------------------------------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------------------------------
        # @brief      Energy.
        #
        # @param[in]  x X.
        # @return     Returned values: sum((j * w * w / 2 for j, w in zip(model.inertia, x))).
        #
        # @details    Local nested helper. Calls: sum, zip.
        # -------------------------------------------------------------------------------------------------------------------
        def energy(x):
            return sum(j*w*w/2 for j, w in zip(model.inertia, x))

        # -------------------------------------------------------------------------------------------------------------------
        # @brief      Momentum.
        #
        # @param[in]  x X.
        # @return     Returned values: sum(((j * w) ** 2 for j, w in zip(model.inertia, x))).
        #
        # @details    Local nested helper. Calls: sum, zip.
        # -------------------------------------------------------------------------------------------------------------------
        def momentum(x):
            return sum((j*w)**2 for j, w in zip(model.inertia, x))

        before = state[:]
        for _ in range(400):
            state = advance(state, [0.]*4, DT, model)
        self.assertAlmostEqual(energy(before), energy(state), places=12)
        self.assertAlmostEqual(momentum(before), momentum(state), places=14)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Private functions
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify passive damping energy decreases.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: Model, energy, range, self.assertLess.
    # --------------------------------------------------------------------------------------------------------------------
    def test_passive_damping_energy_decreases(self):
        model = Model()
        state = [1., 2., 3.]+[0.]*4

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # End Public functions
        # ===================================================================================================================
        # Private functions
        # -------------------------------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------------------------------
        # @brief      Energy.
        #
        # @param[in]  x X.
        # @return     Returned values: sum((j * w * w / 2 for j, w in zip(model.inertia, x))).
        #
        # @details    Local nested helper. Calls: sum, zip.
        # -------------------------------------------------------------------------------------------------------------------
        def energy(x):
            return sum(j*w*w/2 for j, w in zip(model.inertia, x))

        before = energy(state)
        for _ in range(100):
            state = advance(state, [0.]*4, DT, model)
        self.assertLess(energy(state), before)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Private functions
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify invalid inertias rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: float, self.assertRaises, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_invalid_inertias_rejected(self):
        for j in [(0, .002, .003), (-1, 1, 1), (.001, .001, .004), (float('nan'), .002, .003)]:
            with self.assertRaises(ValueError):
                Model(inertia=j)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify invalid delay and dt.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dict, self.assertRaises, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_invalid_delay_and_dt(self):
        for kwargs in [dict(sensor_cycles=-1), dict(compute_cycles=.5), dict(step=.02), dict(jitter_fraction=1),
                       dict(actuator_max=(1, 1, 1, -.1)), dict(motor_tau=-1)]:
            with self.assertRaises(ValueError):
                Model(**kwargs)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify nonfinite or zero plant dt rejected.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: float, self.assertRaises, advance, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_nonfinite_or_zero_plant_dt_rejected(self):
        for dt in [0, -1, float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                advance([0.]*7, [0.]*4, dt, Model())

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify local exact map matches rk4 plant.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: Model, linear_step, allocate, advance.
    # --------------------------------------------------------------------------------------------------------------------
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


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class LoopTests(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify fully failed actuators.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: simulate, self.assertTrue, self.assertGreater, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_fully_failed_actuators(self):
        rows, _ = simulate(dataclasses.replace(Model(), actuator_max=(0.,)*4), seconds=1.1)
        self.assertTrue(all(r['q'+str(a)] == 0 for r in rows for a in range(4)))
        self.assertTrue(all(r['w'+str(a)] == 0 for r in rows for a in range(3)))
        self.assertGreater(rows[-1]['i0'], 0)
        self.assertEqual(rows[-1]['blocked0'], 0)  # Diagnostic feedback does not see physical failure.

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify zero gains do not move model.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: simulate, self.assertTrue, all.
    # --------------------------------------------------------------------------------------------------------------------
    def test_zero_gains_do_not_move_model(self):
        rows, _ = simulate(seconds=1.1, gains=[(0, 0, 0, 0, 0)]*3)
        self.assertTrue(all(r['w0'] == r['w1'] == r['w2'] == 0 for r in rows))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify zero equilibrium and priming.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: simulate, self.assertEqual, self.assertTrue, all.
    # --------------------------------------------------------------------------------------------------------------------
    def test_zero_equilibrium_and_priming(self):
        rows, _ = simulate(kind='zero', seconds=.02)
        self.assertEqual(rows[0]['valid'], 0)
        self.assertEqual(rows[0]['prime'], 1)
        self.assertTrue(all(r['valid'] for r in rows[1:]))
        self.assertTrue(all(r['w0'] == r['w1'] == r['w2'] == 0 for r in rows))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify reproducible noise and jitter.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dataclasses.replace, self.assertEqual, self.assertNotEqual, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_reproducible_noise_and_jitter(self):
        model = dataclasses.replace(Model(), noise_sigma=.02, jitter_fraction=.2)
        self.assertEqual(simulate(model, seconds=.05), simulate(model, seconds=.05))
        self.assertNotEqual(simulate(model, seconds=.05, seed=2), simulate(model, seconds=.05, seed=3))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify pid reuses reference exactly.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: simulate, Reference, zip, reference.step.
    # --------------------------------------------------------------------------------------------------------------------
    def test_pid_reuses_reference_exactly(self):
        rows, inputs = simulate(seconds=1.05)
        reference = Reference()
        for row, inp in zip(rows, inputs):
            result = reference.step(inp)
            for name in ['p', 'i', 'd', 'raw', 'out']:
                for a in range(3):
                    self.assertEqual(row[name+str(a)], result[name+str(a)].value)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify integrator and motor bounds.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: simulate, self.assertTrue, any, all.
    # --------------------------------------------------------------------------------------------------------------------
    def test_integrator_and_motor_bounds(self):
        rows, _ = simulate(kind='saturation', seconds=5)
        self.assertTrue(any(r['scaled'] for r in rows))
        self.assertTrue(any(r['blocked0'] for r in rows))
        self.assertTrue(all(abs(r['i'+str(a)]) <= GAINS[a][3] for r in rows for a in range(3)))
        self.assertTrue(all(0 <= r['q'+str(a)] <= 1 for r in rows for a in range(4)))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify numerical substep convergence.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dataclasses.replace, simulate, self.assertLess, Model.
    # --------------------------------------------------------------------------------------------------------------------
    def test_numerical_substep_convergence(self):
        base = dataclasses.replace(Model(), substeps=2)
        a, _ = simulate(base, kind='axes', seconds=2)
        b, _ = simulate(dataclasses.replace(base, substeps=8), kind='axes', seconds=2)
        self.assertLess(max(abs(x['w'+str(k)]-y['w'+str(k)]) for x, y in zip(a, b) for k in range(3)), 2e-7)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify sine has no false step metrics.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: simulate, metrics, self.assertIsNone.
    # --------------------------------------------------------------------------------------------------------------------
    def test_sine_has_no_false_step_metrics(self):
        rows, _ = simulate(kind='sine_2', seconds=2)
        result = metrics(rows, 'sine_2')
        self.assertIsNone(result['axes'][0]['rise_s'])
        self.assertIsNone(result['axes'][0]['settle_2percent_s'])

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify production gain guards.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: validate_gains, self.assertRaises.
    # --------------------------------------------------------------------------------------------------------------------
    def test_production_gain_guards(self):
        for g in [(6, 0, 0, 0, 0), (0, 0, .1, 0, 41), (0, 0, .1, 0, 0), (0, 0, 0, 2, 0)]:
            with self.assertRaises(ValueError):
                validate_gains([g]*3)
        validate_gains([(0, 0, 0, 0, 0)]*3)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
@unittest.skipUnless(os.environ.get('UTB_AUDIT_EVIDENCE'), 'Set UTB_AUDIT_EVIDENCE to new offline evidence.json')
class ArchiveRegressionTests(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Set up class.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method. Calls: json.loads, Path(os.environ['UTB_AUDIT_EVIDENCE']).read_text, Path.
    # --------------------------------------------------------------------------------------------------------------------
    @classmethod
    def setUpClass(cls):
        cls.evidence = json.loads(Path(os.environ['UTB_AUDIT_EVIDENCE']).read_text())

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unhealthy message during full download.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertTrue, any.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unhealthy_message_during_full_download(self):
        log = self.evidence['tlogs']['full-download.tlog']
        first, last = log['data']['first'][0], log['data']['last'][0]
        unhealthy = [m for m in log['statustext'] if 'EKF3 core 0 unhealthy' in m['text']]
        self.assertTrue(any(first < m['host_s'] < last for m in unhealthy))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify startup losses precede running.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: next, self.assertTrue, self.assertEqual, all.
    # --------------------------------------------------------------------------------------------------------------------
    def test_startup_losses_precede_running(self):
        log = self.evidence['bins']['SYNTHETIC_FULL_400.bin']
        running = next(r['TimeUS'] for r in log['UTBX_status_changes'] if r['Test'] == 2)
        for key, maximum in [('22', 402), ('23', 130), ('29', 6011)]:
            rows = log['UBST_changes'][key]
            self.assertTrue(all(r['TimeUS'] < running for r in rows))
            self.assertEqual(rows[-1]['Val'], maximum)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify complete marker matches original report.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: next, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_complete_marker_matches_original_report(self):
        rows = self.evidence['bins']['SYNTHETIC_FULL_400.bin']['UTBX_status_changes']
        complete = next(r for r in rows if r['Test'] == 3)
        self.assertEqual((complete['TimeUS'], complete['Cycle']), (218037671, 8000))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify recovered payload total.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertEqual, sum.
    # --------------------------------------------------------------------------------------------------------------------
    def test_recovered_payload_total(self):
        logs = self.evidence['tlogs']
        self.assertEqual(sum(logs[n]['data']['received_payload_bytes'] for n in
                             ['short-download.tlog', 'short-download-recovery.tlog']), 2569496)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify coldboot restores health and no transmitted requests fabricated.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.assertEqual, logs['full-download.tlog']['counts'].get.
    # --------------------------------------------------------------------------------------------------------------------
    def test_coldboot_restores_health_and_no_transmitted_requests_fabricated(self):
        logs = self.evidence['tlogs']
        self.assertEqual(logs['final-readback.tlog']['ekf_last']['flags'], 1024)
        self.assertEqual(logs['final-coldboot-readback.tlog']['ekf_last']['flags'], 167)
        self.assertEqual(logs['full-download.tlog']['counts'].get('LOG_REQUEST_DATA', 0), 0)


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
