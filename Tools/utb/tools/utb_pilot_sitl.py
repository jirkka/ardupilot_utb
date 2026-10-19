#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Finite actual pilot ACRO/ANGLE SITL experiments, with no hardware endpoint."""
# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses remain unchanged.
# @file utb_pilot_sitl.py
# @brief Samostatný ACRO a přímé ACRO/ANGLE přechody ve skutečném lokálním SITL.
# @author Jiri Kovar
# @version 2026-10-18
# @date 2026-10-18
# --------------------------------------------------------------------------------------------------------------------
# Include Files
# Standalone scenarios below target historical2G; current3A runner: utb_native_sitl.py.
import argparse
import hashlib
from functools import partial
import json
import math
from pathlib import Path

from pymavlink import mavutil
import utb_angle_sitl as angle
# End Include Files
# Static data declarations
PARAMS = ('UTB_C_ENABLE 1\nUTB_C_RMAX 0.15\nUTB_C_PMAX 0.15\nUTB_C_YMAX 0.05\nUTB_C_AW 1\n'
          'UTB_A_AW 1\nUTB_A_RMAX 0.3\nFLTMODE3 43\nFS_GCS_ENABLE 1\nFS_THR_ENABLE 1\nFS_OPTIONS 0\n')
# End Static data declarations
# Data type, constant, macro definition


# End Data type, constant, macro definition
# Public functions


# ----------------------------------------------------------------------------------------------------
# @brief Launch one owned bfxrev; preserve all failed evidence and complete parameter provenance.
#
# @param[in] binary Lokální executable vlastněného SITL.
# @param[in] output Samostatný adresář důkazů, nepřepisuje historii.
# @param[in] name Jednoznačný název scénáře.
# @param[in] action Callback konečného testu.
# @param[in] extra Dodatečné SITL parametry.
# @param[in] rcmap Skutečné AP RCMAP pořadí kanálů.
# @param[in] speedup Poměr simulačního a host času; původní default 1.
# @param[in] model Existující model SITL, případně explicitní JSON setrvačnosti.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def launch(binary, output, name, action, extra='', rcmap=(1, 2, 4, 3), speedup=1, model='bfxrev'):
    """Launch one owned bfxrev; preserve all failed evidence and complete parameter provenance."""
    return angle.launch(binary.resolve(), output.resolve(), name, action, extra=PARAMS + extra,
                        rcmap=rcmap, speedup=speedup, model=model)


# ----------------------------------------------------------------------------------------------------
# @brief Request the native GCS or FLTMODE switch without modifying sticks or throttle.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @param[in] mode Skutečné číslo AP režimu.
# @param[in] rc True pro standardní FLTMODE výběr přes RC.
# @param[in] armed Očekávaný skutečný armed stav.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def switch(sim, mode, rc=False, armed=True):
    """Request the native GCS or FLTMODE switch without modifying sticks or throttle."""
    sim.marks.append(dict(boot_ms=sim.last_boot_ms, label=f'switch_to_{mode}', rc=rc,
                          roll=sim.roll, pitch=sim.pitch, yaw=sim.yaw, throttle=sim.throttle))
    if rc:
        sim.expected_mode = None
        sim.rc5 = {0: 1000, 42: 1300, 43: 1450}[mode]
        sim.wait('HEARTBEAT', lambda m: m.custom_mode == mode and bool(m.base_mode & 128) == armed)
        sim.expected_mode = mode
    else:
        sim.select(mode, armed=armed, settle=False)


# ----------------------------------------------------------------------------------------------------
# @brief Check native selection, both stick directions, throttle, historical mode40 and fault latch.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def disarmed(sim):
    """Check native selection, both stick directions, throttle, historical mode40 and fault latch."""
    sim.param('UTB_C_ENABLE', 0)
    sim.mode(43, 0)
    sim.param('UTB_C_ENABLE', 1)
    for mode in (43, 42, 43):
        switch(sim, mode, rc=(mode == 42), armed=False)
        sim.sticks(seconds=.5)
        for axis in ('roll', 'pitch', 'yaw'):
            for sign in (-1, 1):
                sim.sticks(**{axis: sign}, seconds=.5, label=f'{mode}_{axis}_{sign}')
        sim.sticks(1, 1, 1, seconds=.5)
        sim.sticks(seconds=.5)
        for throttle in (1000, 1500, 2000, 1000):
            sim.throttle = throttle
            sim.duration(.5)
    switch(sim, 0, rc=True, armed=False)
    sim.select(40)
    assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
    assert sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.select(0)
    sim.select(43)
    sim.expected_mode = None
    sim.roll = 2100
    sim.duration(1)
    sim.roll = 1500
    sim.duration(1)
    assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.select(0)
    sim.select(43)
    sim.duration(1)
    sim.select(0)


# ----------------------------------------------------------------------------------------------------
# @brief Require actual mode43 identity, exact linear rate map and zero disarmed output.
#
# @param[in] records Skutečné dekódované BIN záznamy.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def analyse_disarmed(records):
    """Require actual mode43 identity, exact linear rate map and zero disarmed output."""
    assert records['UARC'] and records['UAMT']
    assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
    assert all(max(r[f'C{i}'] for i in range(1, 5)) <= 1000 for r in records['UAMT'])
    assert any(r['Stop'] == 2 and r['RCF'] == 5 for r in records['UARC'])
    pilot = {(r['TimeUS'], r['Seq']): r for r in records['UARC']}
    acro = [r for r in records['UAAT'] if r['Src'] == 0 and pilot[(r['TimeUS'], r['Seq'])]['V']]
    assert acro
    for r in acro:
        p = pilot[(r['TimeUS'], r['Seq'])]
        assert p['Md'] == 43
        for target, stick, limit in zip(('RR', 'PR', 'YR'), ('NR', 'NP', 'NY'), (.15, .15, .05)):
            assert abs(r[target] - p[stick] * limit) < 1e-6
    for label in ('NR', 'NP', 'NY'):
        values = [pilot[(r['TimeUS'], r['Seq'])][label] for r in acro]
        assert min(values) < -.99 and max(values) > .99
    return dict(disarmed=True, linear_rc=True, disarmed_output_zero=True, samples=len(acro), mode40_rejected=True)


# ----------------------------------------------------------------------------------------------------
# @brief Reject unsupported policy/config at entry and latch runtime mutations while disarmed.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def guards(sim):
    """Reject unsupported policy/config at entry and latch runtime mutations while disarmed."""
    # The unselected controller's opt-in/limits must not be an entry dependency.
    sim.param('UTB_A_ENABLE', 0)
    sim.param('UTB_A_ANGLE', 100)
    sim.param('UTB_A_RMAX', 1)
    sim.select(43)
    sim.duration(1)
    sim.select(0)
    sim.param('UTB_A_ENABLE', 1)
    sim.param('UTB_A_ANGLE', 5)
    sim.param('UTB_A_RMAX', .3)
    sim.param('UTB_C_ENABLE', 0)
    sim.param('UTB_C_RMAX', .2)
    sim.select(42)
    sim.duration(1)
    sim.select(0)
    sim.param('UTB_C_ENABLE', 1)
    sim.param('UTB_C_RMAX', .15)
    for mode in (42, 43):
        invalid_entries = [('FS_OPTIONS', 16, 0), ('FS_GCS_ENABLE', 0, 1), ('FS_THR_ENABLE', 0, 1),
                           ('FSTRATE_ENABLE', 1, 0)]
        invalid_entries.append(('UTB_C_RMAX', .2, .15) if mode == 43 else ('UTB_A_RMAX', .4, .3))
        for parameter, invalid, correct in invalid_entries:
            sim.param(parameter, invalid)
            sim.mode(mode, 0)
            sim.param(parameter, correct)
        for parameter, invalid, correct in (('FS_OPTIONS', 16, 0), ('RC1_DZ', 600, 30),
                                            ('UTB_SHADOW', 1, 0), ('FSTRATE_ENABLE', 1, 0)):
            sim.select(mode)
            sim.duration(.5)
            sim.expected_mode = None
            start = len(sim.events)
            sim.param(parameter, invalid)
            sim.duration(1)
            assert any(' stop=' in e['text'] for e in sim.events[start:]), parameter
            sim.param(parameter, correct)
            sim.duration(.5)
            assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
            sim.select(0)
    sim.select(43)
    sim.duration(1)
    sim.expected_mode = None
    sim.param('EK3_PRIMARY', 1)
    sim.wait('STATUSTEXT', lambda m: m.text.startswith('UTB ACRO stop='), timeout=15)
    sim.param('EK3_PRIMARY', 0)
    sim.select(0)


# ----------------------------------------------------------------------------------------------------
# @brief Finite signed RC rate pulses, center hold at nonzero tilt, and explicit ANGLE recovery.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @param[in] axis Testovaná tělová osa.
# @param[in] baseline True pro původní AP ACRO.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def acro_flight(sim, axis='roll', baseline=False):
    """Finite signed RC rate pulses, center hold at nonzero tilt, and explicit ANGLE recovery."""
    angle.prepare_flight(sim)
    index = ('roll', 'pitch', 'yaw').index(axis)
    for sign in (1, -1):
        sim.select(1 if baseline else 43, armed=True)
        sim.sticks(seconds=1, label='initial_center')
        sim.sticks(**{axis: sign}, seconds=1 if index < 2 else 2, label=f'{axis}_{sign}')
        sim.sticks(seconds=1.2, label='zero_rate_nonzero_attitude')
        # Recovery is a separate explicitly selected attitude mode, never hidden ACRO leveling.
        sim.select(0 if baseline else 42, armed=True)
        sim.sticks(seconds=3, label='explicit_level')
    angle.finish_flight(sim)


# ----------------------------------------------------------------------------------------------------
# @brief Actual direct switches with unchanged nonzero sticks, throttle and existing nonzero I.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @param[in] pattern Samostatný krátký přechodový scénář roll/pitch/yaw/combined/throttle.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def transitions(sim, pattern='roll'):
    """Actual direct switches with unchanged nonzero sticks, throttle and existing nonzero I."""
    angle.prepare_flight(sim)
    sim.select(42, armed=True)
    sim.sticks(seconds=2, label='center')
    patterns = dict(roll=(.3, 0, 0), pitch=(0, -.3, 0), yaw=(0, 0, .5), combined=(.3, -.3, .5))
    if pattern in patterns:
        values = patterns[pattern]
        sim.sticks(*values, seconds=1, label='nonzero_before_switch')
        switch(sim, 43)
        sim.sticks(*values, seconds=.6, label='acro_nonzero')
        switch(sim, 42, rc=True)
        sim.sticks(*values, seconds=.6, label='angle_nonzero')
        switch(sim, 43, rc=True)
        sim.sticks(*values, seconds=.6, label='acro_repeated')
        switch(sim, 42)
        sim.sticks(seconds=2, label='level')
    else:
        assert pattern == 'throttle'
        # Separate bounded flight: manual throttle changes never hide an altitude controller.
        for throttle in (1475, 1469, 1472):
            sim.throttle = throttle
            sim.sticks(seconds=.3)
            switch(sim, 43)
            sim.sticks(seconds=.3)
            switch(sim, 42)
            sim.sticks(seconds=.3)
    angle.finish_flight(sim)


# ----------------------------------------------------------------------------------------------------
# @brief Lose actual RC or actual GCS contact and require native RTL, never accept a falling fault-stop.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @param[in] source Odděleně ztracený RC/GCS zdroj.
# @param[in] mode Skutečné číslo AP režimu.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def failsafe(sim, source='rc', mode=43):
    """Lose actual RC or actual GCS contact and require native RTL, never accept a falling fault-stop."""
    angle.prepare_flight(sim)
    sim.select(mode, armed=True)
    sim.sticks(seconds=2)
    sim.expected_mode = None
    sim.bench_altitude = None
    if source == 'rc':
        sim.param('SIM_RC_FAIL', 1)
        sim.send_rc = False
    else:
        sim.enable_native_receiver()
        sim.send_heartbeat = False
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 6, timeout=10)
    sim.duration(2)
    sim.send_rc = sim.send_heartbeat = True
    sim.native_rc = False
    if source == 'rc':
        sim.param('SIM_RC_FAIL', 0)
    sim.duration(2)
    assert sim.wait('HEARTBEAT').custom_mode == 6, 'Automatic pilot reentry after reconnect'
    angle.finish_flight(sim)
    sim.throttle = 1000
    sim.sticks(seconds=1, label='explicit_disarmed_reentry_after_failsafe')
    sim.select(mode)
    sim.duration(1)
    sim.select(0)


# ----------------------------------------------------------------------------------------------------
# @brief Split complete controller evidence by actual mode entry, requiring I/D/output reset and one owner.
#
# @param[in] records Skutečné dekódované BIN záznamy.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def windows(records):
    """Split complete controller evidence by actual mode entry, requiring I/D/output reset and one owner."""
    output = []
    modes = records['MODE']
    for i, start in enumerate(modes):
        if start['ModeNum'] not in (42, 43):
            continue
        end = modes[i + 1]['TimeUS'] if i + 1 < len(modes) else math.inf
        part = {kind: rows if kind == 'PARM' else
                [r for r in rows if start['TimeUS'] < r.get('TimeUS', 0) <= end]
                for kind, rows in records.items()}
        # Mode receipt can be shorter than two seconds in transition experiments; all samples are still checked.
        analysis, rows = angle.analyse_flight(part, pilot_mode=start['ModeNum'], minimum_samples=100)
        assert rows[0]['seq'] == 1
        assert all(rows[0][f'{field}{axis}'] == 0 for field in ('I', 'D', 'Out') for axis in range(3))
        output.append(dict(mode=start['ModeNum'], start=start['TimeUS'], analysis=analysis, rows=rows))
    assert output
    return output


# ----------------------------------------------------------------------------------------------------
# @brief Prove the linear raw=target mapping and zero-rate tilt hold from actual linked samples.
#
# @param[in] parts Úplné vzorky rozdělené skutečnými MODE časovými značkami.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def validate_acro(parts):
    """Prove the linear raw=target mapping and zero-rate tilt hold from actual linked samples."""
    from utb_active_sitl_analysis import step_metrics
    results = []
    for part in parts:
        if part['mode'] != 43:
            continue
        rows = part['rows']
        for r in rows:
            for axis, name, limit in zip(range(3), ('roll', 'pitch', 'yaw'), (.15, .15, .05)):
                assert abs(r[f'Tar{axis}'] - r[f'normalized_{name}'] * limit) < 1e-6
        steps = []
        for axis in range(3):
            active = [r for r in rows if abs(r[f'Tar{axis}']) > .01]
            if not active:
                continue
            converted = [dict(t=r['t'], **{f'ref{axis}': r[f'Tar{axis}'],
                                           f'target{axis}': r[f'Tar{axis}'], f'gyro{axis}': r[f'Gyr{axis}']})
                         for r in active]
            metric = step_metrics(converted, axis)
            assert metric['rise_10_90_s'] is not None
            errors = [abs(r[f'Tar{axis}'] - r[f'Gyr{axis}']) for r in rows]
            metric.update(axis=axis, iae=sum(e * r['dt'] for e, r in zip(errors, rows)),
                          itae=sum(e * r['dt'] * r['t'] for e, r in zip(errors, rows)),
                          max_pid=max(abs(r[f'Out{axis}']) for r in rows),
                          max_i=max(abs(r[f'I{axis}']) for r in rows))
            if axis < 2:
                hold = [r for r in rows if r['t'] >= rows[-1]['t'] - .4]
                assert all(r[f'Tar{axis}'] == 0 for r in hold)
                angle_mean = sum(r[f'angle{axis}'] for r in hold) / len(hold)
                gyro_mean = sum(r[f'Gyr{axis}'] for r in hold) / len(hold)
                assert abs(angle_mean) > math.radians(2), 'No nonzero tilt evidence'
                assert abs(gyro_mean) < .015, 'Rate not stopped at center'
                metric.update(zero_rate_tilt_deg=math.degrees(angle_mean), zero_rate_gyro=gyro_mean,
                              hold_drift_deg=math.degrees(hold[-1][f'angle{axis}'] - hold[0][f'angle{axis}']))
            steps.append(metric)
        results.append(dict(start=part['start'], steps=steps,
                            max_tilt_deg=max(math.degrees(max(abs(r['angle0']), abs(r['angle1']))) for r in rows)))
    assert len(results) == 2
    return results


# ----------------------------------------------------------------------------------------------------
# @brief Demonstrate normal rudder arm/spool/disarm in standalone ACRO without a GUIDED precursor.
#
# @param[in] sim Vlastněný lokální SITL, žádné fyzické zařízení.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def ground(sim):
    """Demonstrate normal rudder arm/spool/disarm in standalone ACRO without a GUIDED precursor."""
    sim.select(43)
    sim.yaw = 2000
    sim.wait('HEARTBEAT', lambda m: bool(m.base_mode & 128), timeout=10)
    sim.yaw = 1500
    sim.duration(1)
    sim.throttle = 1100
    sim.duration(2)
    sim.throttle = 1000
    sim.duration(2)
    sim.yaw = 1000
    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128, timeout=10)
    sim.yaw = 1500
    sim.duration(1)
    sim.select(0)


# ----------------------------------------------------------------------------------------------------
# @brief Reject active test authorization when disarmed evidence is absent or belongs to another executable.
#
# @param[in] binary Lokální executable vlastněného SITL.
# @param[in] output Samostatný adresář důkazů, nepřepisuje historii.
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def require_gate(binary, output):
    """Reject active test authorization when disarmed evidence is absent or belongs to another executable."""
    gate = output / 'disarmed/analysis.json'
    assert gate.exists(), 'Actual DISARMED PASS required'
    evidence = json.loads(gate.read_text())
    assert evidence['disarmed'], 'Actual DISARMED PASS required'
    assert evidence['binary_sha256'] == hashlib.sha256(binary.read_bytes()).hexdigest(), 'Gate binary mismatch'


# ----------------------------------------------------------------------------------------------------
# @brief Ověřit skutečný AP parameter roundtrip a jinou povolenou rychlostní mapu bez armingu.
# @param[in] sim Vlastněný lokální simulátor.
# @return None; nesoulad zachytí assertions.
# @details Globální funkce modulu. Nejde o fyzický Mission Planner GUI test.
# ----------------------------------------------------------------------------------------------------
def limits(sim):
    for values in ((.10, .08, .03), (.15, .15, .05)):
        for name, value in zip(('UTB_C_RMAX', 'UTB_C_PMAX', 'UTB_C_YMAX'), values):
            sim.param(name, value)
        sim.select(43)
        sim.sticks(1, 1, 1, seconds=1, label='configured_rate_limits')
        sim.sticks(seconds=.5)
        sim.select(0)


# ----------------------------------------------------------------------------------------------------
# @brief Ověřit, že skutečné body targety sledují změněné AP parametry a mají nulové disarmované výstupy.
# @param[in] records Skutečné BIN pakety odděleného konfiguračního scénáře.
# @return Kladná analýza pouze při úplné shodě obou konfigurací.
# @details Globální offline funkce; nepřidává diagnostické firmware zápisy.
# ----------------------------------------------------------------------------------------------------
def analyse_limits(records):
    entries = [r['TimeUS'] for r in records['MODE'] if r['ModeNum'] == 43]
    assert len(entries) == 2
    for i, expected in enumerate(((.10, .08, .03), (.15, .15, .05))):
        end = next(r['TimeUS'] for r in records['MODE'] if r['TimeUS'] > entries[i] and r['ModeNum'] != 43)
        part = [r for r in records['UAAT'] if entries[i] < r['TimeUS'] < end]
        for field, value in zip(('RR', 'PR', 'YR'), expected):
            assert abs(max(r[field] for r in part) - value) < 1e-6
    assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
    return dict(parameter_rate_mapping=True, configurations=2, disarmed=True)


# ----------------------------------------------------------------------------------------------------
# @brief Run one bounded suite; active experiments require the prior actual disarmed evidence.
#
# @param[in] None
# @return Výsledek nebo výjimka při nesouladu důkazů; akce bez návratové hodnoty.
#
# @details Globální funkce modulu. Pouze lokální SITL/offline důkazy.
# ----------------------------------------------------------------------------------------------------
def main():
    """Run one bounded suite; active experiments require the prior actual disarmed evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', choices=('disarmed', 'guards', 'limits', 'roll', 'pitch', 'yaw', 'transitions',
                                            'transitions-pitch', 'transitions-yaw', 'transitions-combined',
                                            'transitions-throttle',
                                            'rc-loss', 'gcs-loss', 'angle-rc-loss', 'angle-gcs-loss',
                                            'ap-roll', 'ap-pitch', 'ap-yaw', 'remap-roll', 'ground', 'manual'), required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    suite = args.suite
    result = None
    if suite == 'manual':
        launch(args.binary.resolve(), args.output, suite, angle.manual)
        return
    if suite == 'disarmed':
        records = launch(args.binary.resolve(), args.output, suite, disarmed)
        result = analyse_disarmed(records)
        result['binary_sha256'] = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    else:
        require_gate(args.binary, args.output)
        if suite == 'limits':
            records = launch(args.binary.resolve(), args.output, suite, limits)
            result = analyse_limits(records)
        elif suite == 'guards':
            records = launch(args.binary.resolve(), args.output, suite, guards)
            assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
            result = dict(guards=True, stop_codes=sorted({r['Stop'] for r in records['UARC']}))
        elif suite == 'ground':
            records = launch(args.binary.resolve(), args.output, suite, ground)
            assert {r['Spl'] for r in records['UARX']} == {0, 1, 2, 3, 4}
            result = dict(standard_rudder_arm=True, all_spool_states=True)
        elif 'loss' in suite:
            source = 'gcs' if 'gcs' in suite else 'rc'
            mode = 42 if suite.startswith('angle') else 43
            records = launch(args.binary.resolve(), args.output, suite, partial(failsafe, source=source, mode=mode))
            assert any(r['ModeNum'] == 6 for r in records['MODE'])
            entries = [r for r in records['MODE'] if r['ModeNum'] == mode]
            assert len(entries) == 2, 'Unexpected autonomous pilot reentry'
            last = entries[-1]['TimeUS']
            first = [r for r in records['UARX'] if r['TimeUS'] > last and r['Seq'] == 1]
            assert len(first) == 3 and all(not r['Arm'] for r in first)
            assert all(r[field] == 0 for r in first for field in ('I', 'D', 'Out'))
            result = dict(native_rtl=True, source=source, mode=mode, explicit_disarmed_reentry_reset=True)
        elif suite.startswith('ap-'):
            # Match nominal rates through native AP ACRO parameters; native shaping remains active.
            records = launch(args.binary.resolve(), args.output, suite,
                             partial(acro_flight, axis=suite[3:], baseline=True), extra=(
                                 'UTB_ENABLE 0\nACRO_RP_RATE 8.594367\nACRO_Y_RATE 2.864789\n'
                                 'ACRO_RP_EXPO 0\nACRO_Y_EXPO 0\nACRO_TRAINER 0\nACRO_BAL_ROLL 0\nACRO_BAL_PITCH 0\n'
                                 'ACRO_OPTIONS 2\nACRO_THR_MID 0\nACRO_RP_RATE_TC 0\nACRO_Y_RATE_TC 0\n'))
            assert not records['UARC']
            result = dict(native_ap=True, axis=suite[3:])
        else:
            action = (partial(transitions, pattern=suite.removeprefix('transitions-') if suite != 'transitions' else 'roll')
                      if suite.startswith('transitions') else partial(acro_flight, axis=suite.removeprefix('remap-')))
            records = launch(args.binary.resolve(), args.output, suite, action,
                             extra='RC6_REVERSED 1\n' if suite == 'remap-roll' else '',
                             rcmap=(6, 7, 8, 3) if suite == 'remap-roll' else (1, 2, 4, 3))
            parts = windows(records)
            (args.output / suite / 'windows.json').write_text(json.dumps(parts) + '\n')
            result = dict(windows=[dict(mode=p['mode'], start=p['start'], **p['analysis']) for p in parts])
            if not suite.startswith('transitions'):
                result['acro'] = validate_acro(parts)
    (args.output / suite / 'analysis.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


# End Public functions
# ==============================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------
# None
# End Private function prototypes
# ==============================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------
# None
# End Private functions
if __name__ == '__main__':
    main()
