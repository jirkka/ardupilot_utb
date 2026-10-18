#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''Controlled actual SITL pilot tuning; never accepts a hardware endpoint.'''
# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses remain unchanged.
# @file utb_pilot_tuning.py
# @brief Stejné skutečné RC reference pro systematické porovnání pilotních gains.
# @author Jiri Kovar
# @version 2026-10-18
# @date 2026-10-18
# --------------------------------------------------------------------------------------------------------------------
# Include Files
import argparse
import hashlib
import json
import math
from functools import partial
from pathlib import Path

from pymavlink import mavutil

import utb_angle_sitl as angle
import utb_pilot_sitl as pilot
from utb_active_sitl_analysis import step_metrics
# End Include Files
# Static data declarations
BASELINE = {'RLL': [.19, .16, .0015, .19, 15], 'PIT': [.19, .16, .001, .19, 20],
            'YAW': [.30, .60, 0, .19, 20], 'angle': [3, 3]}
# End Static data declarations
# Data type, constant, macro definition
# None
# End Data type, constant, macro definition
# Public functions


# ----------------------------------------------------------------------------------------------------
# @brief Převést explicitní kandidátní konfiguraci na skutečné AP parametry.
# @param[in] tune R/P/Y P,I,D,IMAX,D_HZ a dvojice attitude P.
# @return Parametry vlastního SITL, nikoli přímá injekce do PID.
# @details Globální funkce; původní RAT_* a AP ATC_* se nemění.
# ----------------------------------------------------------------------------------------------------
def parameter_text(tune):
    lines = []
    for axis in ('RLL', 'PIT', 'YAW'):
        for suffix, value in zip(('P', 'I', 'D', 'IMAX', 'D_HZ'), tune[axis]):
            lines.append(f'UTB_P_{axis}_{suffix} {value}\n')
    for axis, value in zip(('R', 'P'), tune['angle']):
        lines.append(f'UTB_P_ANG_{axis} {value}\n')
    return ''.join(lines)


# ----------------------------------------------------------------------------------------------------
# @brief Oddělit měření původním AP GUIDED návratem na stejnou přípravnou polohu.
# @param[in] sim Vlastněný skutečný SITL.
# @return None; další UTB vstup až po naměřeném ustálení.
# @details Globální funkce; MODE4 je viditelný v logu, žádný AP controller neběží uvnitř UTB kroku.
# ----------------------------------------------------------------------------------------------------
def recover(sim):
    sim.expected_mode = None
    sim.bench_altitude = None
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.throttle = 1500
    sim.select(4, armed=True, settle=False)
    mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    sim.link.mav.set_position_target_local_ned_send(sim.last_boot_ms, 1, 1,
                                                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                                                    0, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0)
    sim.wait('GLOBAL_POSITION_INT', lambda m: 9900 < m.relative_alt < 10100 and abs(m.vz) < 8,
             timeout=45)
    sim.wait('ATTITUDE', lambda m: max(abs(m.roll), abs(m.pitch)) < math.radians(1)
             and max(abs(m.rollspeed), abs(m.pitchspeed), abs(m.yawspeed)) < .03, timeout=30)
    sim.duration(1)
    sim.throttle = 1472
    sim.bench_altitude = sim.wait('GLOBAL_POSITION_INT').relative_alt * .001
    sim.marks.append(dict(boot_ms=sim.last_boot_ms, label='native_guided_recovery_complete'))


# ----------------------------------------------------------------------------------------------------
# @brief Provést oba směry skutečných lineárních RC kroků se stejnou referencí.
# @param[in] sim Vlastněný simulátor.
# @param[in] stage Rate, angle nebo úplná finální validace.
# @return None; nekonečné scénáře ani přímý PID setter zde nejsou.
# @details Globální funkce; standardní GUIDED příprava a AP LAND jsou oddělené od pilotního řízení.
# ----------------------------------------------------------------------------------------------------
def experiment(sim, stage):
    angle.prepare_flight(sim)
    if stage in ('rate', 'validation'):
        for axis in ('roll', 'pitch'):
            for sign in (1, -1):
                recover(sim)
                sim.select(43, armed=True, settle=False)
                sim.sticks(seconds=.6, label='rate_center')
                sim.sticks(**{axis: sign}, seconds=1.2, label=f'rate_{axis}_{sign}')
                sim.sticks(seconds=1, label='rate_zero')
                sim.select(42, armed=True, settle=False)
                sim.sticks(seconds=2, label='explicit_level')
    if stage in ('angle', 'validation'):
        for axis in ('roll', 'pitch'):
            for sign in (1, -1):
                recover(sim)
                sim.select(42, armed=True, settle=False)
                sim.sticks(seconds=.6, label='angle_center')
                sim.sticks(**{axis: sign}, seconds=2, label=f'angle_{axis}_{sign}')
                sim.sticks(seconds=2, label=f'level_{axis}_{sign}')
                # Distinct mode entries make reset/priming provenance unambiguous.
                sim.select(43, armed=True, settle=False)
                sim.sticks(seconds=.4)
    if stage == 'validation':
        for sign in (1, -1):
            recover(sim)
            sim.select(43, armed=True, settle=False)
            sim.sticks(yaw=sign, seconds=2, label=f'yaw_{sign}')
            sim.sticks(seconds=1)
        for values in ((.3, -.3, .5), (-.3, .3, -.5)):
            recover(sim)
            sim.select(43, armed=True, settle=False)
            sim.sticks(*values, seconds=.7, label='acro_combined')
            pilot.switch(sim, 42, rc=True)
            sim.sticks(*values, seconds=.6, label='angle_combined')
            pilot.switch(sim, 43, rc=True)
            sim.sticks(*values, seconds=.6, label='acro_switch')
            pilot.switch(sim, 42)
            sim.sticks(seconds=2, label='level_after_switch')
        recover(sim)
        sim.select(42, armed=True, settle=False)
        sim.sticks(.6, -.6, .5, seconds=2, label='angle_diagonal')
        sim.sticks(-.6, .6, -.5, seconds=2, label='angle_diagonal_reverse')
        sim.sticks(seconds=2)
    angle.finish_flight(sim)


# ----------------------------------------------------------------------------------------------------
# @brief Vyžádat skutečný AP parametr přes lokální MAVLink spojení.
# @param[in] sim Vlastněný simulátor.
# @param[in] name Existující název AP parametru.
# @return Potvrzená hodnota z PARAM_VALUE.
# @details Globální funkce; žádný implicitní zápis nebo hardware endpoint.
# ----------------------------------------------------------------------------------------------------
def read_parameter(sim, name):
    sim.link.mav.param_request_read_send(1, 1, name.encode(), -1)
    return sim.wait('PARAM_VALUE', lambda m: m.param_id == name).param_value


# ----------------------------------------------------------------------------------------------------
# @brief Ověřit meze a neměnnost nových pilotních parametrů na DISARMED SITL.
# @param[in] sim Vlastněný simulátor.
# @return None; entry rejection, latched CONFIG a explicitní návrat se kontrolují jednotlivě.
# @details Globální funkce; žádný negativní arming pokus v původním AP režimu.
# ----------------------------------------------------------------------------------------------------
def gain_guards(sim):
    for mode in (42, 43):
        for name, invalid in (('UTB_P_RLL_P', .31), ('UTB_P_PIT_D', .005), ('UTB_P_RLL_D_HZ', 0)):
            correct = read_parameter(sim, name)
            sim.param(name, invalid)
            sim.mode(mode, 0)
            sim.param(name, correct)
        for name in ('UTB_P_RLL_P', 'UTB_P_PIT_D_HZ'):
            correct = read_parameter(sim, name)
            sim.select(mode)
            sim.duration(.5)
            sim.expected_mode = None
            first = len(sim.events)
            sim.param(name, correct * .95)
            sim.duration(1)
            assert any(' stop=1' in e['text'] for e in sim.events[first:]), name
            sim.param(name, correct)
            sim.duration(.5)
            assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
            sim.select(0)
            sim.select(mode)
            sim.duration(.5)
            sim.select(0)
    correct = read_parameter(sim, 'UTB_P_ANG_R')
    sim.param('UTB_P_ANG_R', 7)
    sim.mode(42, 0)
    sim.select(43)
    sim.duration(1)
    sim.select(0)
    sim.param('UTB_P_ANG_R', correct)
    sim.select(42)
    sim.duration(.5)
    sim.expected_mode = None
    first = len(sim.events)
    sim.param('UTB_P_ANG_R', correct * .95)
    sim.duration(1)
    assert any(' stop=1' in e['text'] for e in sim.events[first:])
    sim.param('UTB_P_ANG_R', correct)
    assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.select(0)


# ----------------------------------------------------------------------------------------------------
# @brief Vyhodnotit jednotné časové okno bez podmínění výsledku požadovaným tune.
# @param[in] rows Skutečné úplné logové vzorky.
# @param[in] axis Tělová osa.
# @param[in] kind Rate nebo angle.
# @return Rise/overshoot/settling/IAE a složky PID s jednoznačnými jednotkami.
# @details Globální funkce; nedosažené ustálení zůstává None, není nahrazeno PASS.
# ----------------------------------------------------------------------------------------------------
def plateau(rows, axis, kind):
    refkey = f'Tar{axis}' if kind == 'rate' else f'angle_ref{axis}'
    actkey = f'Gyr{axis}' if kind == 'rate' else f'angle{axis}'
    active = [r for r in rows if abs(r[refkey]) > (.01 if kind == 'rate' else .02)]
    if not active:
        return None
    amplitude = active[0][refkey]
    group = []
    start = active[0]['t']
    duration = 1.0 if kind == 'rate' and axis < 2 else 2.0
    for r in active:
        if abs(r[refkey] - amplitude) > 1e-6 or r['t'] - start > duration:
            break
        group.append(r)
    if len(group) < (300 if kind == 'rate' else 600):
        return None
    converted = [dict(t=r['t'] - start, ref0=amplitude, target0=r[refkey], gyro0=r[actkey]) for r in group]
    metric = step_metrics(converted, 0)
    errors = [abs(r[refkey] - r[actkey]) for r in group]
    metric.update(axis=axis, kind=kind, samples=len(group),
                  iae=sum(e * r['dt'] for e, r in zip(errors, group)),
                  itae=sum(e * r['dt'] * (r['t'] - start) for e, r in zip(errors, group)),
                  max_p=max(abs(r[f'P{axis}']) for r in group),
                  max_i=max(abs(r[f'I{axis}']) for r in group),
                  max_d=max(abs(r[f'D{axis}']) for r in group),
                  max_output=max(abs(r[f'Out{axis}']) for r in group))
    if kind == 'angle':
        metric['rate_cap_samples'] = sum(math.hypot(r['Tar0'], r['Tar1']) >= .3 - 1e-6 for r in group)
        metric['rate_cap_fraction'] = metric['rate_cap_samples'] / len(group)
        centered = [r for r in rows if r['t'] > group[-1]['t'] and r[refkey] == 0]
        if len(centered) > 400:
            first = centered[0]['t']
            recovery = [dict(t=r['t'] - first, ref0=abs(amplitude), target0=abs(amplitude),
                             gyro0=abs(amplitude) - math.copysign(1, amplitude) * r[actkey])
                        for r in centered if r['t'] - first <= 2.0]
            metric['self_level'] = step_metrics(recovery, 0)
    return metric


# ----------------------------------------------------------------------------------------------------
# @brief Ověřit vlastnictví, reference a uzavřít naměřené metriky všech os.
# @param[in] records Skutečné BIN záznamy vlastního procesu.
# @return Přehled a úplné spárované vzorky pro reprodukovatelné grafy.
# @details Globální funkce; normální RP experimenty se neoznačí za saturační validaci.
# ----------------------------------------------------------------------------------------------------
def analyse(records):
    declared = {line.split()[0] for line in parameter_text(BASELINE).splitlines()}
    assert declared <= {r['Name'] for r in records['PARM']}, 'Missing actual pilot parameter readback'
    parts = pilot.windows(records)
    results = []
    for part in parts:
        kind = 'rate' if part['mode'] == 43 else 'angle'
        for axis in range(3 if kind == 'rate' else 2):
            metric = plateau(part['rows'], axis, kind)
            if metric is not None:
                metric['start'] = part['start']
                results.append(metric)
        for row in part['rows']:
            if part['mode'] == 43:
                for a, name, limit in zip(range(3), ('roll', 'pitch', 'yaw'), (.15, .15, .05)):
                    assert abs(row[f'Tar{a}'] - row[f'normalized_{name}'] * limit) < 1e-6
    allrows = [row for part in parts for row in part['rows']]
    return dict(steps=results, samples=len(allrows),
                maximum_tilt_deg=max(math.degrees(max(abs(x['angle0']), abs(x['angle1']))) for x in allrows),
                maximum_vertical_speed=max(abs(x['vz']) for x in allrows),
                allocation_limit_masks=sorted({x['allocation_limits'] for x in allrows}),
                integration_blocked_masks=sorted({x['integration_blocked'] for x in allrows}),
                logger_drops_max=max((r['Dp'] for r in records['DSF']), default=None)), parts


# ----------------------------------------------------------------------------------------------------
# @brief Potvrdit všech sedmnáct skutečně použitých gains z časově platného BIN readbacku.
# @param[in] records Skutečný BIN včetně startup PARM a pilotních záznamů.
# @param[in] tune Požadovaný kandidát.
# @return None; chybějící, rozdílná nebo za běhu změněná hodnota odmítne měření.
# @details Globální funkce; malé relativní tolerance pokrývají pouze float32 serializaci.
# ----------------------------------------------------------------------------------------------------
def verify_logged_tune(records, tune):
    first = min(r['TimeUS'] for r in records['UARC'])
    last = max(r['TimeUS'] for r in records['UARC'])
    for line in parameter_text(tune).splitlines():
        name, value = line.split()
        prior = [r for r in records['PARM'] if r['Name'] == name and r['TimeUS'] <= first]
        assert prior, ('Missing tune readback', name)
        expected = float(value)
        assert math.isclose(prior[-1]['Value'], expected, rel_tol=2e-7, abs_tol=1e-10), name
        assert all(math.isclose(r['Value'], expected, rel_tol=2e-7, abs_tol=1e-10)
                   for r in records['PARM'] if r['Name'] == name and first < r['TimeUS'] <= last), name


# ----------------------------------------------------------------------------------------------------
# @brief Spustit pouze schválený lokální SITL s explicitním tune a identitou binary.
# @param[in] None
# @return None; chyby zachovávají raw evidence a vrací nenulový exit.
# @details Globální funkce; aktivní scénáře vyžadují SHA-vázaný DISARMED gate.
# ----------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--stage', choices=('gate', 'rate', 'angle', 'validation', 'gain-guards'), required=True)
    parser.add_argument('--tune', type=Path)
    parser.add_argument('--speedup', type=float, default=2)
    parser.add_argument('--model', default='bfxrev',
                        help='Native SITL model; JSON path is relative to each isolated run directory')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    tune = BASELINE if args.tune is None else json.loads(args.tune.read_text())
    if args.stage == 'gate':
        records = pilot.launch(args.binary, args.output, 'disarmed', pilot.disarmed,
                               extra=parameter_text(tune), speedup=args.speedup, model=args.model)
        result = pilot.analyse_disarmed(records)
        result['binary_sha256'] = hashlib.sha256(args.binary.read_bytes()).hexdigest()
        (args.output / 'disarmed/analysis.json').write_text(json.dumps(result, indent=2))
        return
    if args.stage == 'gain-guards':
        records = pilot.launch(args.binary, args.output, args.name, gain_guards, extra=parameter_text(tune),
                               speedup=args.speedup, model=args.model)
        assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
        assert any(r['Stop'] == 1 for r in records['UARC'])
        result = dict(disarmed=True, gain_guards=True, stop_codes=sorted({r['Stop'] for r in records['UARC']}))
        (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
        return
    pilot.require_gate(args.binary, args.output)
    action = partial(experiment, stage=args.stage)
    records = pilot.launch(args.binary, args.output, args.name, action, extra=parameter_text(tune),
                           speedup=args.speedup, model=args.model)
    verify_logged_tune(records, tune)
    result, parts = analyse(records)
    for kind in (('rate', 'angle') if args.stage == 'validation' else (args.stage,)):
        amplitude = .15 if kind == 'rate' else math.radians(5)
        main_steps = [x for x in result['steps'] if x['kind'] == kind
                      and abs(x['amplitude_rads'] - amplitude) < 1e-6]
        assert len(main_steps) == 4 and {(x['axis'], x['sign']) for x in main_steps} == {
            (0, 1), (0, -1), (1, 1), (1, -1)}, 'Missing signed main scenario'
    if args.stage == 'validation':
        yaw_steps = [x for x in result['steps'] if x['kind'] == 'rate' and x['axis'] == 2
                     and abs(x['amplitude_rads'] - .05) < 1e-6]
        assert len(yaw_steps) == 2 and {x['sign'] for x in yaw_steps} == {1, -1}, 'Missing signed yaw steps'
    result.update(tune=tune, model=args.model, speedup=args.speedup,
                  binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest())
    (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
    (args.output / args.name / 'windows.json').write_text(json.dumps(parts))
    print(json.dumps(result), flush=True)


# End Public functions
# Private function prototypes
# None
# End Private function prototypes
# Private functions
# None
# End Private functions
if __name__ == '__main__':
    main()
