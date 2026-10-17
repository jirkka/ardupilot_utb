#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Finite attitude experiments in owned local ArduPilot SITL processes only."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_attitude_sitl.py
# @brief      Konečné attitude flights, gain sweep/repeats/AP porovnání/robust cases a strict angle metrics.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from pymavlink import mavutil
from test_utb_active_sitl import launch, assemble, flight, analyse_flight
from utb_rate_optimization import allocation_analysis

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
INNER = [(0.19, 0.16, 0.0015, 0.19, 15), (0.19, 0.16, 0.001, 0.19, 20), (0.30, 0.60, 0, 0.19, 20)]


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Parameters.
#
# @param[in]  source Source.
# @param[in]  owner Owner.
# @param[in]  profile Explicit finite reference profile identifier.
# @param[in]  angle Angle.
# @param[in]  gain Gain.
# @param[in]  extra Extra.
# @return     Parameter values prepared for the explicit experiment configuration.
#
# @details    Global module function. Calls: zip.
# --------------------------------------------------------------------------------------------------------------------
def parameters(source=1, owner=1, profile=1, angle=3, gain=2, extra=''):
    result = (f'UTB_S_OWNER {owner}\nUTB_S_ATT {source}\nUTB_S_PROFILE {profile}\nUTB_S_ANG {angle}\n'
              f'UTB_S_ARP {gain}\nUTB_S_APP {gain}\nUTB_S_RMAX 0.3\nUTB_S_YRATE 0\nUTB_S_TOFF 0\n'
              'UTB_S_DLY 0\nUTB_S_AW 0\nUTB_S_RUN 0\n')
    for axis, values in zip(['RLL', 'PIT', 'YAW'], INNER):
        for key, value in zip(['P', 'I', 'D', 'IMAX', 'D_HZ'], values):
            result += f'UTB_RAT_{axis}_{key} {value}\n'
    return result + extra


# --------------------------------------------------------------------------------------------------------------------
# @brief      Read parameter.
#
# @param[in]  sim Owned local simulator connection.
# @param[in]  name Name.
# @return     Parameter value read from the explicitly connected peer.
#
# @details    Global module function. Calls: sim.link.mav.param_request_read_send, name.encode, sim.wait.
# --------------------------------------------------------------------------------------------------------------------
def read_parameter(sim, name):
    sim.link.mav.param_request_read_send(1, 1, name.encode(), -1)
    return sim.wait('PARAM_VALUE', lambda m: m.param_id == name).param_value


# --------------------------------------------------------------------------------------------------------------------
# @brief      Disarmed.
#
# @param[in]  sim Owned local simulator connection.
# @return     None.
#
# @details    Global module function. Calls: sim.mode, sim.duration, sim.param, read_parameter.
# --------------------------------------------------------------------------------------------------------------------
def disarmed(sim):
    assert read_parameter(sim, 'UTB_S_ATT') == 0
    sim.mode(41, 41)
    sim.duration(2)
    sim.mode(0, 0)
    for name, wrong, correct in [('UTB_S_ATT', 2, 1), ('UTB_S_ANG', 4, 3), ('UTB_S_ARP', 3, 2),
                                 ('UTB_S_APP', 3, 2), ('UTB_S_RMAX', .2, .3),
                                 ('UTB_S_YRATE', .05, 0), ('UTB_S_TOFF', .0002, 0)]:
        sim.param('UTB_S_ATT', 1)
        sim.mode(41, 41)
        sim.duration(1)
        start = len(sim.events)
        sim.param(name, wrong)
        sim.duration(1)
        assert any(e['text'].startswith('UTB SITL fault=5 ') for e in sim.events[start:]), name
        sim.param(name, correct)
        sim.duration(1)
        assert sim.arm(force=False) != mavutil.mavlink.MAV_RESULT_ACCEPTED
        sim.mode(0, 0)
    sim.param('UTB_S_ANG', 6)
    sim.mode(41, 0)
    sim.param('UTB_S_ANG', 3)
    sim.mode(41, 41)
    sim.duration(2)
    sim.mode(0, 0)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Analyse disarmed.
#
# @param[in]  records Decoded diagnostic records grouped by message type.
# @return     Analysis result; assertions reject invalid or incomplete input evidence.
#
# @details    Global module function. Calls: all, any, dict, max.
# --------------------------------------------------------------------------------------------------------------------
def analyse_disarmed(records):
    assert records['UAAT'] and records['UAST']
    assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
    assert all(r['APN'] == 0 and r['MN'] == r['Seq'] for r in records['UAMT'])
    assert all(max(r[f'C{i}'] for i in range(1, 5)) <= 1000 for r in records['UAMT'])
    assert all(r['AN'] == 0 for r in records['UAST'])
    assert any(r['UN'] > 100 for r in records['UAST'])
    assert records['UAMT'][-1]['Flt'] == 0
    return dict(disarmed=True, default_rate_only=True, frozen_config=True, reentry=True, no_motor_demand=True)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Attitude flight.
#
# @param[in]  sim Owned local simulator connection.
# @param[in]  yaw_degrees Yaw degrees.
# @return     None.
#
# @details    Global module function. Calls: sim.wait, sim.mode, sim.standard_arm, sim.duration.
# --------------------------------------------------------------------------------------------------------------------
def attitude_flight(sim, yaw_degrees=0):
    sim.wait('EKF_STATUS_REPORT', lambda m: m.flags & (16 | 512) and not m.flags & 1024, timeout=45)
    sim.mode(4, 4)
    sim.standard_arm()
    sim.throttle = 1500
    assert sim.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10) == 0
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt >= 9700, timeout=45)
    sim.duration(4)
    if yaw_degrees:
        assert sim.command(mavutil.mavlink.MAV_CMD_CONDITION_YAW, yaw_degrees, 10, 1, 0) == 0
        sim.wait('ATTITUDE', lambda m: abs(math.atan2(math.sin(m.yaw-math.radians(yaw_degrees)),
                                                      math.cos(m.yaw-math.radians(yaw_degrees)))) < .02, timeout=30)
        sim.duration(3)
    sim.wait('ATTITUDE', lambda m: max(abs(m.rollspeed), abs(m.pitchspeed), abs(m.yawspeed)) < .03
             and max(abs(m.roll), abs(m.pitch)) < math.radians(3), timeout=15)
    for axis in ['RLL', 'PIT', 'YAW']:
        for suffix in ['FLTT', 'FLTE', 'FF']:
            sim.param('ATC_RAT_'+axis+'_'+suffix, 0)
    sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 41)
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 41)
    sim.guard = True
    sim.duration(2)
    sim.param('UTB_S_RUN', 1)
    sim.duration(15)
    sim.guard = False
    sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 9)
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt < 200, timeout=40)
    sim.duration(3)
    assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Compute signed angle step metrics and integrated tracking error from complete samples.
#
# @param[in]  rows Exported sample records.
# @param[in]  axis Axis.
# @return     Returned values: result.
#
# @details    Global module function. Calls: dict, enumerate, len, next.
# --------------------------------------------------------------------------------------------------------------------
def angle_metrics(rows, axis):
    errors = [r[f'angle_ref{axis}']-r[f'angle{axis}'] for r in rows]
    result = dict(iae_rad_s=sum(abs(e)*r['dt'] for e, r in zip(errors, rows)),
                  itae_rad_s2=sum(abs(e)*r['t']*r['dt'] for e, r in zip(errors, rows)),
                  rms_rad=math.sqrt(sum(e*e for e in errors)/len(errors)),
                  maximum_abs_angle_rad=max(abs(r[f'angle{axis}']) for r in rows), steps=[])
    changes = [i for i in range(1, len(rows)) if abs(
        rows[i][f'angle_ref{axis}'] - rows[i - 1][f'angle_ref{axis}']) > 1e-5]
    # Constant plateau metrics only; sine remains integral/RMS analysis.
    if len(changes) > 8:
        return result
    for j, start in enumerate(changes):
        stop = changes[j+1] if j+1 < len(changes) else len(rows)
        segment = rows[start:stop]
        target, old = segment[0][f'angle_ref{axis}'], rows[start-1][f'angle_ref{axis}']
        delta = target-old
        values = [(r[f'angle{axis}']-old)/delta for r in segment]
        times = [r['t']-segment[0]['t'] for r in segment]
        t10 = next((t for t, y in zip(times, values) if y >= .1), None)
        t90 = next((t for t, y in zip(times, values) if y >= .9), None)
        outside = [i for i, r in enumerate(segment) if abs(r[f'angle{axis}']-target) > .02*abs(delta)]
        settling_index = outside[-1]+1 if outside else 0
        settling = times[settling_index] if settling_index < len(times) and times[-1]-times[settling_index] >= .25 else None
        tail = [r[f'angle{axis}']-target for r in segment if r['t'] >= segment[-1]['t']-.5]
        result['steps'].append(dict(start_s=segment[0]['t'], target_rad=target,
                                    rise_10_90_s=None if t10 is None or t90 is None else t90-t10,
                                    overshoot_pct=max(0., max(values)-1)*100, settling_2pct_s=settling,
                                    steady_error_rad=sum(tail)/len(tail), max_tail_error_rad=max(map(abs, tail))))
    return result


# --------------------------------------------------------------------------------------------------------------------
# @brief      Analyse.
#
# @param[in]  records Decoded diagnostic records grouped by message type.
# @param[in]  directory Experiment input/output directory.
# @param[in]  source Source.
# @param[in]  owner Owner.
# @return     Analysis result; assertions reject invalid or incomplete input evidence.
#
# @details    Global module function. Calls: assemble, all, allocation_analysis, zip.
# --------------------------------------------------------------------------------------------------------------------
def analyse(records, directory, source, owner):
    sets = assemble(records)
    selected = [(m, a) for m, a in sets if m['Ph'] == 1]
    selected += [(m, a) for m, a in sets if m['Ph'] == 2][:1]
    assert len(selected) == 5600, ('Incomplete finite experiment', len(selected))
    assert all(m['Flt'] == 0 for m, _ in sets), 'Latched stop observed'
    angles = {(r['TimeUS'], r['Seq']): r for r in records['UAAT']}
    status = {(r['TimeUS'], r['Seq']): r for r in records['UAST']}
    params = {r['Name']: r['Value'] for r in records['PARM']}
    rows = []
    first = selected[0][0]['TimeUS']
    parity = 0.
    for m, axes in selected:
        key = (m['TimeUS'], m['Seq'])
        a, s = angles[key], status[key]
        assert a['Src'] == source and a['Flt'] == 0
        assert m['MN'] == m['Seq']
        assert m['APN'] == (m['Seq'] if owner == 2 else 0)
        assert m['UN'] == (m['Seq'] if owner == 1 else 0)
        assert s['AN'] == (m['Seq'] if source == 2 else 0)
        assert s['UN'] == (m['Seq'] if source == 1 else 0)
        row = dict(t=(m['TimeUS']-first)*1e-6, seq=m['Seq'], dt=axes[0]['Dt'],
                   altitude=s['Alt'], vz=s['VZ'], throttle=s['Thr'], yaw=a['Yaw'],
                   limited=a['Lim'], execution_us=m['Ex'])
        for i, label in enumerate(['Roll', 'Pitch']):
            row[f'angle{i}'] = a[label]
            row[f'angle_ref{i}'] = a['RRef' if i == 0 else 'PRef']
        for i in range(3):
            assert axes[i]['Arm'] == 1 and axes[i]['Spl'] == 3 and axes[i]['Own'] == owner
            for field, name in [('Tar', 'target'), ('Gyr', 'gyro'), ('P', 'p'), ('I', 'i'), ('D', 'd'), ('Out', 'out')]:
                row[f'{name}{i}'] = axes[i][field]
        if source == 1:
            # Independently derive R(current)^T R(desired) e_z in a yaw-cancelled frame.
            r, p = a['Roll'], a['Pitch']
            rd, pd = a['RRef'], a['PRef']
            x = math.cos(p)*math.sin(pd)*math.cos(rd)-math.sin(p)*math.cos(pd)*math.cos(rd)
            y = (math.sin(p)*math.sin(r)*math.sin(pd)*math.cos(rd)-math.cos(r)*math.sin(rd)
                 + math.cos(p)*math.sin(r)*math.cos(pd)*math.cos(rd))
            z = (math.sin(p)*math.cos(r)*math.sin(pd)*math.cos(rd)+math.sin(r)*math.sin(rd)
                 + math.cos(p)*math.cos(r)*math.cos(pd)*math.cos(rd))
            k = math.atan2(math.hypot(x, y), z)/math.hypot(x, y) if math.hypot(x, y) > 1e-9 else 1
            parity = max(parity, abs(a['ER']+y*k), abs(a['EP']-x*k))
            rr, pr = -y*k*params['UTB_S_ARP'], x*k*params['UTB_S_APP']
            norm = math.hypot(rr, pr)
            scale = min(1, params['UTB_S_RMAX']/norm) if norm else 1
            parity = max(parity, abs(a['RR']-rr*scale), abs(a['PR']-pr*scale))
        assert all(math.isfinite(v) for v in row.values())
        rows.append(row)
    assert parity < 2e-6, ('Independent rotation/rate mismatch', parity)
    assert all(b['seq'] == a['seq']+1 for a, b in zip(rows, rows[1:]))
    alloc = allocation_analysis(records, rows, directory, INNER, bool(params['UTB_S_AW']), int(params['UTB_S_DLY']))
    for axis, expected in zip(['RLL', 'PIT', 'YAW'], INNER):
        assert all(abs(params[f'UTB_RAT_{axis}_{key}']-value) < 1e-7
                   for key, value in zip(['P', 'I', 'D', 'IMAX', 'D_HZ'], expected))
    with (directory/'attitude-response.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    result = dict(samples=len(rows), observed_hz=(len(rows)-1)/(rows[-1]['t']-rows[0]['t']), source=source, owner=owner,
                  independent_math_max_error=parity, angles=[angle_metrics(rows, i) for i in range(2)],
                  max_body_rate_rads=max(math.sqrt(sum(r[f'gyro{i}']**2 for i in range(3))) for r in rows),
                  max_rate_target_rads=max(math.hypot(r['target0'], r['target1']) for r in rows),
                  rate_rms_rads=[math.sqrt(sum((r[f'target{i}']-r[f'gyro{i}'])**2 for r in rows)/len(rows))
                                 for i in range(3)], outer_clipped_cycles=sum(r['limited'] for r in rows),
                  height_min_m=min(r['altitude'] for r in rows), height_max_m=max(r['altitude'] for r in rows),
                  max_abs_vz_ms=max(abs(r['vz']) for r in rows), yaw_start_rad=rows[0]['yaw'], yaw_end_rad=rows[-1]['yaw'],
                  dt_min_s=min(r['dt'] for r in rows), dt_max_s=max(r['dt'] for r in rows),
                  max_host_rate_slot_us=max(r['execution_us'] for r in rows), allocation=alloc, params=params)
    captures = {r['Seq']: r for r in records['UALT']}
    collective = [sum(captures[r['seq']][f'T{i}']/captures[r['seq']]['G'] for i in range(1, 5))/4 for r in rows]
    periods = [b['t']-a['t'] for a, b in zip(rows, rows[1:])]
    executions = sorted(r['execution_us'] for r in rows)
    result.update(throttle_min=min(r['throttle'] for r in rows), throttle_max=max(r['throttle'] for r in rows),
                  collective_min=min(collective), collective_max=max(collective),
                  period_min_s=min(periods), period_max_s=max(periods),
                  mean_absolute_period_jitter_s=sum(abs(p-.0025) for p in periods)/len(periods),
                  host_execution_mean_us=sum(executions)/len(executions),
                  host_execution_p95_us=executions[int(.95*(len(executions)-1))])
    for axis in result['angles']:
        for step in axis['steps']:
            assert step['max_tail_error_rad'] < math.radians(.25), ('No tracking/return', step)
    (directory/'attitude-metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args,
#             args.output.resolve.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', choices=['disarmed', 'preliminary', 'sweep', 'repeat', 'compare', 'robust', 'rate'],
                        required=True)
    parser.add_argument('--gain', type=float, default=2)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    (out/'run-config.json').write_text(json.dumps(dict(
        binary=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        binary_size=binary.stat().st_size, suite=args.suite, gain=args.gain, speedup=2), indent=2)+'\n')
    results = {}
    if args.suite == 'disarmed':
        records = launch(binary, out, 'disarmed', disarmed,
                         parameters(source=0, profile=1), speedup=2)
        results['disarmed'] = analyse_disarmed(records)
    elif args.suite == 'rate':
        for name, profile in [('roll', 1), ('pitch', 2)]:
            records = launch(binary, out, name, flight, parameters(source=0, profile=profile), speedup=2)
            result, _ = analyse_flight(records, 1, out/name)
            results[name] = result
    else:
        cases = []
        if args.suite == 'preliminary':
            cases = [(name, p, angle, 2, 1, 1, '', 'bfxrev', 0)
                     for name, p, angle in [('zero', 0, 0), ('roll-small', 1, 1), ('pitch-small', 2, 1),
                                            ('roll-three', 1, 3), ('pitch-three', 2, 3)]]
        if args.suite == 'sweep':
            cases = [(f'{axis}-k{gain}', p, 3, gain, 1, 1, '', 'bfxrev', 0)
                     for gain in [1.5, 2, 3] for axis, p in [('roll', 1), ('pitch', 2)]]
        if args.suite == 'repeat':
            cases = [(f'{axis}-{i}', p, 3, args.gain, 1, 1, '', 'bfxrev', 0)
                     for i in range(3) for axis, p in [('roll', 1), ('pitch', 2)]]
        if args.suite == 'compare':
            cases = [(f'{axis}-{name}', p, 3, args.gain, source, owner, '', 'bfxrev', 0)
                     for axis, p in [('roll', 1), ('pitch', 2)]
                     for name, source, owner in [('utb', 1, 1), ('ap-outer-utb-rate', 2, 1), ('ap-chain', 2, 2)]]
        if args.suite == 'robust':
            inertia = 3*.25*(.35*.5)**2
            for name, factor in [('plus', 1.1), ('minus', .9)]:
                data = dict(mass=3, diagonal_size=.35, moment_inertia=[inertia*factor]*2+[2*inertia*factor])
                (out/f'inertia-{name}.json').write_text(json.dumps(data))
                cases.append((f'inertia-{name}', 4, 3, args.gain, 1, 1, '', f'bfxrev:../inertia-{name}.json', 0))
            (out/'motor.json').write_text(json.dumps(dict(slew_max=100)))
            cases += [(name, p, amp, args.gain, 1, 1, extra, model, yaw) for name, p, amp, extra, model, yaw in [
                ('combined', 4, 3, '', 'bfxrev', 0), ('sine', 5, 3, '', 'bfxrev', 0),
                ('sign', 6, 3, '', 'bfxrev', 0), ('heading30', 4, 3, '', 'bfxrev', 30),
                ('yaw-rate', 4, 3, 'UTB_S_YRATE 0.05\n', 'bfxrev', 0),
                ('roll-five', 1, 5, '', 'bfxrev', 0), ('delay', 4, 3, 'UTB_S_DLY 1\n', 'bfxrev', 0),
                ('noise', 4, 3, 'SIM_GYR1_RND 0.5\nSIM_GYR2_RND 0.5\n', 'bfxrev', 0),
                ('throttle', 4, 3, 'UTB_S_TOFF 0.0002\n', 'bfxrev', 0),
                ('motor', 4, 3, '', 'bfxrev:../motor.json', 0),
                ('clamp', 1, 3, 'UTB_S_RMAX 0.05\n', 'bfxrev', 0),
                ('aw', 4, 3, 'UTB_S_AW 1\n', 'bfxrev', 0)]]
        plan = [dict(name=n, profile=p, angle=amp, gain=g, source=src, owner=own, extra=e, model=m, heading=y)
                for n, p, amp, g, src, own, e, m, y in cases]
        (out/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
        for n, p, amp, g, src, own, extra, model, yaw in cases:
            records = launch(binary, out, n, lambda sim: attitude_flight(sim, yaw),
                             parameters(src, own, p, amp, g, extra), model=model, speedup=2)
            results[n] = analyse(records, out/n, src, own)
            (out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
            print(n, 'PASS', results[n]['samples'], flush=True)
    (out/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    print(args.suite, 'PASS', flush=True)


if __name__ == '__main__':
    main()

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Data type, constant, macro definition
# ====================================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private function prototypes
# ====================================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private functions
