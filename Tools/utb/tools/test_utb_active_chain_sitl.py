#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_utb_active_chain_sitl.py
# @brief Actual owned SITL flights, independent motor authority and closed-loop A/B evidence.
# @author Jiri Kovar
# @version 2026-10-24
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import socket
import struct

from pymavlink import mavutil
from test_utb_active_sitl import ActiveSITL, BASE
from test_utb_attitude_shadow_sitl import get_parameter, set_mode


def parameters(sim):
    """Record the actual full configuration; do not assume two parameter banks match."""
    result = {}
    sim.link.mav.param_request_list_send(1, 1)
    while True:
        row = sim.wait('PARAM_VALUE')
        result[row.param_id] = row.param_value
        if len(result) == row.param_count:
            return result


def sync_bank(sim, directory):
    """Apply the reference AP configuration only to the independent UT3 bank of this fresh simulator."""
    before = parameters(sim)
    matched = {}
    for name, value in before.items():
        own = 'UT3_' + name[4:]
        if name.startswith('ATC_') and own in before:
            if before[own] != value:
                sim.param(own, value)
            matched[name] = dict(value=value, own=own)
    assert 'ATC_RAT_RLL_P' in matched and 'ATC_RAT_YAW_I' in matched
    (directory/'parameter-alignment.json').write_text(json.dumps(matched, indent=2))
    (directory/'parameters-before.json').write_text(json.dumps(before, indent=2))


def hold(sim, seconds):
    """Use simulated timestamps for scenario duration, never host-wall flight timing."""
    start = sim.wait('ATTITUDE').time_boot_ms
    while ((sim.wait('ATTITUDE').time_boot_ms - start) & 0xffffffff) < seconds * 1000:
        pass


def sticks(sim, marks, label, seconds, roll=1500, pitch=1500, yaw=1500, throttle=None):
    """Finite reference segment; the native RC input path remains active."""
    sim.roll, sim.pitch, sim.yaw = roll, pitch, yaw
    if throttle is not None:
        sim.throttle = throttle
    sim.rc()
    marks.append(dict(label=label, approximate_boot_ms=sim.last_boot_ms, roll=roll, pitch=pitch,
                      yaw=yaw, throttle=sim.throttle, duration_s=seconds))
    hold(sim, seconds)


def flight(sim, mode, case, marks, rc_port):
    """Ordinary native takeoff, actual active-owner maneuvers, ordinary native landing."""
    set_mode(sim, 0)
    sim.standard_arm()
    sim.throttle = 1700
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt > 12000, timeout=35)
    sim.throttle = 1500
    hold(sim, 3)
    set_mode(sim, mode)
    hold(sim, 2)
    if case == 'gain':
        original_gain = get_parameter(sim, 'UT3_RAT_RLL_P')
        sticks(sim, marks, 'gain_reference', 3, roll=1600)
        sticks(sim, marks, 'neutral', 3)
        sim.param('UT3_RAT_RLL_P', original_gain * .5)
        sticks(sim, marks, 'own_gain_halved', 3, roll=1600)
        sticks(sim, marks, 'neutral', 3)
        sim.param('UT3_RAT_RLL_P', original_gain)
    elif case in ('rc', 'gcs'):
        original = sim.rc
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if case == 'rc':
            sim.param('SIM_RC_FAIL', 1)
            sim.rc = lambda: None
        else:
            # Actual simulator receiver continues while real GCS heartbeats stop.
            sim.rc = lambda: receiver.sendto(struct.pack('<8H', 1500, 1500, 1500, 1500,
                                                         1000, 1000, 1000, 1000), ('127.0.0.1', rc_port))
            sim.heartbeat_wall = float('inf')
        sim.wait('HEARTBEAT', lambda m: m.custom_mode in (6, 9), timeout=30)
        hold(sim, 2)
        sim.rc = original
        sim.heartbeat_wall = -1
        if case == 'rc':
            sim.param('SIM_RC_FAIL', 0)
        receiver.close()
        hold(sim, 2)
        assert sim.wait('HEARTBEAT').custom_mode in (6, 9), 'Unexpected automatic UTB return'
        set_mode(sim, mode)
        hold(sim, 2)
    elif case == 'saturation':
        sticks(sim, marks, 'combined_full', .35, roll=2000, pitch=2000, yaw=2000, throttle=1900)
        sticks(sim, marks, 'release', .5, throttle=1500)
    elif case == 'transitions':
        for destination in (43, 42, 0, 42, 1, 43, 42):
            sticks(sim, marks, f'before_{destination}', .3, roll=1560, pitch=1440, yaw=1550)
            set_mode(sim, destination)
            hold(sim, .5)
    elif mode in (0, 42):
        for axis in ('roll', 'pitch'):
            for value in (1600, 1400, 1800, 1200):
                sticks(sim, marks, f'{axis}_{value}', 3, **{axis: value})
                sticks(sim, marks, 'neutral', 3)
        sticks(sim, marks, 'combined_positive', 1.5, roll=1600, pitch=1400, yaw=1560)
        sticks(sim, marks, 'combined_negative', 1.5, roll=1400, pitch=1600, yaw=1440)
    else:
        for axis in ('roll', 'pitch', 'yaw'):
            for value in (1560, 1440):
                sticks(sim, marks, f'{axis}_{value}', .4, **{axis: value})
                sticks(sim, marks, 'neutral', .6)
        sticks(sim, marks, 'combined_positive', .3, roll=1560, pitch=1440, yaw=1560)
        sticks(sim, marks, 'combined_negative', .3, roll=1440, pitch=1560, yaw=1440)
    sticks(sim, marks, 'neutral_before_level', .5)
    set_mode(sim, 42 if mode >= 40 else 0)
    hold(sim, 5)
    attitude = sim.wait('ATTITUDE')
    assert max(abs(attitude.roll), abs(attitude.pitch)) < math.radians(3), 'Self-level failed (existing 3A criterion)'
    set_mode(sim, 9)
    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128, timeout=65)
    hold(sim, 2)


def ground(sim, case):
    """Ordinary arming and rejected opt-ins/context; never send force-arm."""
    set_mode(sim, 0)
    if case == 'gate':
        for mode in (42, 43):
            set_mode(sim, mode)
            sim.standard_arm()
            hold(sim, 2)
            assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
            sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
            hold(sim, 1)
        set_mode(sim, 0)
    if case == 'unsupported':
        sim.param('FSTRATE_ENABLE', 1)
        hold(sim, 2)
    for mode, parameter in ((42, 'UTB_A_ENABLE'), (43, 'UTB_C_ENABLE')):
        if case == 'gate':
            sim.param(parameter, 0)
        sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
        hold(sim, 1)
        assert sim.wait('HEARTBEAT').custom_mode == 0, 'Unsupported/disabled active mode was accepted'
        if case == 'gate':
            sim.param(parameter, 1)
    if case == 'unsupported':
        sim.param('FSTRATE_ENABLE', 0)


def check_sequence(rows):
    """Reject a missing/duplicated rate step; mode entry starts an explicit new epoch."""
    previous = None
    epochs = 0
    for row in rows:
        assert previous is None or row['TimeUS'] > previous['TimeUS'], 'Nonmonotonic rate capture'
        assert row['Src'] + 1 == row['Seq'], 'Extra target delay or repeated Attitude input'
        if row['Seq'] == 1:
            epochs += 1
        else:
            assert previous is not None and row['Seq'] == previous['Seq'] + 1, 'Missing/duplicate rate step'
            assert row['Md'] == previous['Md'], 'Mode changed without epoch initialization'
        previous = row
    assert epochs > 0, 'Missing initialized first rate step'
    return epochs


def check_released(rows, start, end=float('inf')):
    """MODE follows the old motor slot in the same loop-start timestamp domain."""
    assert sum(r['TimeUS'] == start for r in rows) <= 1
    assert not any(start < r['TimeUS'] < end for r in rows), 'UTB kept authority after failsafe'


def analyse(directory, active, mode, case):
    """Check actual capture-to-actuator dataflow, not a label or successful process exit."""
    names = ['UGC1', 'UGA1', 'UARX', 'UAMT', 'UAAT', 'ATT', 'ANG', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'RCOU',
             'RCIN', 'MODE', 'DSF', 'PARM', 'ERR', 'ARM', 'EV']
    records = {name: [] for name in names}
    for logfile in sorted((directory/'logs').glob('*.BIN')):
        reader = mavutil.mavlink_connection(str(logfile))
        while (message := reader.recv_match(type=names)) is not None:
            records[message.get_type()].append(message.to_dict())
        reader.close()
    for name, rows in records.items():
        if rows:
            with (directory/f'{name}.csv').open('w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
    assert records['RATE'] and records['ATT'] and records['RCOU'], 'Missing native flight evidence'
    result = dict(active=active, paired_steps=0, rate_sets=0,
                  actual_limit_samples=sum(bool(r['Lim']) for r in records['UGC1']),
                  logger_drops=max((r['Dp'] for r in records['DSF']), default=None),
                  modes=records['MODE'])
    assert result['logger_drops'] == 0, 'Incomplete diagnostic export; no pairs may be silently skipped'
    if active:
        assert records['UGC1'] and records['UARX'], 'No active owner diagnostics'
        result['initialized_epochs'] = check_sequence(records['UGC1'])
        axes = {}
        for row in records['UARX']:
            assert row['Own'] == 3 and row['Flt'] == 0
            axes.setdefault((row['TimeUS'], row['Seq']), {})[row['Ax']] = row
        assert len(records['UARX']) == 3 * len(axes) == 3 * len(records['UGC1']), 'Duplicate/unpaired PID axis export'
        assert len(records['UAMT']) == len(records['UGC1']), 'Missing motor-output slot audit'
        for row in records['UAMT']:
            assert row['APN'] == row['Flt'] == 0 and row['UN'] == row['MN'] == row['Seq'], row
        assert records['UGA1'], 'No own Attitude producer trace'
        producers = records['UGA1']
        producer_times = [r['TimeUS'] for r in producers]
        for row in records['UGC1']:
            assert row['Md'] in (42, 43), 'Invalid claimed active ownership'
            assert row['APN'] == row['Bad'] == 0 and row['Wr'] == 6, row
            assert row['Src'] + 1 == row['Seq'], row
            key = (row['TimeUS'], row['Seq'])
            assert set(axes[key]) == {0, 1, 2}, key
            if row['Seq'] > 1:
                source_index = bisect.bisect_left(producer_times, row['TimeUS']) - 1
                assert source_index >= 0
                source = producers[source_index]
                assert source['Seq'] == row['Src'] and source['Md'] == row['Md'], (source, row)
                for axis, label in enumerate(('R', 'P', 'Y')):
                    assert axes[key][axis]['Tar'] == source[label], 'Own Attitude target transport broken'
                result['paired_steps'] += 1
        result['rate_sets'] = len(records['UGC1'])
        assert result['paired_steps'] > 100
    else:
        assert not records['UGC1'] and not records['UARX'], 'Native baseline unexpectedly used UTB'
    result['peak_i'] = {name: max(abs(r['I']) for r in records[name]) for name in ('PIDR', 'PIDP', 'PIDY')}
    mode_times = [r['TimeUS'] for r in records['MODE']]

    def in_mode(row):
        index = bisect.bisect_right(mode_times, row['TimeUS']) - 1
        return index >= 0 and records['MODE'][index]['ModeNum'] == mode

    if case in ('flight', 'ff'):
        response = {}
        for desired, actual in (('DesRoll', 'Roll'), ('DesPitch', 'Pitch')) if mode in (0, 42) else (
                ('RDes', 'R'), ('PDes', 'P'), ('YDes', 'Y')):
            rows = [r for r in records['ATT' if mode in (0, 42) else 'RATE'] if in_mode(r)]
            demand = [r[desired] for r in rows]
            measured = [r[actual] for r in rows]
            assert min(demand) < 0 < max(demand), 'Missing signed reference excitation'
            # Excitation detection only, not a new closed-loop quality/safety threshold.
            assert max(measured) > .1 * max(demand) and min(measured) < .1 * min(demand), 'No signed physical response'
            response[actual] = dict(desired_min=min(demand), desired_max=max(demand),
                                    actual_min=min(measured), actual_max=max(measured))
        result['response_excitation'] = response
        if active:
            assert 'Acc' in records['UGA1'][0]
            result['shaping_acceleration_peak_radss'] = max(r['Acc'] for r in records['UGA1'])
            assert result['shaping_acceleration_peak_radss'] > 1e-6, 'No own input-shaper excitation'
            independent = [r for r in records['PIDR'] if in_mode(r) and not r['Flags'] & 8 and abs(r['I']) > 1e-6]
            assert len(independent) > 100, 'No nonzero I history without repeated reset'
            result['nonzero_i_without_reset_samples'] = len(independent)
    if case == 'ff':
        result['feedforward_peak'] = {}
        for name in ('PIDR', 'PIDP', 'PIDY'):
            rows = [r for r in records[name] if in_mode(r)]
            result['feedforward_peak'][name] = {
                'ff': max(abs(r['FF']) for r in rows),
                'dff': max(abs(r['DFF']) for r in rows),
            }
        assert result['feedforward_peak']['PIDR']['ff'] > 1e-6
        assert result['feedforward_peak']['PIDR']['dff'] > 1e-6
    if case in ('rc', 'gcs'):
        reason = 3 if case == 'rc' else 5  # AP_Vehicle/ModeReason.h
        takeovers = [r for r in records['MODE'] if r['Rsn'] == reason and r['ModeNum'] in (6, 9)]
        assert takeovers, 'No original AP failsafe mode transition'
        for transition in takeovers:
            start = transition['TimeUS']
            end = next((r['TimeUS'] for r in records['MODE']
                        if r['TimeUS'] > start and r['ModeNum'] in (42, 43)), float('inf'))
            check_released(records['UGC1'], start, end)
        result['native_failsafe_takeovers'] = takeovers
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--owner', choices=('native', 'utb'), default='utb')
    parser.add_argument('--speedup', type=float, default=2)
    parser.add_argument('--mode', choices=('angle', 'acro'), default='angle')
    cases = ('flight', 'ff', 'transitions', 'saturation', 'rc', 'gcs', 'gate', 'absence', 'unsupported', 'gain')
    parser.add_argument('--case', choices=cases, default='flight')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    binary = args.binary.resolve()
    assert binary.is_file() and binary.name == 'arducopter'
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    mode = (42 if args.mode == 'angle' else 43) if args.owner == 'utb' else (0 if args.mode == 'angle' else 1)
    params = BASE + ('UTB_A_ENABLE 1\nUTB_C_ENABLE 1\nUNS_RUN 0\nUAS_RUN 0\nUCS_RUN 0\n'
                     'ACRO_OPTIONS 2\nFS_OPTIONS 0\nARMING_RUDDER 0\nLOG_FILE_BUFSIZE 512\n')
    if args.case == 'ff':
        params += 'ATC_RAT_RLL_FF 0.02\nATC_RAT_PIT_FF 0.02\nATC_RAT_YAW_FF 0.01\nATC_RAT_RLL_D_FF 0.001\n'
    sim = None
    marks = []
    status = dict(success=False, hardware=False, mode=mode, case=args.case, owner=args.owner,
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    with (output/'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                rc_port = allocation.getsockname()[1]
            sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', output,
                             console, True, 1, extra_params=params, model='bfxrev', speedup=args.speedup, rcin_port=rc_port)
            sim.link.logfile = (output/'receive.tlog').open('xb')
            sim.ready()
            if args.case != 'absence':
                sync_bank(sim, output)
            sim.wait('EKF_STATUS_REPORT', lambda m: bool(m.flags & (16 | 512)) and not m.flags & 1024, timeout=45)
            if args.case in ('gate', 'absence', 'unsupported'):
                ground(sim, args.case)
            else:
                flight(sim, mode, args.case, marks, rc_port)
            assert not any('InternalError' in event['text'] or 'Internal error' in event['text'] for event in sim.events)
            status['events'] = sim.events
            status['flight_completed'] = True
        except Exception as error:
            status['error'] = repr(error)
            raise
        finally:
            if sim:
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
            (output/'session.json').write_text(json.dumps(status, indent=2))
            (output/'marks.json').write_text(json.dumps(marks, indent=2))
    try:
        result = analyse(output, args.owner == 'utb' and args.case not in ('absence', 'unsupported'), mode, args.case)
    except Exception as error:
        status['analysis_error'] = repr(error)
        (output/'session.json').write_text(json.dumps(status, indent=2))
        raise
    status['success'] = True
    (output/'session.json').write_text(json.dumps(status, indent=2))
    (output/'analysis.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
