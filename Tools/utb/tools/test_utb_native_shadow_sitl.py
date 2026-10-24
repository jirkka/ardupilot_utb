#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_utb_native_shadow_sitl.py
# @brief Owned local SITL only; finite live native AP rate history comparison.
# @author Jiri Kovar
# @version 2026-10-21
# @date 2026-10-21
import argparse
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import socket
import struct
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
from test_utb_active_sitl import ActiveSITL

# Global data: native AP defaults are retained. Only the ordinary AP logging mask
# is raised for OFF/ON evidence; SHADOW itself never logs during capture.
BASE = ('FRAME_CLASS 1\nFRAME_TYPE 18\nSCHED_LOOP_RATE 400\nFSTRATE_ENABLE 0\n'
        'UTB_SHADOW 0\nUNS_RUN 0\nUNS_COUNT 8000\n'
        'LOG_DISARMED 1\nLOG_BITMASK 131071\nFS_THR_ENABLE 1\nFS_GCS_ENABLE 1\n'
        'SIM_GYR1_RND 0\nSIM_GYR2_RND 0\nSIM_ACC1_RND 0\nSIM_ACC2_RND 0\nSIM_MAG_RND 0\n'
        'SIM_WIND_SPD 0\nACRO_TRAINER 0\n')


def json_values(value):
    """Local function: undefined metrics are JSON null, never a fabricated zero or nonstandard NaN token."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_values(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_values(item) for item in value]
    return value


def get_parameter(sim, name):
    """Local function: read one ordinary AP parameter, without a write or timeout bypass."""
    sim.link.mav.param_request_read_send(1, 1, name.encode(), -1)
    return sim.wait('PARAM_VALUE', lambda m: m.param_id == name).param_value


def read_result(sim, destination):
    """Local function: export only after the finite measurement window has ended."""
    assert get_parameter(sim, 'UNS_RUN') == 0
    ftp = MAVFTP(sim.link, target_system=1, target_component=1)
    ftp.cmd_get(['@SYS/utb_rate_shadow.txt', str(destination)])
    response = ftp.process_ftp_reply('get', timeout=15)
    assert response.error_code == 0, str(response)
    lines = destination.read_text().splitlines()
    assert lines[0].startswith('UTBNativeShadowV1,frozen,'), lines[:1]
    result = {'header': lines[0], 'metrics': []}
    metric_rows = False
    for line in lines[1:]:
        fields = line.split(',')
        if line == 'axis,metric,max_abs,rmse':
            metric_rows = True
        elif metric_rows:
            result['metrics'].append(dict(axis=int(fields[0]), metric=int(fields[1]),
                                          max_abs=float(fields[2]), rmse=float(fields[3])))
        elif len(fields) == 2:
            result[fields[0]] = int(Decimal(fields[1]))
        else:
            result[fields[0]] = fields[1:]
    assert result['native_motor_checks'] + result['native_motor_skipped'] == result['seen'], result
    assert result['native_motor_changes'] == 0, result
    return result


def set_mode(sim, mode):
    """Local function: switch native mode without a host-wall observation delay inside the finite window."""
    sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == mode)


def scenario(sim, directory, comparison, case):
    """Local function: use the real AP controller and pilot RC references, never UTB modes."""
    set_mode(sim, 1)
    sim.duration(2)
    if case == 'rate-thread':
        sim.param('FSTRATE_ENABLE', 1)
    if case == 'unsupported':
        # Test explicit rejection of a real AP parameter setting, not a fake supported tune.
        sim.param('ATC_RAT_RLL_SMAX', 50)
    sim.param('UNS_RUN', comparison)
    sim.roll, sim.pitch, sim.yaw = 1580, 1440, 1530
    sim.duration(5)
    sim.roll, sim.pitch, sim.yaw = 1420, 1560, 1470
    set_mode(sim, 0)
    sim.duration(5)
    if case == 'configuration':
        before = get_parameter(sim, 'ATC_RAT_RLL_P')
        sim.param('ATC_RAT_RLL_P', before + .01)
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.duration(14)
    if comparison:
        result = read_result(sim, directory/'rate-shadow.txt')
        assert result['seen'] == 8000, result
        assert result['seen'] == sum(result[k] for k in ('valid', 'invalid', 'desync', 'unsupported'))
        assert result['mismatch'] == 0 and result['invalid'] == 0, result
        if case in ('unsupported', 'rate-thread'):
            assert result['unsupported'] == 8000 and result['valid'] == 0, result
        elif case == 'configuration':
            assert result['config_changes'] == 1 and result['desync'] == 1, result
        else:
            assert result['valid'] == 8000 and result['desync'] == result['unsupported'] == 0, result
            assert all(m['max_abs'] == m['rmse'] == 0 for m in result['metrics']), result
            if comparison == 1:
                assert result['synchronizations'] == 8000
            else:
                assert result['synchronizations'] == 1, result
        (directory/'result.json').write_text(json.dumps(json_values(result), indent=2, allow_nan=False))
    else:
        (directory/'result.json').write_text(json.dumps(dict(comparison_off=True)))
    sim.observe(0, False)


def flight_scenario(sim, directory, comparison, case):
    """Local function: ordinary AP arming and landing, software-only; no force-arm or UTB mode."""
    set_mode(sim, 0)
    if case == 'flight-ff':
        # A separate validation profile excites FF/D_FF; this is not PID tuning or a default change.
        for axis in ('RLL', 'PIT', 'YAW'):
            sim.param(f'ATC_RAT_{axis}_FF', .05)
            sim.param(f'ATC_RAT_{axis}_D_FF', .002)
    sim.standard_arm()
    sim.throttle = 1700
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt > 2500, timeout=30)
    sim.throttle = 1500
    sim.duration(3)
    sim.param('UNS_COUNT', 16000)
    sim.param('UNS_RUN', comparison)
    if case in ('flight', 'flight-ff', 'flight-input'):
        sim.roll, sim.pitch, sim.yaw = ((1560, 1440, 1550) if case == 'flight-input' else (1520, 1480, 1510))
        sim.duration(4)
        set_mode(sim, 1)
        sim.roll, sim.pitch, sim.yaw = ((1550, 1450, 1550) if case == 'flight-input' else (1510, 1490, 1505))
        sim.duration(2)
        sim.roll, sim.pitch, sim.yaw = ((1450, 1550, 1450) if case == 'flight-input' else (1490, 1510, 1495))
        sim.duration(2)
        set_mode(sim, 0)
        sim.roll = sim.pitch = sim.yaw = 1500
        sim.duration(34)
    else:
        sim.duration(3)
        start = len(sim.events)
        if case == 'rc-failsafe':
            sim.param('SIM_RC_FAIL', 1)
            original_rc = sim.rc
            sim.rc = lambda: None
        else:
            # RC override also renews AP's GCS last-seen timer (GCS_Common.cpp:4224).
            # Keep an independent simulated receiver healthy, stop both GCS heartbeat and overrides.
            original_rc = sim.rc
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.sendto(struct.pack('<8H', 1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000),
                            ('127.0.0.1', sim.owned_rc_port))
            sim.rc = lambda: None
            sim.heartbeat_wall = float('inf')
        if case == 'rc-failsafe':
            heartbeat = sim.wait('HEARTBEAT', lambda m: m.custom_mode in (6, 9), timeout=30)
        else:
            # Default AP FS_OPTIONS=16 intentionally keeps pilot control on GCS loss.
            assert int(get_parameter(sim, 'FS_OPTIONS')) == 16
            sim.wait('STATUSTEXT', lambda m: 'GCS Failsafe - Continuing Pilot Control' in m.text, timeout=30)
            heartbeat = sim.wait('HEARTBEAT')
            assert heartbeat.custom_mode == 0
        sim.duration(1)  # drain STATUSTEXT following the mode heartbeat
        events = sim.events[start:]
        (directory/'failsafe.json').write_text(json.dumps(dict(mode=heartbeat.custom_mode, events=events), indent=2))
        if case == 'rc-failsafe':
            sim.param('SIM_RC_FAIL', 0)
            sim.rc = original_rc
        else:
            sim.rc = original_rc
            receiver.close()
            sim.heartbeat_wall = -1
        sim.duration(3)
        set_mode(sim, 9)
        sim.duration(40)
    set_mode(sim, 9)
    sim.throttle = 1000
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt < 150, timeout=90)
    sim.duration(3)
    heartbeat = sim.wait('HEARTBEAT')
    if heartbeat.base_mode & 128:
        assert sim.arm(disarm=True) == mavutil.mavlink.MAV_RESULT_ACCEPTED
        sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
    if comparison:
        result = read_result(sim, directory/'rate-shadow.txt')
        assert result['seen'] == result['valid'] == 16000, result
        assert result['invalid'] == result['unsupported'] == result['desync'] == result['mismatch'] == 0, result
        assert result['synchronizations'] == (1 if comparison == 2 else 16000), result
        assert all(m['max_abs'] == m['rmse'] == 0 for m in result['metrics']), result
        (directory/'result.json').write_text(json.dumps(result, indent=2))
    else:
        (directory/'result.json').write_text(json.dumps(dict(comparison_off=True)))
    sim.observe(9, False)


def run(binary, output, comparison, case):
    """Public function: create and dispose one owned software process with a loopback-only endpoint."""
    root = Path(__file__).resolve().parents[3]
    assert binary.is_file() and binary.name in ('arducopter', 'arducopter.elf'), binary
    output.mkdir(parents=True, exist_ok=False)
    sim = None
    result = {'comparison': comparison, 'case': case, 'hardware': False,
              'simulated_arming': case in ('flight', 'flight-ff', 'flight-input', 'rc-failsafe', 'gcs-failsafe')}
    result['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    started = time.monotonic()
    with (output/'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                owned_rc_port = allocation.getsockname()[1]
            sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', output,
                             console, True, 0, extra_params=BASE, model='bfxrev', speedup=5, rcin_port=owned_rc_port)
            sim.owned_rc_port = owned_rc_port
            sim.link.logfile = (output/'receive.tlog').open('xb')
            sim.ready()
            if case in ('flight', 'flight-ff', 'flight-input', 'rc-failsafe', 'gcs-failsafe'):
                flight_scenario(sim, output, comparison, case)
            else:
                scenario(sim, output, comparison, case)
            sim.duration(3)
            result['success'] = True
        except Exception as error:
            result['success'] = False
            result['error'] = repr(error)
        finally:
            if sim:
                (output/'events.json').write_text(json.dumps(sim.events, indent=2))
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
    if result.get('success') and case in ('rc-failsafe', 'gcs-failsafe'):
        modes, errors = [], []
        for logfile in (output/'logs').glob('*.BIN'):
            reader = mavutil.mavlink_connection(str(logfile))
            while (message := reader.recv_match(type=['MODE', 'ERR'])) is not None:
                (modes if message.get_type() == 'MODE' else errors).append(message.to_dict())
            reader.close()
        if case == 'rc-failsafe':
            result['native_failsafe_mode_verified'] = any(m['Rsn'] == 3 and m['ModeNum'] in (6, 9) for m in modes)
        else:
            result['native_failsafe_mode_verified'] = (
                any(e['Subsys'] == 8 and e['ECode'] == 1 for e in errors) and
                any('GCS Failsafe - Continuing Pilot Control' in e['text'] for e in sim.events))
        (output/'native-errors.json').write_text(json.dumps(errors, indent=2))
        (output/'native-modes.json').write_text(json.dumps(modes, indent=2))
        if not result['native_failsafe_mode_verified']:
            result['success'] = False
            result['error'] = 'Native failsafe MODE reason missing'
    result['host_wall_s'] = time.monotonic() - started
    (output/'session.json').write_text(json.dumps(result, indent=2))
    assert result['success'], result
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--comparison', type=int, choices=(0, 1, 2), default=2)
    parser.add_argument('--case',
                        choices=('normal', 'configuration', 'unsupported', 'rate-thread',
                                 'flight', 'flight-ff', 'flight-input', 'rc-failsafe', 'gcs-failsafe'),
                        default='normal')
    args = parser.parse_args()
    print(json.dumps(run(args.binary.resolve(), args.output.resolve(), args.comparison, args.case), indent=2))
