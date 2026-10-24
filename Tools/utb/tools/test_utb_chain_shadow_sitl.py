#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_utb_chain_shadow_sitl.py
# @brief Owned software-only native flight and integrated passive chain validation.
# @author Jiri Kovar
# @version 2026-10-24
import argparse
import json
from pathlib import Path
import socket
import struct
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
from test_utb_active_sitl import ActiveSITL
from test_utb_attitude_shadow_sitl import BASE, get_parameter, set_mode
from test_utb_attitude_shadow_sitl import read_result as read_attitude_result, check_result as check_attitude_result
from test_utb_native_shadow_sitl import read_result as read_rate_result


def read_result(sim, destination):
    """Read immutable diagnostic data only after the window and simulated disarming."""
    assert get_parameter(sim, 'UCS_RUN') == 0
    ftp = MAVFTP(sim.link, target_system=1, target_component=1)
    ftp.cmd_get(['@SYS/utb_chain_shadow.txt', str(destination)])
    reply = ftp.process_ftp_reply('get', timeout=20)
    assert reply.error_code == 0, str(reply)
    lines = destination.read_text().splitlines()
    assert lines[0].startswith('UTBChainV1,frozen,reason='), lines[:1]
    result = dict(header=lines[0], metrics=[], raw=[])
    for line in lines[1:]:
        fields = line.split(',')
        if fields[0] == 'section':
            continue
        if fields[0] in ('att', 'target', 'pid'):
            result['metrics'].append(dict(section=fields[0], axis=int(fields[1]), metric=int(fields[2]),
                                          max_abs=float(fields[3]), rmse=float(fields[4])))
        elif len(fields) == 2:
            result[fields[0]] = int(fields[1])
        elif fields[0] == 'peaks':
            result['peaks'] = list(map(float, fields[1:]))
        else:
            result['raw'].append(fields)
    return result


def check(result, count, unsupported=False):
    """Account for warmup separately and require actual dynamic evidence where applicable."""
    assert result['attempts'] == count if not unsupported else result['attempts'] < count
    assert result['header'].endswith('reason=6' if unsupported else 'reason=1'), result
    assert result['warmup'] == 1 and result['paired'] == result['pid_valid'] == result['pid_seen'], result
    assert result['paired'] + result['warmup'] + result['unsupported'] == result['attempts'], result
    assert result['pid_synchronizations'] == 1, result
    assert result['att_seen'] == result['att_valid'], result
    for key in ('missing', 'duplicate', 'invalid', 'upstream_mismatch', 'downstream_mismatch', 'motor_changes',
                'att_mismatch', 'att_desync', 'att_invalid', 'pid_mismatch', 'pid_desync', 'pid_invalid', 'pid_unsupported'):
        assert result[key] == 0, (key, result)
    assert len(result['metrics']) == 84
    assert all(m['max_abs'] == m['rmse'] == 0 for m in result['metrics']), result
    assert (result['rate_loop'] - result['source_loop']) & 0xffffffff == 1


def near_limits(directory):
    """Inspect native log evidence after shutdown; 90% is an excitation marker, not a flight safety limit."""
    params, high, outputs = {}, [], []
    for logfile in (directory/'logs').glob('*.BIN'):
        reader = mavutil.mavlink_connection(str(logfile))
        while (message := reader.recv_match(type=['PARM', 'RCIN', 'RCOU'])) is not None:
            row = message.to_dict()
            if message.get_type() == 'PARM':
                params[row['Name']] = row['Value']
            elif message.get_type() == 'RCIN' and row['C3'] >= 1940:
                high.append(row['TimeUS'])
            elif message.get_type() == 'RCOU':
                outputs.append(row)
        reader.close()
    assert high and params['MOT_PWM_MAX'] > params['MOT_PWM_MIN'] > 0
    selected = [row for row in outputs if min(high) <= row['TimeUS'] <= max(high)]
    maxima = {key: max(row[key] for row in selected) for key in ('C1', 'C2', 'C3', 'C4')}
    fraction = (max(maxima.values()) - params['MOT_PWM_MIN']) / (params['MOT_PWM_MAX'] - params['MOT_PWM_MIN'])
    result = dict(high_collective_window_us=[min(high), max(high)], samples=len(selected), max_pwm=maxima,
                  pwm_min=params['MOT_PWM_MIN'], pwm_max=params['MOT_PWM_MAX'], fraction=fraction)
    (directory/'near-limits.json').write_text(json.dumps(result, indent=2))
    assert fraction >= .9, result
    return result


def run(binary, output, case, enabled=True):
    """Own a loopback SITL process; no serial paths or hardware endpoints are accepted."""
    root = Path(__file__).resolve().parents[3]
    count = 6000 if case == 'disarmed' else 16000
    output.mkdir(parents=True, exist_ok=False)
    sim = None
    started = time.monotonic()
    status = dict(hardware=False, case=case, enabled=enabled, success=False)
    params = BASE + f'UCS_RUN 0\nUCS_COUNT {count}\nUTT_RUN 0\n'
    # Deliberate FF/DFF test fixture, not a new flight tune or a change to AP algorithms.
    if case == 'feedforward':
        params += 'ATC_RAT_RLL_FF 0.02\nATC_RAT_PIT_FF 0.02\nATC_RAT_YAW_FF 0.01\nATC_RAT_RLL_D_FF 0.001\n'
    with (output/'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                rc_port = allocation.getsockname()[1]
            sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', output,
                             console, True, 0, extra_params=params, model='bfxrev', speedup=5, rcin_port=rc_port)
            sim.link.logfile = (output/'receive.tlog').open('xb')
            sim.ready()
            set_mode(sim, 0)
            if case != 'disarmed':
                if case == 'rate-only':
                    # ModeAcro::AcroOptions::RATE_LOOP_ONLY is bit 1; bit 0 is AIR_MODE.
                    sim.param('ACRO_OPTIONS', 2)
                sim.standard_arm()
                sim.throttle = 1700
                sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt > 2500, timeout=30)
                sim.throttle = 1500
                sim.duration(3)
            if case == 'parallel':
                sim.param('UNS_COUNT', count)
                sim.param('UAS_COUNT', count)
                sim.param('UNS_RUN', 2)
                sim.param('UAS_RUN', 2)
            if enabled:
                sim.param('UCS_RUN', 1)
            if case == 'limits':
                sim.throttle, sim.roll, sim.pitch, sim.yaw = 1950, 1800, 1800, 1800
                sim.duration(1.5)
                sim.throttle, sim.roll, sim.pitch, sim.yaw = 1500, 1500, 1500, 1500
            if case in ('rc-failsafe', 'gcs-failsafe'):
                sim.duration(3)
                original_rc = sim.rc
                receiver = None
                if case == 'rc-failsafe':
                    sim.param('SIM_RC_FAIL', 1)
                    sim.rc = lambda: None
                    sim.wait('HEARTBEAT', lambda m: m.custom_mode in (6, 9), timeout=30)
                    sim.duration(3)
                    sim.param('SIM_RC_FAIL', 0)
                else:
                    assert int(get_parameter(sim, 'FS_OPTIONS')) == 16
                    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    receiver.sendto(struct.pack('<8H', 1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000),
                                    ('127.0.0.1', rc_port))
                    sim.rc = lambda: None
                    sim.heartbeat_wall = float('inf')
                    sim.wait('STATUSTEXT', lambda m: 'GCS Failsafe - Continuing Pilot Control' in m.text, timeout=30)
                    sim.duration(3)
                    sim.heartbeat_wall = -1
                sim.rc = original_rc
                if receiver:
                    receiver.close()
                status['native_failsafe_observed'] = True
            else:
                sim.roll, sim.pitch, sim.yaw = 1560, 1440, 1550
                sim.duration(4)
                sim.roll, sim.pitch, sim.yaw = 1440, 1560, 1450
                sim.duration(4)
                set_mode(sim, 1)
                sim.roll, sim.pitch, sim.yaw = 1550, 1450, 1550
                sim.duration(2)
                sim.roll, sim.pitch, sim.yaw = 1450, 1550, 1450
                sim.duration(2)
                set_mode(sim, 0)
                sim.roll = sim.pitch = sim.yaw = 1500
                sim.duration(5 if case == 'disarmed' else 31)
            if case != 'disarmed':
                set_mode(sim, 9)
                sim.throttle = 1000
                sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt < 150, timeout=90)
                sim.duration(3)
                hb = sim.wait('HEARTBEAT')
                if hb.base_mode & 128:
                    assert sim.arm(disarm=True) == mavutil.mavlink.MAV_RESULT_ACCEPTED
                    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
            if enabled:
                result = read_result(sim, output/'chain-shadow.txt')
                check(result, count, case in ('rc-failsafe', 'gcs-failsafe'))
                if case not in ('disarmed', 'rc-failsafe', 'gcs-failsafe'):
                    assert result['att_shaping'] > 0 and result['peaks'][0] > .1 and result['peaks'][1] > .01
                    assert result['independent_i'] > 0 and result['peaks'][2] > 1e-5
                if case == 'feedforward':
                    assert result['peaks'][3] > 0 and result['peaks'][4] > 0
                if case == 'rate-only':
                    assert result['att_command_2_valid'] > 0 and result['att_command_1_valid'] == 0
                if case == 'limits':
                    status['armed_saturation_observed'] = result['limited'] > 0
                (output/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False))
                if case == 'parallel':
                    attitude = read_attitude_result(sim, output/'old-attitude.txt')
                    check_attitude_result(attitude, count)
                    rates = read_rate_result(sim, output/'old-rate.txt')
                    assert rates['valid'] == rates['seen'] == count
                    assert rates['invalid'] == rates['desync'] == rates['mismatch'] == rates['unsupported'] == 0
                    assert all(v['max_abs'] == v['rmse'] == 0 for v in rates['metrics'])
                    (output/'old-modules.json').write_text(json.dumps(dict(attitude=attitude, rate=rates), indent=2))
            status['success'] = True
        except Exception as error:
            status['error'] = repr(error)
        finally:
            if sim:
                (output/'events.json').write_text(json.dumps(sim.events, indent=2))
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
    if status['success'] and case == 'limits':
        status['near_limits'] = near_limits(output)
    status['host_wall_s'] = time.monotonic() - started
    (output/'session.json').write_text(json.dumps(status, indent=2))
    assert status['success'], status
    return status


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--case', choices=('disarmed', 'flight', 'rate-only', 'parallel', 'feedforward', 'limits',
                           'rc-failsafe', 'gcs-failsafe'), default='disarmed')
    parser.add_argument('--off', action='store_true')
    args = parser.parse_args()
    assert args.binary.is_file() and args.binary.name in ('arducopter', 'arducopter.elf')
    print(json.dumps(run(args.binary.resolve(), args.output.resolve(), args.case, not args.off), indent=2))
