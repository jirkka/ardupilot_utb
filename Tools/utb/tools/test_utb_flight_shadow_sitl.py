#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_utb_flight_shadow_sitl.py
# @brief Actual native flights with the existing passive chain and bounded frozen RAM pages.
# @author Jiri Kovar
# @version 2026-10-25
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
from test_utb_active_sitl import ActiveSITL
from test_utb_active_chain_sitl import hold, parameters, sticks
from test_utb_attitude_shadow_sitl import BASE, get_parameter, set_mode
from test_utb_chain_shadow_sitl import read_result as read_chain, check as check_chain
from test_utb_timing_sitl import export as export_timing
from utb_flight_shadow_analysis import parse, check_supported, write_results


def fetch(sim, name, destination, exists=True):
    """Only frozen/disarmed exports are decoded; a pending read never exposes live data."""
    ftp = MAVFTP(sim.link, target_system=1, target_component=1)
    ftp.cmd_get(['@SYS/' + name, str(destination)])
    reply = ftp.process_ftp_reply('get', timeout=30)
    if not exists:
        assert reply.error_code != 0, 'Compile-OFF unexpectedly exposes Flight files'
        return ''
    assert reply.error_code == 0, str(reply)
    return destination.read_text()


def wait_until(sim, boot_ms):
    while sim.wait('ATTITUDE').time_boot_ms < boot_ms:
        pass


def process_sample(sim):
    """Actual owned Linux process CPU/RSS, measured by the client outside AP's control path."""
    values = Path(f'/proc/{sim.process.pid}/stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=sim.process.pid, start_ticks=int(values[19]), cpu_ticks=int(values[11]) + int(values[12]),
                ticks_per_second=os.sysconf('SC_CLK_TCK'), rss_bytes=int(values[21]) * os.sysconf('SC_PAGE_SIZE'),
                host_us=time.monotonic_ns() // 1000)


def maneuvers(sim, marks, short=False):
    """Actual AP RC path, both signs, combined inputs, yaw and neutral."""
    if short:
        for _ in range(6):
            sticks(sim, marks, 'combined_positive', .6, roll=1600, pitch=1400, yaw=1560)
            sticks(sim, marks, 'combined_negative', .6, roll=1400, pitch=1600, yaw=1440)
            sticks(sim, marks, 'neutral', .6)
    else:
        for axis in ('roll', 'pitch'):
            for value in (1600, 1400, 1750, 1250):
                sticks(sim, marks, f'{axis}_{value}', 2, **{axis: value})
                sticks(sim, marks, 'neutral', 2)
        sticks(sim, marks, 'combined_positive', 2, roll=1600, pitch=1400, yaw=1560)
        sticks(sim, marks, 'combined_negative', 2, roll=1400, pitch=1600, yaw=1440)
        sticks(sim, marks, 'yaw_positive', 1, yaw=1560)
        sticks(sim, marks, 'yaw_negative', 1, yaw=1440)
    sticks(sim, marks, 'neutral', 2)


def failsafe(sim, case, rc_port):
    """Original AP radio/GCS failsafe; the receiver remains distinct from the GCS."""
    original = sim.rc
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if case == 'rc':
        sim.param('SIM_RC_FAIL', 1)
        sim.rc = lambda: None
    else:
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
    assert sim.wait('HEARTBEAT').custom_mode in (6, 9)
    set_mode(sim, 0)  # Explicit pilot return; the frozen diagnostic must not restart.
    hold(sim, 2)


def export_native(output):
    names = ['MODE', 'ARM', 'ERR', 'DSF', 'PM', 'ANG', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'RCIN', 'RCOU', 'UGC1', 'UARX']
    data = {name: [] for name in names}
    for logfile in sorted((output / 'logs').glob('*.BIN')):
        reader = mavutil.mavlink_connection(str(logfile))
        while (message := reader.recv_match(type=names)) is not None:
            data[message.get_type()].append(message.to_dict())
        reader.close()
    for name, rows in data.items():
        if rows:
            with (output / (name + '.csv')).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    assert not any(r['ModeNum'] in (41, 42, 43) for r in data['MODE']), 'Active UTB mode used in passive experiment'
    assert not data['UGC1'] and not data['UARX'], 'Active UTB controller evidence in a passive experiment'
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=('native', 'chain', 'flight', 'absent'), default='flight')
    parser.add_argument('--timing', action='store_true',
                        help='Also exercise the existing finite main/rate/attitude clock probe')
    cases = ('normal', 'ff', 'mode', 'rc', 'gcs', 'early', 'overflow', 'no-logger', 'pressure',
             'disarmed', 'unsupported', 'conflict', 'interrupted')
    parser.add_argument('--case', choices=cases, default='normal')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    binary, output = args.binary.resolve(), args.output.resolve()
    assert binary.is_file() and binary.name == 'arducopter'
    output.mkdir(parents=True, exist_ok=False)
    count = {'ff': 12000, 'early': 2000, 'overflow': 8000, 'pressure': 12000,
             'disarmed': 4000, 'unsupported': 4000, 'conflict': 4000, 'interrupted': 4000}.get(args.case, 24000)
    stride = 1 if args.case == 'overflow' else 40 if args.case == 'disarmed' else 100 if args.case == 'ff' else 200
    params = BASE + ('UTB_A_ENABLE 0\nUTB_C_ENABLE 0\nUCS_RUN 0\nUAS_RUN 0\nUNS_RUN 0\n'
                     'FS_OPTIONS 0\nARMING_RUDDER 0\nLOG_FILE_BUFSIZE 512\n')
    if args.case == 'ff':
        params += 'ATC_RAT_RLL_FF 0.02\nATC_RAT_PIT_FF 0.02\nATC_RAT_YAW_FF 0.01\nATC_RAT_RLL_D_FF 0.001\n'
    if args.case == 'no-logger':
        params += 'LOG_BACKEND_TYPE 0\n'
    if args.case == 'pressure':
        params += 'LOG_FILE_BUFSIZE 1\n'
    if args.case == 'unsupported':
        params += 'FSTRATE_ENABLE 1\n'
    status = dict(success=False, hardware=False, profile=args.profile, case=args.case, count=count, stride=stride,
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    sim, result, marks = None, None, []
    with (output / 'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                rc_port = allocation.getsockname()[1]
            sim = ActiveSITL(binary, root / 'Tools/autotest/default_params/copter.parm', output, console, True, 1,
                             extra_params=params, model='bfxrev', speedup=4 if args.case == 'pressure' else 2,
                             rcin_port=rc_port, isolated_aux_ports=True)
            sim.link.logfile = (output / 'receive.tlog').open('xb')
            sim.ready()
            sim.wait('EKF_STATUS_REPORT', lambda m: bool(m.flags & (16 | 512)) and not m.flags & 1024, timeout=45)
            before = parameters(sim)
            (output / 'parameters-before.json').write_text(json.dumps(before, indent=2))
            assert before['UCS_RUN'] == before['UAS_RUN'] == before['UNS_RUN'] == 0
            set_mode(sim, 0)
            start_ms = sim.wait('ATTITUDE').time_boot_ms
            status['process_before_window'] = process_sample(sim)
            if args.timing:
                assert args.case == 'disarmed' and before['UTT_RUN'] == 0
                sim.param('UTT_COUNT', count)
            if args.profile == 'flight':
                assert before['UCS_F_RUN'] == 0
                sim.param('UCS_F_COUNT', count)
                sim.param('UCS_F_DECIM', stride)
                sim.param('UCS_F_RUN', 1)
            elif args.profile == 'chain':
                sim.param('UCS_COUNT', count)
                sim.param('UCS_RUN', 1)
            elif args.profile == 'absent':
                assert 'UCS_F_RUN' not in before
            if args.timing:
                sim.param('UTT_RUN', 1)
            hold(sim, 1)
            if args.case == 'interrupted':
                text = fetch(sim, 'utb_flight.txt', output / 'utb_flight.txt')
                assert text.startswith('UTBFlightV1,pending')
                try:
                    parse(text, [])
                except AssertionError:
                    status['incomplete_rejected'] = True
                else:
                    raise AssertionError('Incomplete session was accepted')
                assert not sim.wait('HEARTBEAT').base_mode & 128
            elif args.case in ('disarmed', 'unsupported', 'conflict') or args.profile == 'absent':
                if args.case == 'conflict':
                    sim.param('UCS_RUN', 1)
                wait_until(sim, start_ms + count * 2.5 + 1500)
                assert not sim.wait('HEARTBEAT').base_mode & 128
            else:
                sim.standard_arm()
                sim.throttle = 1700
                sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt > 12000, timeout=35)
                sim.throttle = 1500
                hold(sim, 2)
                if args.case in ('rc', 'gcs'):
                    failsafe(sim, args.case, rc_port)
                elif args.case == 'mode':
                    sticks(sim, marks, 'roll_positive', 2, roll=1600)
                    sticks(sim, marks, 'neutral', 2)
                    set_mode(sim, 1)
                    hold(sim, 2)
                    set_mode(sim, 0)
                    hold(sim, 2)
                    assert get_parameter(sim, 'UCS_F_RUN') == 0
                else:
                    maneuvers(sim, marks, args.case in ('ff', 'overflow', 'pressure'))
                    wait_until(sim, start_ms + count * 2.5 + 1500)
                if args.profile == 'flight':
                    assert get_parameter(sim, 'UCS_F_RUN') == 0
                elif args.profile == 'chain':
                    assert get_parameter(sim, 'UCS_RUN') == 0
                status['process_after_window'] = process_sample(sim)
                status['window_sim_seconds'] = (sim.wait('ATTITUDE').time_boot_ms - start_ms) / 1000
                sticks(sim, marks, 'neutral_before_land', 3)
                attitude = sim.wait('ATTITUDE')
                assert max(abs(attitude.roll), abs(attitude.pitch)) < .052359878, 'Existing 3A self-level criterion'
                set_mode(sim, 9)
                sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128, timeout=65)
                hold(sim, 2)
            if args.profile == 'flight' and args.case != 'interrupted':
                assert get_parameter(sim, 'UCS_F_RUN') == 0
                boot_count = get_parameter(sim, 'STAT_BOOTCNT')
                metadata = fetch(sim, 'utb_flight.txt', output / 'utb_flight.txt')
                if args.case in ('unsupported', 'conflict'):
                    reason = 6 if args.case == 'unsupported' else 4
                    assert metadata.startswith(f'UTBFlightV1,frozen,reason={reason},')
                    status['expected_rejection'] = reason
                else:
                    pages = [fetch(sim, f'utb_flight{k}.csv', output / f'utb_flight{k}.csv') for k in range(8)]
                    assert get_parameter(sim, 'STAT_BOOTCNT') == boot_count, 'Restart during immutable page export'
                    status['export_boot_count'] = boot_count
                    result = parse(metadata, pages)
                    write_results(result, output / 'analysis')  # Preserve a decoded mismatch before rejecting it.
                    check_supported(result, 7 if args.case in ('mode', 'rc', 'gcs') else 1)
                    if args.case == 'overflow':
                        assert result['records']['overwritten'] > 0 and len(result['rows']) == 128
                    else:
                        assert result['records']['overwritten'] == 0
                    if args.case not in ('early', 'disarmed'):
                        assert result['scalars']['independent_i'] > 100
                    if args.case == 'ff':
                        armed = [row for row in result['rows'] if int(row['Flags']) & 5 == 5]
                        assert result['scalars']['att_shaping'] > 0
                        assert any(abs(row['UTB0FF']) > 1e-6 for row in armed), 'No recorded own FF excitation'
                        assert any(abs(row['UTB0DFF']) > 1e-6 for row in armed), 'No recorded own D_FF excitation'
            elif args.profile == 'chain':
                result = read_chain(sim, output / 'chain.txt')
                check_chain(result, count)
                (output / 'chain-result.json').write_text(json.dumps(result, indent=2))
            elif args.profile == 'absent':
                fetch(sim, 'utb_flight.txt', output / 'missing.txt', exists=False)
            if args.timing:
                timing = export_timing(sim, output / 'utb_timing.txt')
                assert timing['header'] == 'UTBTimingV1,frozen,reason=1' and timing['cycles'] == count
                for key in ('missing_rate', 'missing_att', 'duplicate_rate', 'duplicate_att', 'missing_finish',
                            'invalid_time', 'cycle_gaps', 'unsupported'):
                    assert timing[key] == 0, (key, timing[key])
                (output / 'timing.json').write_text(json.dumps(timing, indent=2))
            assert not any('InternalError' in e['text'] or 'Internal error' in e['text'] for e in sim.events)
            status['events'] = sim.events
            status['experiment_completed'] = True
        except Exception as error:
            status['error'] = repr(error)
            raise
        finally:
            if sim:
                sim.link.logfile.close()
                sim.close()
            (output / 'session.json').write_text(json.dumps(status, indent=2))
            (output / 'marks.json').write_text(json.dumps(marks, indent=2))
    native = export_native(output)
    if args.case not in ('no-logger', 'pressure', 'interrupted'):
        assert native['DSF'] and max(r['Dp'] for r in native['DSF']) == 0
    status['native_logger_drops'] = max((r['Dp'] for r in native['DSF']), default=None)
    if args.case == 'pressure':
        status['native_logging_pressure_observed'] = bool(status['native_logger_drops'])
    if args.case in ('rc', 'gcs'):
        reason = 3 if args.case == 'rc' else 5
        assert any(r['Rsn'] == reason and r['ModeNum'] in (6, 9) for r in native['MODE'])
    if args.case == 'normal':
        assert native['ANG']
        for key in ('Roll', 'Pitch'):
            values = [r[key] for r in native['ANG']]
            assert min(values) < -1 and max(values) > 1, 'No signed physical motion'
    status['success'] = True
    (output / 'session.json').write_text(json.dumps(status, indent=2))
    print(json.dumps({k: v for k, v in status.items() if k != 'events'}))


if __name__ == '__main__':
    main()
