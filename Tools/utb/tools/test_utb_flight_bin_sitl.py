#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_utb_flight_bin_sitl.py
# @brief Owned local SITL; runtime OFF, native flight, mode/failsafe and one-session lifecycle.
# @author Jiri Kovar
# @version 2026-10-26
import argparse
import hashlib
import json
from pathlib import Path
import socket

from pymavlink import mavutil
from test_utb_active_sitl import ActiveSITL
from test_utb_active_chain_sitl import hold, sticks
from test_utb_attitude_shadow_sitl import BASE, get_parameter, set_mode
from test_utb_flight_shadow_sitl import fetch, failsafe, process_sample
from utb_flight_bin_analysis import read_bin, write_results


def run(args):
    root = Path(__file__).resolve().parents[3]
    output, binary = args.output.resolve(), args.binary.resolve()
    output.mkdir(parents=True, exist_ok=False)
    enabled = args.case not in ('off', 'absent')
    params = BASE + ('UCS_RUN 0\nUCS_F_RUN 0\nUTT_RUN 0\nARMING_RUDDER 0\nFS_OPTIONS 0\n'
                     'LOG_FILE_BUFSIZE 512\nATC_RAT_RLL_FF 0.02\nATC_RAT_PIT_FF 0.02\n'
                     'ATC_RAT_YAW_FF 0.01\nATC_RAT_RLL_D_FF 0.001\n')
    if args.case != 'absent':
        params += f'UCS_B_ENABLE {int(enabled)}\nUCS_B_RATE {args.rate}\nUCS_B_SECS 35\n'
    if args.case == 'pressure':
        params += 'LOG_FILE_BUFSIZE 1\n'
    if args.case == 'no-backend':
        params += 'LOG_BACKEND_TYPE 0\n'
    status = dict(success=False, hardware=False, case=args.case, rate=args.rate,
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    sim, marks, chain = None, [], ''
    with (output/'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                rc_port = allocation.getsockname()[1]
            sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', output, console,
                             True, 0, extra_params=params, model='bfxrev', speedup=8 if args.case == 'pressure' else 2,
                             rcin_port=rc_port, isolated_aux_ports=True)
            sim.ready()
            set_mode(sim, 0)
            if args.case == 'absent':
                from test_utb_active_chain_sitl import parameters
                assert 'UCS_B_ENABLE' not in parameters(sim)
            else:
                assert get_parameter(sim, 'UCS_B_ENABLE') == int(enabled)
            status['process_before'] = process_sample(sim)
            status['start_ms'] = sim.wait('ATTITUDE').time_boot_ms
            sim.standard_arm()
            status['armed_ms'] = sim.wait('ATTITUDE').time_boot_ms
            if args.case == 'rearm':
                hold(sim, 2)
                assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
                sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
                hold(sim, 2)
                sim.standard_arm()
                hold(sim, 2)
                assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
                sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
            else:
                sim.throttle = 1700
                sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt > 12000, timeout=40)
                sim.throttle = 1500
                for _ in range(3):
                    sticks(sim, marks, 'positive', 1, roll=1580, pitch=1420, yaw=1540)
                    sticks(sim, marks, 'negative', 1, roll=1420, pitch=1580, yaw=1460)
                    sticks(sim, marks, 'neutral', 1)
                if args.case == 'rc':
                    failsafe(sim, 'rc', rc_port)
                elif args.case != 'mode':
                    hold(sim, 30)
                set_mode(sim, 9)
                sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128, timeout=80)
            hold(sim, 3)
            status['end_ms'] = sim.wait('ATTITUDE').time_boot_ms
            status['process_after'] = process_sample(sim)
            if enabled:
                chain = fetch(sim, 'utb_chain_shadow.txt', output/'chain.txt')
                assert chain.startswith('UTBChainV1,frozen'), chain[:200]
                assert 'motor_changes,0' in chain
                binline = next((line for line in chain.splitlines() if line.startswith('bin,')), None)
                if binline:
                    status['bin_counters'] = dict((key, int(value)) for key, value in
                                                  (field.split('=') for field in binline.split(',')[1:]))
                for name in ('invalid', 'missing', 'duplicate', 'upstream_mismatch', 'downstream_mismatch',
                             'att_mismatch', 'att_desync', 'pid_mismatch', 'pid_desync'):
                    assert f'\n{name},0\n' in chain, (name, chain[:900])
            assert not any('InternalError' in e['text'] or 'Internal error' in e['text'] for e in sim.events)
            status['events'] = sim.events
            status['control_pass'] = True
        finally:
            if sim is not None:
                sim.close()
            (output/'status.json').write_text(json.dumps(status, indent=2))
    files = sorted((output/'logs').glob('*.BIN'))
    if args.case == 'no-backend':
        assert not files
        if 'bin_counters' in status:
            assert status['bin_counters']['rejected'] > 0 and status['bin_counters']['accepted'] == 0
        status['success'] = True
        (output/'status.json').write_text(json.dumps(status, indent=2))
        return status
    assert len(files) == 1, files
    result = read_bin(files[0])
    write_results(result, output/'analysis')
    if not enabled:
        assert result['complete'] == result['status_count'] == 0
    else:
        if args.case == 'pressure':
            assert result['errors'], 'Pressure case did not produce measurable logging loss'
            assert status['bin_counters']['rejected'] > 0
            status['pressure_result'] = result['errors']
            (output/'status.json').write_text(json.dumps(status, indent=2))
        else:
            assert result['qualified'], result['errors']
        if args.case != 'pressure':
            assert result['complete'] > 5
        if args.case == 'rearm':
            assert result['final']['Rsn'] == 2
        if args.case == 'mode':
            assert result['final']['Rsn'] == 3
        if args.case == 'rc':
            assert result['final']['Rsn'] in (3, 10)
        if args.case == 'dynamic':
            assert any(abs(r['0_UFF']) > 1e-6 for r in result['rows'])
            assert any(abs(r['0_UDF']) > 1e-6 for r in result['rows'])
            assert any(r['0_Flags'] & 32 for r in result['rows'])
    # Persist only compact raw native motor evidence, not a duplicate full BIN or packet JSON.
    reader = mavutil.mavlink_connection(str(files[0]))
    import csv
    with (output/'native-motors.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['TimeUS', 'C1', 'C2', 'C3', 'C4'])
        while (m := reader.recv_match(type='RCOU')) is not None:
            writer.writerow([m.TimeUS, m.C1, m.C2, m.C3, m.C4])
    reader.close()
    status['success'] = True
    (output/'status.json').write_text(json.dumps(status, indent=2))
    print(args.case, result['complete'], result['qualified'], result['final'])
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=('off', 'absent', 'dynamic', 'mode', 'rc', 'rearm', 'no-backend', 'pressure'),
                        required=True)
    parser.add_argument('--rate', type=int, choices=(10, 20), default=10)
    run(parser.parse_args())


if __name__ == '__main__':
    main()
