#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @author Jiri Kovar
# @version 2026-10-25
"""Owned loopback SITL only; finite DISARMED execution timing and lifecycle checks."""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
import test_utb_skeleton
from test_utb_active_sitl import ActiveSITL
from test_utb_attitude_shadow_sitl import BASE, get_parameter, set_mode
from test_utb_flight_shadow_sitl import fetch
from test_utb_active_chain_sitl import hold, sticks
from utb_execution_timing_analysis import parse, write
from utb_flight_shadow_analysis import parse as flight_parse, check_supported


def start_sim(*args, short_wipe=False, **kwargs):
    """Use the HAL-supported short wipe option when testing an actual in-place SITL reboot."""
    original = test_utb_skeleton.subprocess.Popen
    if short_wipe:
        def launch(argv, *positional, **named):
            argv = ['-w' if item == '--wipe' else item for item in argv]
            return original(argv, *positional, **named)
        test_utb_skeleton.subprocess.Popen = launch
    try:
        return ActiveSITL(*args, **kwargs)
    finally:
        test_utb_skeleton.subprocess.Popen = original


def run(binary, output, case):
    """Preserve real assertions and failures; simulated ARM never opens a hardware endpoint."""
    root = Path(__file__).resolve().parents[3]
    output.mkdir(parents=True, exist_ok=False)
    status = dict(success=False, hardware=False, case=case, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    sim = None
    started = time.monotonic()
    with (output / 'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                port = allocation.getsockname()[1]
            settings = BASE + 'UCS_F_RUN 0\nUCS_RUN 0\nUTT_RUN 0\nUTT_COUNT 4000\n'
            if case == 'joint-dynamic':
                settings += ('ATC_RAT_RLL_FF 0.02\nATC_RAT_PIT_FF 0.02\nATC_RAT_YAW_FF 0.01\n'
                             'ATC_RAT_RLL_D_FF 0.001\n')
            if case == 'unsupported':
                settings += 'FSTRATE_ENABLE 1\n'
            sim = start_sim(binary, root / 'Tools/autotest/default_params/copter.parm', output, console, True, 1,
                            extra_params=settings, model='bfxrev', speedup=4, rcin_port=port,
                            isolated_aux_ports=True, short_wipe=case == 'restart')
            sim.link.logfile = (output / 'receive.tlog').open('xb')
            sim.ready()
            status['pid'] = sim.process.pid
            status['boot_before'] = get_parameter(sim, 'STAT_BOOTCNT')
            set_mode(sim, 0)
            pending = fetch(sim, 'utb_execution.txt', output / 'pending.txt')
            assert pending == 'UTBExecutionV1,pending\n'
            if case == 'baseline':
                hold(sim, 3)
                assert fetch(sim, 'utb_execution.txt', output / 'still-pending.txt') == pending
            else:
                count = 400 if case in ('timing', 'joint', 'repeated', 'restart', 'interrupted') else 8000
                if case == 'joint-dynamic':
                    count = 4000
                if case == 'invalid-count':
                    count = 0
                sim.param('UTT_COUNT', count)
                if case in ('joint', 'joint-dynamic'):
                    sim.param('UCS_F_COUNT', count)
                    sim.param('UCS_F_DECIM', 40)
                sim.param('UTT_RUN', 1)
                if case in ('joint', 'joint-dynamic'):
                    sim.param('UCS_F_RUN', 1)
                if case == 'joint-dynamic':
                    marks = []
                    for _ in range(2):
                        sticks(sim, marks, 'combined_positive', 1, roll=1600, pitch=1400, yaw=1560)
                        sticks(sim, marks, 'combined_negative', 1, roll=1400, pitch=1600, yaw=1440)
                        sticks(sim, marks, 'neutral', 1)
                    status['rc_marks'] = marks
                reason = 6 if case == 'unsupported' else 4 if case == 'invalid-count' else 1
                if case in ('cancel', 'config', 'mode', 'arm'):
                    hold(sim, 1)
                    if case == 'cancel':
                        assert fetch(sim, 'utb_execution.txt', output / 'during.txt') == pending
                        sim.param('UTT_RUN', 0)
                        reason = 2
                    elif case == 'config':
                        sim.param('UTT_COUNT', count - 1)
                        reason = 7
                    elif case == 'mode':
                        set_mode(sim, 9)
                        reason = 6
                    else:
                        sim.standard_arm()
                        hold(sim, 1)
                        assert get_parameter(sim, 'UTT_RUN') == 0
                        assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0) == 0
                        reason = 2
                hold(sim, count / 400 + 2 if reason == 1 else 2)
                assert get_parameter(sim, 'UTT_RUN') == 0
                text = fetch(sim, 'utb_execution.txt', output / 'execution.txt')
                assert get_parameter(sim, 'STAT_BOOTCNT') == status['boot_before'], 'Restart during export'
                result = parse(text)
                status['execution_session_us'] = result['header']['session_us']
                write(result, output / 'analysis')
                assert result['header']['reason'] == reason, result
                if reason == 1:
                    assert result['header']['cycles'] == count
                if case in ('joint', 'joint-dynamic'):
                    assert get_parameter(sim, 'UCS_F_RUN') == 0
                    metadata = fetch(sim, 'utb_flight.txt', output / 'flight.txt')
                    pages = [fetch(sim, f'utb_flight{k}.csv', output / f'flight{k}.csv') for k in range(8)]
                    flight = flight_parse(metadata, pages)
                    check_supported(flight)
                    assert flight['scalars']['attempts'] == count
                    assert result['metrics'][5]['count'] > 0
                    if case == 'joint-dynamic':
                        assert flight['scalars']['att_shaping'] > 0
                        assert any(abs(row['UTB0FF']) > 1e-6 for row in flight['rows'] if row['Flags'] & 1)
                        assert any(abs(row['UTB0DFF']) > 1e-6 for row in flight['rows'] if row['Flags'] & 1)
                        assert flight['scalars']['independent_i'] == 0  # Native DISARMED reset remains active.
                    status['flight'] = {k: v for k, v in flight.items() if k != 'rows'}
                elif reason == 1:
                    assert result['metrics'][5]['count'] == 0
                if case == 'interrupted':
                    original_receive = sim.link.recv_match
                    dropped = [0]

                    def lose_ftp(*args, **kwargs):
                        message = original_receive(*args, **kwargs)
                        if message and message.get_type() == 'FILE_TRANSFER_PROTOCOL':
                            dropped[0] += 1
                            return None
                        return message

                    sim.link.recv_match = lose_ftp
                    try:
                        ftp = MAVFTP(sim.link, target_system=1, target_component=1)
                        ftp.cmd_get(['@SYS/utb_execution.txt', str(output / 'interrupted.txt')])
                        reply = ftp.process_ftp_reply('get', timeout=5)
                        assert reply.error_code != 0
                    finally:
                        sim.link.recv_match = original_receive
                    assert dropped[0] > 0
                    hold(sim, 1)
                    assert fetch(sim, 'utb_execution.txt', output / 'after-interruption.txt') == text
                    status['dropped_ftp_responses'] = dropped[0]
                if case == 'repeated':
                    sim.param('UTT_RUN', 1)
                    hold(sim, 1)
                    assert get_parameter(sim, 'UTT_RUN') == 0
                    assert fetch(sim, 'utb_execution.txt', output / 'after-retrigger.txt') == text
                if case == 'restart':
                    # Only the owned loopback process receives a reboot command; never a physical endpoint.
                    first_boot = get_parameter(sim, 'STAT_BOOTCNT')
                    address = sim.link.address
                    process_id = sim.process.pid
                    sim.command(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
                    if sim.link.logfile:
                        sim.link.logfile.close()
                    sim.link.close()
                    time.sleep(2)
                    sim.link = mavutil.mavlink_connection(address, source_system=255)
                    sim.link.logfile = (output / 'after-reboot.tlog').open('xb')
                    sim.wait('HEARTBEAT')
                    sim.link.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
                    sim.ready()
                    second_boot = get_parameter(sim, 'STAT_BOOTCNT')
                    assert second_boot > first_boot and sim.process.pid == process_id
                    assert fetch(sim, 'utb_execution.txt', output / 'after-restart.txt') == pending
                    status['restart_boots'] = [first_boot, second_boot]
            assert not sim.wait('HEARTBEAT').base_mode & 128
            status['boot_after'] = get_parameter(sim, 'STAT_BOOTCNT')
            if case != 'restart':
                assert status['boot_after'] == status['boot_before']
            status['success'] = True
        except Exception as error:
            status['error'] = repr(error)
            raise
        finally:
            if sim:
                (output / 'events.json').write_text(json.dumps(sim.events, indent=2))
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
            status['host_wall_s'] = time.monotonic() - started
            (output / 'session.json').write_text(json.dumps(status, indent=2))
    return status


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    cases = ('baseline', 'timing', 'joint', 'joint-dynamic', 'cancel', 'config', 'mode', 'arm', 'repeated',
             'restart', 'interrupted', 'unsupported', 'invalid-count')
    parser.add_argument('--case', choices=cases, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.binary.resolve(), args.output.resolve(), args.case), indent=2))
