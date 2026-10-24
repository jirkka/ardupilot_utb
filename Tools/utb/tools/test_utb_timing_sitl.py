#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_utb_timing_sitl.py
# @brief Owned loopback-only SITL, paired timing OFF/ON; no physical endpoints or arming.
# @author Jiri Kovar
# @version 2026-10-24
import argparse
import hashlib
import json
from pathlib import Path
import socket
import time

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
from test_utb_active_sitl import ActiveSITL
from test_utb_attitude_shadow_sitl import BASE, get_parameter, read_result


def export(sim, destination):
    """Read the immutable timing snapshot after the runtime session stopped."""
    assert get_parameter(sim, 'UTT_RUN') == 0
    ftp = MAVFTP(sim.link, target_system=1, target_component=1)
    ftp.cmd_get(['@SYS/utb_timing.txt', str(destination)])
    response = ftp.process_ftp_reply('get', timeout=15)
    assert response.error_code == 0, response
    return parse(destination.read_text())


def parse(text):
    """Keep counts, per-phase metrics and bounded context without conflating epochs."""
    lines = text.splitlines()
    result = dict(header=lines[0], stats=[], records=[])
    for line in lines[1:]:
        values = line.split(',')
        if values[0] == 'stat':
            if values[1] == 'group':
                keys = values[1:]
            else:
                result['stats'].append(dict(zip(keys, map(int, values[1:]))))
        elif values[0] == 'record':
            if values[1] == 'cycle':
                record_keys = values[1:]
            else:
                result['records'].append(dict(zip(record_keys, map(int, values[1:]))))
        elif len(values) == 2:
            result[values[0]] = int(values[1])
    return result


def check(result):
    """Validate actual capture topology; virtual timing is never presented as hardware cost."""
    assert result['header'] == 'UTBTimingV1,frozen,reason=1', result
    assert result['cycles'] == 4000, result
    for name in ('missing_rate', 'missing_att', 'duplicate_rate', 'duplicate_att', 'missing_finish',
                 'invalid_time', 'cycle_gaps', 'unsupported'):
        assert result[name] == 0, (name, result)
    stats = {(s['group'], s['metric']): s for s in result['stats']}
    assert all(stats[0, k]['count'] == 3999 for k in (0, 1, 2, 4))
    assert stats[0, 3]['count'] == 4000
    assert stats[2, 3]['count'] == 1000
    assert stats[1, 3]['count'] == 3000
    assert result['phase_changes'] == 2
    assert len(result['records']) <= 17
    for previous, current in zip(result['records'], result['records'][1:]):
        assert (current['cycle'] - previous['cycle']) & 0xffffffff == 1
        rate_dt = (current['rate'] - previous['rate']) & 0xffffffff
        att_dt = (current['att'] - previous['att']) & 0xffffffff
        delay = (current['att'] - current['rate']) & 0xffffffff
        previous_delay = (previous['att'] - previous['rate']) & 0xffffffff
        assert att_dt == rate_dt + delay - previous_delay


def run(binary, output, timing):
    """Create one isolated software instance with ordinary native control and DISARMED spool."""
    root = Path(__file__).resolve().parents[3]
    output.mkdir(parents=True, exist_ok=False)
    session = dict(timing_runtime_on=timing, hardware=False, arming=False,
                   binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    sim = None
    started = time.monotonic()
    with (output/'console.log').open('x') as console:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as allocation:
                allocation.bind(('127.0.0.1', 0))
                port = allocation.getsockname()[1]
            settings = BASE + 'UTT_RUN 0\nUTT_COUNT 4000\nUAS_COUNT 1000\n'
            sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', output,
                             console, True, 0, extra_params=settings, model='bfxrev', speedup=5, rcin_port=port)
            sim.owned_rc_port = port
            sim.link.logfile = (output/'receive.tlog').open('xb')
            sim.ready()
            sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
            sim.wait('HEARTBEAT', lambda m: m.custom_mode == 0 and not m.base_mode & 128)
            assert get_parameter(sim, 'UTT_RUN') == 0
            if timing:
                sim.param('UTT_RUN', 1)
            sim.duration(3)
            sim.param('UAS_RUN', 1)
            sim.duration(9)
            shadow = read_result(sim, output/'attitude-shadow.txt')
            assert shadow['attempts'] == 1000 and shadow['valid'] == shadow['seen'] == 2000
            assert shadow['motor_changes'] == shadow['mismatch'] == shadow['desync'] == shadow['invalid'] == 0
            assert all(v['max_abs'] == v['rmse'] == 0 for v in shadow['metrics'])
            (output/'attitude-result.json').write_text(json.dumps(shadow, indent=2))
            result = export(sim, output/'timing.txt')
            if timing:
                check(result)
            else:
                assert result['header'] == 'UTBTimingV1,pending', result
            assert not sim.wait('HEARTBEAT').base_mode & 128
            (output/'timing.json').write_text(json.dumps(result, indent=2))
            sim.duration(3)
            session['success'] = True
        except Exception as error:
            session['success'] = False
            session['error'] = repr(error)
        finally:
            if sim:
                (output/'events.json').write_text(json.dumps(sim.events, indent=2))
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
    # Native DISARMED motor command sets, read only after the owned process stopped.
    motors = set()
    messages = 0
    for filename in (output/'logs').glob('*.BIN'):
        log = mavutil.mavlink_connection(str(filename))
        while (message := log.recv_match(type='RCOU')) is not None:
            row = message.to_dict()
            motors.add(tuple(row.get(f'C{i}') for i in range(1, 5)))
            messages += 1
        log.close()
    session['native_disarmed_motor_sets'] = sorted(motors)
    session['native_motor_messages'] = messages
    session['host_wall_s'] = time.monotonic() - started
    (output/'session.json').write_text(json.dumps(session, indent=2))
    assert session['success'] and messages > 0, session
    return session


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timing', required=True, type=int, choices=(0, 1))
    args = parser.parse_args()
    print(json.dumps(run(args.binary.resolve(), args.output.resolve(), bool(args.timing)), indent=2))
