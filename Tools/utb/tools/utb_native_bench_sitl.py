#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later. Author: Jiri Kovar, revision 2026-10-19.
"""One owned DISARMED SITL benchmark. No serial or arming command interface."""
import argparse
from pathlib import Path

from pymavlink import mavutil
from test_utb_active_sitl import launch
from utb_native_bench_analysis import analyse


def exercise(sim):
    heartbeat = sim.wait('HEARTBEAT')
    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    sim.param('UB3_RUN', 1)
    sim.duration(43)
    sim.link.mav.param_request_read_send(1, 1, b'UB3_RUN', -1)
    sim.wait('PARAM_VALUE', lambda m: m.param_id == 'UB3_RUN' and m.param_value == 0)
    heartbeat = sim.wait('HEARTBEAT')
    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def suppressed(sim):
    sim.param('UB3_RUN', 1)
    sim.duration(1)
    sim.param('LOG_DISARMED', 0)
    sim.duration(1)
    sim.link.mav.param_request_read_send(1, 1, b'UB3_RUN', -1)
    sim.wait('PARAM_VALUE', lambda m: m.param_id == 'UB3_RUN' and m.param_value == 0)
    sim.param('LOG_DISARMED', 1)
    sim.duration(1)
    sim.link.mav.param_request_read_send(1, 1, b'UB3_RUN', -1)
    sim.wait('PARAM_VALUE', lambda m: m.param_id == 'UB3_RUN' and m.param_value == 0)
    heartbeat = sim.wait('HEARTBEAT')
    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=['comparison', 'suppression'], default='comparison')
    args = parser.parse_args()
    args.output = args.output.resolve()
    launch(args.binary.resolve(), args.output, 'disarmed', exercise if args.stage == 'comparison' else suppressed,
           extra='LOG_DISARMED 1\nUTB_SHADOW 0\nUB3_RUN 0\n', speedup=1)
    logs = list((args.output/'disarmed/logs').glob('*.BIN'))
    assert len(logs) == 1
    result = analyse(logs[0], args.output/'analysis', 'SITL')
    if args.stage == 'suppression':
        assert 0 < result['complete_sets'] < 2000
        print('DISARMED logging-policy suppression and no automatic restart verified')
        raise SystemExit(0)
    assert result['complete_sets'] == 2000
    assert result['partial_sets'] == 0
    assert result['excluded_input_or_health_sets'] == 0
    assert result['nonfinite_values'] == 0
    assert not result['missing_sequence_numbers']
    print('Complete DISARMED SITL experiment and offline analysis; inspect numerical differences in summary.json')
