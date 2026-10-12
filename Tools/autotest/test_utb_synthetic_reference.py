#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Autonomous, no-RC local SITL and independent signal checks; no hardware transport."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import random
import subprocess
import time

from pymavlink import mavutil

from test_utb_math_sitl import GAINS, packet_roundtrip, verify_recurrence
from test_utb_shadow import ShadowSITL, source_audit
from utb_math_reference import U, f32
import utb_math_validation

ENDS = (200, 1800, 3400, 4200, 5000, 5800, 6600, 7400, 7800, 8000)


def reference(cycle, profile):
    if not 0 <= cycle < 8000:
        return 9, [0, 0, 0], [0, 0, 0]
    phase = next(i for i, end in enumerate(ENDS) if cycle < end)
    start = 0 if phase == 0 else ENDS[phase - 1]
    angle = 2 * math.pi * (cycle - start) / 800
    amplitude = f32(.3) if profile == 1 else 8
    values, errors = [0, 0, 0], [0, 0, 0]
    if phase in (1, 2):
        values[0] = amplitude if phase == 1 else -amplitude
    elif phase in (3, 4):
        values[1] = amplitude if phase == 3 else -amplitude
    else:
        functions = {5: ((0, math.sin, 1),), 6: ((2, math.sin, 1),),
                     7: ((0, math.sin, 1), (1, math.cos, 1), (2, math.sin, 2))}
        for axis, function, multiple in functions.get(phase, ()):
            values[axis] = amplitude * function(angle * multiple)
            gamma = 6 * U / (1 - 6 * U)
            errors[axis] = amplitude * (gamma * abs(angle * multiple) + U) + U * abs(values[axis])
    return phase, values, errors


def check_signals(binary, output):
    env = os.environ.copy()
    env['UTB_REFERENCE_OUTPUT'] = str((output / 'signals.csv').resolve())
    with (output / 'unit-synthetic.log').open('x') as log:
        subprocess.run([str(binary.resolve())], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    maximum = 0
    with (output / 'signals.csv').open() as stream:
        data = list(csv.reader(stream))
    assert len(data) == 16000
    for profile, cycle, phase, *actual in data:
        section, expected, errors = reference(int(cycle), int(profile))
        assert int(phase) == section
        for value, target, bound in zip(actual, expected, errors):
            error = abs(f32(float(value)) - target)
            assert error <= bound, (profile, cycle, error, bound)
            maximum = max(maximum, error)
    return dict(compared_signal_vectors=len(data), max_absolute_error=maximum,
                tolerance='gamma_6 float32 angle error, sin/cos Lipschitz bound, one libm/output rounding')


def measurement_vectors():
    randomizer = random.Random(471)
    rows = []
    for n in range(8000):
        target = reference(n, 1)[1]
        time = n * .0025
        measurement = [.12 * math.sin(2 * math.pi * time + .7) + randomizer.uniform(-.002, .002),
                       .06 * time if n < 4000 else .6, -.1 if 3000 <= n < 3200 else 0]
        rows.append(dict(scenario=0, sample=n, action=3 if n == 6000 else 0, frame=18,
                         dt=f32(.0025 + (n % 17 - 8) * .000003), thrust=f32(.5),
                         target=list(map(f32, target)), measurement=list(map(f32, measurement)), gains=GAINS))
    return rows, ['synthetic-reference-and-isolated-measurement']


class NoRC(ShadowSITL):
    def rc(self):
        # Genuine host GCS heartbeat only. No RC overrides or native RC pulses.
        now = time.monotonic()
        if now - getattr(self, '_heartbeat_time', 0) >= 1:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._heartbeat_time = now


BASE = ('FRAME_TYPE 18\nSIM_RC_FAIL 1\nFS_THR_ENABLE 1\nFS_GCS_ENABLE 1\nLOG_DISARMED 1\nLOG_BITMASK 5225\n'
        'UTB_SHADOW 1\nUTB_LOG_FMT 1\nUTB_LOG_RATE 400\nUTB_B_SYNTH 0\n'
        'SIM_GYR1_RND 0\nSIM_GYR2_RND 0\nSIM_ACC1_RND 0\nSIM_ACC2_RND 0\nSIM_MAG_RND 0\n')
for axis in ('RLL', 'PIT', 'YAW'):
    for name, value in zip(('P', 'I', 'D', 'IMAX', 'D_HZ'), GAINS[0]):
        BASE += f'UTB_RAT_{axis}_{name} {value}\n'


def read_log(directory):
    records = {}
    for path in (directory / 'logs').glob('*.BIN'):
        reader = mavutil.mavlink_connection(str(path))
        while True:
            message = reader.recv_match()
            if message is None:
                break
            kind = message.get_type()
            if kind.startswith(('UTB', 'UB')) or kind in ('PARM', 'PM', 'RCOU', 'RCIN'):
                records.setdefault(kind, []).append(message.to_dict())
    return records


def exercise(sim):
    sim.health(lambda flags, calc, obs, policy: calc == 4 and bool(flags & 4))
    sim.param('UTB_B_SYNTH', 1)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.duration(21)
    sim.health(lambda flags, calc, obs, policy: calc == 4 and not flags & 64)
    sim.param('UTB_B_SYNTH', 0)
    sim.duration(1)
    sim.param('UTB_LOG_RATE', 200)
    sim.param('UTB_B_SYNTH', 2)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.duration(21)
    sim.param('UTB_B_SYNTH', 0)
    sim.param('UTB_LOG_RATE', 400)
    sim.param('UTB_B_SYNTH', 1)
    sim.duration(2)
    sim.mode(1, 1)
    sim.health(lambda flags, calc, obs, policy: calc == 4 and not flags & 64)
    sim.param('UTB_B_SYNTH', 0)
    sim.param('UTB_B_SYNTH', 1)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.param('AHRS_EKF_TYPE', 2)
    sim.health(lambda flags, calc, obs, policy: calc == 2 and not flags & 64)
    sim.param('AHRS_EKF_TYPE', 3)
    sim.health(lambda flags, calc, obs, policy: calc == 4 and not flags & 64)
    sim.param('UTB_B_SYNTH', 0)
    sim.mode(40, 40)
    assert sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.observe(40, False)
    sim.mode(0, 0)
    sim.param('UTB_SHADOW', 0)
    sim.duration(2)


def identity(packet, metadata):
    anchors = [m for m in metadata if m['Epoch'] == packet['Epoch'] and m['Test'] == 2]
    terminal = [m['Seq'] for m in metadata if m['Epoch'] == packet['Epoch'] and m['Test'] in (3, 4)]
    if not anchors or (terminal and packet['Seq'] >= min(terminal)):
        return None
    anchor = min(anchors, key=lambda m: abs(m['Seq'] - packet['Seq']))
    cycle = anchor['Cycle'] + packet['Seq'] - anchor['Seq']
    return (anchor['Prof'], cycle) if 0 <= cycle < 8000 else None


def analyse(records):
    metadata = records['UTBX']
    packets = records['UTBC']
    decoded = [packet_roundtrip(p) for p in packets]
    pairs, maxima, profiles, cycles, valid_count = 0, {}, {}, {}, 0
    for i, packet in enumerate(packets):
        source = identity(packet, metadata)
        if source is None:
            continue
        profile, cycle = source
        _, target, bounds = reference(cycle, profile)
        for axis in range(3):
            assert abs(decoded[i][axis]['Des'] - target[axis]) <= bounds[axis]
        assert packet['RCms'] == 0, 'Unexpected real or overridden RC input'
        profiles[profile] = profiles.get(profile, 0) + 1
        cycles.setdefault(profile, set()).add(cycle)
        valid_count += int(bool(decoded[i][3]['Val']))
        if i and packets[i - 1]['Epoch'] == packet['Epoch'] and packets[i - 1]['Seq'] + 1 == packet['Seq']:
            if all(r['Val'] for r in decoded[i - 1][:3] + decoded[i][:3]):
                for name, error in verify_recurrence(decoded[i - 1], decoded[i]).items():
                    maxima[name] = max(maxima.get(name, 0), error)
                pairs += 1
    assert profiles.get(1, 0) > 7000 and profiles.get(2, 0) > 3500, profiles
    assert pairs > 7000, pairs
    assert {m['Phase'] for m in metadata if m['Prof'] == 1 and m['Test'] == 2} == set(range(10))
    assert {3, 4} <= {m['Test'] for m in metadata}
    assert not any(s['Pol'] == 2 for s in records['UTBS'])
    assert all(tuple(r[f'C{j}'] for j in range(1, 5)) == (1000,) * 4 for r in records['RCOU'])
    parameters = {r['Name']: r['Value'] for r in records['PARM']}
    assert parameters['FS_THR_ENABLE'] == 1 and parameters['FS_GCS_ENABLE'] == 1 and parameters['SIM_RC_FAIL'] == 1
    assert any(abs(d[0]['I']) == GAINS[0][3] for d in decoded), 'IMAX not reached'
    assert any(d[0]['Sat'] & 4 for d in decoded), 'Saturation gate not exercised'
    return dict(packets=len(packets), valid_synthetic_packets=valid_count, profile_packets=profiles,
                compared_pid_pairs=pairs, max_absolute_errors=maxima, phases='all 10', rc_timestamp_all_zero=True,
                radio_failsafe_enabled=True, disarmed_outputs_unchanged=True, IMAX_reached=True,
                completion_and_abort_latched=True, time_budget_latch=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--unit-binary', type=Path, required=True)
    parser.add_argument('--math-binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(signals=check_signals(args.unit_binary, args.output))
    utb_math_validation.scenarios = measurement_vectors
    utb_math_validation.validate(args.math_binary, args.output / 'isolated-measurement')
    directory = args.output / 'no-rc'
    directory.mkdir()
    root = Path(__file__).resolve().parents[2]
    with (directory / 'console.log').open('x') as console:
        sim = NoRC(args.binary.resolve(), root / 'Tools/autotest/default_params/copter.parm', directory,
                   console, True, 1, extra_params=BASE, model='bfxrev', speedup=1)
        try:
            exercise(sim)
        finally:
            sim.close()
    result['sitl'] = analyse(read_log(directory))
    result['source_audit'] = source_audit(root)
    (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
