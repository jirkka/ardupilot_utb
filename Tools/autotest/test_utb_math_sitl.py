#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Nonzero PID validation using exclusively self-launched local SITL instances."""
import argparse
import json
import math
from pathlib import Path
import struct

import test_utb_shadow as shadow
from utb_log_decode import AXIS, TAIL, decode_packed
from utb_math_reference import Bound, Reference, U, f32

GAINS = [(f32(.31), f32(.23), f32(.004), f32(.19), f32(20))] * 3
BASE = ("FRAME_TYPE 18\nSCHED_LOOP_RATE 400\nLOG_DISARMED 1\nLOG_BITMASK 5225\n"
        "UTB_SHADOW 1\nUTB_LOG_RATE 400\nUTB_LOG_FMT 1\nACRO_TRAINER 0\nEK3_IMU_MASK 1\n"
        "SIM_GYR1_RND 0\nSIM_GYR2_RND 0\nSIM_ACC1_RND 0\nSIM_ACC2_RND 0\nSIM_MAG_RND 0\n")
for axis in ('RLL', 'PIT', 'YAW'):
    for name, value in zip(('P', 'I', 'D', 'IMAX', 'D_HZ'), GAINS[0]):
        BASE += f'UTB_RAT_{axis}_{name} {value}\n'


def packet_roundtrip(packet):
    decoded = decode_packed(packet)
    axis_fields = ('Des', 'Meas', 'Err', 'P', 'I', 'D', 'Raw', 'Out', 'AP', 'Val', 'Sat')
    payload = b''.join(AXIS.pack(*(decoded[j][name] for name in axis_fields))
                       for j in range(3))
    motor, achieved, timing = decoded[3:6]
    motor_fields = ('R', 'P', 'Y', 'T', 'M1', 'M2', 'M3', 'M4', 'S', 'Shift', 'Val', 'Lo', 'Hi')
    payload += TAIL.pack(*(motor[name] for name in motor_fields),
                         *(achieved[name] for name in ('R', 'P', 'Y', 'T', 'Pos', 'Neg', 'Scale', 'Shift')),
                         *(timing[name] for name in ('Dt', 'Ex', 'Req', 'Max')), 1)
    original = struct.pack('<96h', *(list(packet['D0']) + list(packet['D1']) + list(packet['D2'])))
    assert payload == original, 'Decoder changed a payload bit'
    assert all(r['Seq'] == packet['Seq'] and r['TimeUS'] == packet['TimeUS'] for r in decoded)
    assert decoded[-1]['Epoch'] == packet['Epoch']
    for name in ('APUS', 'APS', 'APM', 'Cap', 'RCms', 'Eff'):
        assert timing[name] == packet[name]
    return decoded


def verify_recurrence(previous, current):
    model = Reference()
    model.configuration = GAINS
    model.previous = [r['Meas'] for r in previous[:3]]
    model.integral = [Bound(r['I']) for r in previous[:3]]
    # D is logged after a float32 multiplication. Invert it with that one rounding uncertainty.
    model.acceleration = [Bound(-r['D'] / GAINS[j][2], U * abs(r['D'] / GAINS[j][2]) / (1 - U))
                          for j, r in enumerate(previous[:3])]
    model.residual = [Bound(r['Raw']) - previous[4][name] for r, name in zip(previous[:3], ('R', 'P', 'Y'))]
    row = dict(sample=1, action=0, frame=18, dt=current[5]['Dt'], thrust=current[3]['T'], gains=GAINS,
               target=[r['Des'] for r in current[:3]], measurement=[r['Meas'] for r in current[:3]])
    expected = model.step(row)
    values = {}
    for j, r in enumerate(current[:3]):
        for field, name in zip(('e', 'p', 'i', 'd', 'raw', 'out'), ('Err', 'P', 'I', 'D', 'Raw', 'Out')):
            values[f'{field}{j}'] = r[name]
        values[f'blocked{j}'] = (r['Sat'] >> 2) & 1
    values.update({f'm{i}': current[3][f'M{i + 1}'] for i in range(4)})
    values.update({f'a{i}': current[4][name] for i, name in enumerate(('R', 'P', 'Y'))})
    values.update(at=current[4]['T'], scale=current[3]['S'], shift=current[3]['Shift'],
                  pos=current[4]['Pos'], neg=current[4]['Neg'], lo=current[3]['Lo'], hi=current[3]['Hi'],
                  scaled=current[4]['Scale'], shifted=current[4]['Shift'])
    errors = {}
    for field, value in values.items():
        bound = expected[field]
        if isinstance(bound, Bound):
            error = abs(value - bound.value)
            assert error <= bound.error, (current[0]['Seq'], field, value, bound)
            errors[field] = error
        else:
            assert value == bound, (current[0]['Seq'], field, value, bound)
    return errors


def analyse(records):
    packets = records['UTBC']
    decoded = [packet_roundtrip(p) for p in packets]
    pairs, maxima = 0, {}
    for i in range(1, len(packets)):
        previous, current = packets[i - 1:i + 1]
        if (current['Seq'] == previous['Seq'] + 1 and current['Epoch'] == previous['Epoch']
                and current['Md'] == previous['Md'] and all(r['Val'] for r in decoded[i - 1][:3] + decoded[i][:3])):
            for name, error in verify_recurrence(decoded[i - 1], decoded[i]).items():
                maxima[name] = max(maxima.get(name, 0), error)
            pairs += 1
    assert pairs > 4000, pairs
    rates = records['UTBR']
    assert all(math.isfinite(r[k]) for r in rates for k in ('Des', 'Meas', 'Err', 'P', 'I', 'D', 'Raw', 'Out'))
    assert all(abs(r['I']) <= GAINS[0][3] and abs(r['Out']) <= 1 for r in rates)
    assert any(r['Sat'] == 0 and r['Val'] for r in rates)
    assert any(r['Sat'] != 0 and r['Val'] for r in rates)
    assert any(r['Sat'] & 4 for r in rates), 'Directional integration gate never exercised'
    assert any(abs(r['P']) > .01 for r in rates)
    assert any(abs(r['I']) > .001 for r in rates)
    # Disarmed gyro is nearly constant; nonzero D dynamics are proven by synthetic tests, not forced here.
    states = records['UTBS']
    assert any(s['Calc'] == 6 for s in states), 'Missing frame invalidation'
    assert any(s['Calc'] == 2 for s in states), 'Missing state invalidation'
    first_healthy = min(s['TimeUS'] for s in states if s['Calc'] == 0 and s['Flg'] & 256)
    assert any(s['Calc'] == 2 and s['TimeUS'] > first_healthy for s in states), 'No runtime state invalidation'
    assert not any(s['Pol'] == 2 for s in states), 'Execution budget guard latched'
    assert any(s['Calc'] == 0 and s['TimeUS'] > max(t['TimeUS'] for t in states if t['Calc'] == 2) for s in states)
    epochs = sorted({p['Epoch'] for p in packets})
    assert len(epochs) >= 3, epochs
    for motors in records['UTBM']:
        assert all(0 <= motors[f'M{i}'] <= 1 for i in range(1, 5))
        if not motors['Val']:
            assert all(motors[f'M{i}'] == 0 for i in range(1, 5))
    seq_steps = {}
    for a, b in zip(packets, packets[1:]):
        if a['Epoch'] == b['Epoch']:
            step = b['Seq'] - a['Seq']
            seq_steps[str(step)] = seq_steps.get(str(step), 0) + 1
    assert seq_steps.get('1', 0) > 4000
    return dict(packets=len(packets), payload_bit_roundtrips=len(packets), compared_adjacent_pairs=pairs,
                max_absolute_error=maxima, epochs=epochs, sequence_steps=seq_steps,
                saturation_and_gate_exercised=True, state_invalidation_recovery=True,
                time_budget_latch=False, d_dynamic_claim=False)


def exercise(sim):
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    for roll, pitch, yaw in ((1500, 1500, 1500), (1600, 1400, 1550), (1900, 1900, 1900),
                             (1100, 1100, 1100), (1500, 1500, 1500)):
        sim.roll, sim.pitch, sim.yaw, sim.throttle = roll, pitch, yaw, 1500
        sim.duration(3)
    sim.param('FRAME_TYPE', 1)
    sim.health(lambda flags, calc, obs, policy: calc == 6 and not flags & 64)
    sim.param('FRAME_TYPE', 18)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    # Select the existing disabled EKF2 backend temporarily: AHRS.healthy() becomes false.
    # No filter implementation or UTB validity rule is changed; this is a virtual disarmed fault.
    sim.param('AHRS_EKF_TYPE', 2)
    sim.health(lambda flags, calc, obs, policy: calc == 2 and not flags & 64)
    sim.param('AHRS_EKF_TYPE', 3)
    sim.health(lambda flags, calc, obs, policy: bool(flags & 256))
    sim.param('UTB_LOG_RATE', 200)
    sim.duration(3)
    sim.param('UTB_LOG_FMT', 0)
    sim.duration(3)
    sim.param('UTB_LOG_FMT', 1)
    sim.param('UTB_LOG_RATE', 400)
    sim.duration(3)
    sim.throttle = 1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True, exist_ok=False)
    records, statuses = shadow.launch(args.binary.resolve(), root, args.output.resolve(), 'nonzero', BASE,
                                      exercise, model='bfxrev')
    result = dict(source_audit=shadow.source_audit(root), nonzero=analyse(records), health_states=statuses)
    # Reuse identical fixed RC and ground-idle comparison; all axes now have nonzero P/I/D.
    shadow.BASE = BASE
    result['motor_isolation'] = shadow.isolation(args.binary.resolve(), root, args.output.resolve(), model='bfxrev')
    (args.output / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
