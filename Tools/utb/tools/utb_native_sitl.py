#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""AP-derived UTB flight-path regression; owns a local SITL executable only."""
# @author Jiri Kovar
# @version 2026-10-19
# @date 2026-10-19
# GNU GPL version 3 or later.
import argparse
import bisect
from functools import partial
import hashlib
import json
import math
from pathlib import Path

from pymavlink import mavutil
import utb_angle_sitl as angle
import utb_pilot_sitl as pilot
import utb_pilot_tuning as tuning

# Match the existing AP copter.parm yaw configuration; this is not a PID tune.
PARAMS = 'UT3_RAT_YAW_P 0.3\nUT3_RAT_YAW_I 0.02\nARMING_RUDDER 0\n'


def gate(sim):
    """Prove the independent parameter bank and physical disarmed isolation."""
    assert abs(tuning.read_parameter(sim, 'UT3_RAT_RLL_P') - .135) < 1e-6
    for mode in (42, 43):
        sim.select(mode)
        for axis in ('roll', 'pitch', 'yaw'):
            for sign in (-1, 1):
                sim.sticks(**{axis: sign}, seconds=.5)
        sim.sticks(seconds=.5)
    for mode, parameter in ((42, 'UTB_A_ENABLE'), (43, 'UTB_C_ENABLE')):
        sim.select(mode)
        sim.param(parameter, 0)
        assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
        assert sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED
        sim.param(parameter, 1)
    sim.select(0)
    sim.param('UTB_A_ENABLE', 0)
    sim.mode(42, 0)
    sim.param('UTB_A_ENABLE', 1)
    sim.param('UTB_C_ENABLE', 0)
    sim.mode(43, 0)
    sim.param('UTB_C_ENABLE', 1)
    sim.select(40)
    assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
    assert sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.select(0)


def disabled(sim):
    """A fresh boot with the original UTB_ENABLE opt-in disabled must refuse both pilot modes."""
    assert tuning.read_parameter(sim, 'UTB_ENABLE') == 0
    for mode in (42, 43):
        sim.mode(mode, 0)
    sim.sticks(1, -1, 1, seconds=1)
    sim.sticks(seconds=1)


def ekf_reset(sim):
    """Original EK3_PRIMARY policy forces healthy primary selection while DISARMED only."""
    sim.select(42)
    for primary in (1, 0):
        sim.param('EK3_PRIMARY', primary)
        sim.wait('STATUSTEXT', lambda m: m.text.startswith(f'EKF primary changed:{primary}'), timeout=15)
        msg = observe(sim, 2)
        assert max(abs(msg.roll), abs(msg.pitch)) < math.radians(3)
    sim.select(0)


def ground_arming(sim):
    """Use ordinary prearm/arm/disarm in each UTB pilot mode, with zero throttle on the simulated ground."""
    sim.wait('EKF_STATUS_REPORT', lambda m: m.flags & (16 | 512) and not m.flags & 1024, timeout=45)
    sim.throttle = 1000
    for mode in (42, 43):
        sim.select(mode)
        sim.standard_arm()
        observe(sim, 2)
        assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
        sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
        observe(sim, 1)
    sim.select(0)


def prepare(sim, height=20):
    """Runner envelope only, never firmware fail-stop thresholds."""
    sim.pilot_tilt_limit_deg = 180
    sim.pilot_gyro_limit_rads = math.radians(1000) * math.sqrt(3)
    sim.pilot_altitude_delta_m = height - 2
    sim.pilot_vertical_speed_mps = 20
    sim.pilot_min_height_m = 1
    sim.takeoff_height_m = height
    sim.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20000)
    angle.prepare_flight(sim)


def observe(sim, seconds):
    """Measure simulated maneuver duration using actual attitude timestamps."""
    first = sim.wait('ATTITUDE')
    msg = first
    while ((msg.time_boot_ms - first.time_boot_ms) & 0xffffffff) * .001 < seconds:
        msg = sim.wait('ATTITUDE')
    return msg


def pilot_flight(sim, mode, axis, sign):
    """Small AP-native reference, manual throttle, explicit ANGLE self-level."""
    prepare(sim)
    sim.select(mode, armed=True, settle=False)
    observe(sim, .5)
    setattr(sim, ('roll', 'pitch')[axis], angle.pwm(sign * (.1 if mode == 43 else .5)))
    observe(sim, .5 if mode == 43 else 2)
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.rc()
    sim.select(42, armed=True, settle=False)
    msg = observe(sim, 4)
    assert max(abs(msg.roll), abs(msg.pitch)) < math.radians(3), 'Self-level did not converge'
    angle.finish_flight(sim)


def rotation(sim, axis, sign):
    """One measured body-axis rotation; no Euler wrapping shortcut."""
    prepare(sim, height=40)
    sim.select(43, armed=True, settle=False)
    observe(sim, .3)
    previous = sim.wait('ATTITUDE')
    start = previous.time_boot_ms
    total = 0.
    setattr(sim, ('roll', 'pitch')[axis], angle.pwm(sign))
    while sign * total < math.tau:
        msg = sim.wait('ATTITUDE')
        dt = ((msg.time_boot_ms - previous.time_boot_ms) & 0xffffffff) * .001
        assert 0 < dt < .1
        field = ('rollspeed', 'pitchspeed')[axis]
        total += .5 * (getattr(previous, field) + getattr(msg, field)) * dt
        previous = msg
        assert ((msg.time_boot_ms - start) & 0xffffffff) < 4000, 'Rotation not achieved'
    sim.marks.append(dict(kind='measured_rotation', axis=axis, sign=sign, degrees=math.degrees(total)))
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.rc()
    observe(sim, .5)
    sim.select(42, armed=True, settle=False)
    msg = observe(sim, 5)
    assert max(abs(msg.roll), abs(msg.pitch)) < math.radians(3)
    angle.finish_flight(sim)


def manual_throttle(sim):
    """Both pilot modes forward real RC throttle; no altitude controller is supplied by the runner."""
    prepare(sim, height=30)
    for mode in (43, 42):
        sim.select(mode, armed=True, settle=False)
        for pwm in (1400, 1600, 1500):
            sim.throttle = pwm
            sim.rc()
            observe(sim, 2)
            sim.marks.append(dict(kind='manual_throttle', mode=mode, pwm=pwm, boot_ms=sim.last_boot_ms))
    angle.finish_flight(sim)


def angle_limits(sim):
    """Change the actual native-style lean parameter, then verify both axes/signs and self-level."""
    sim.param('UT3_ANGLE_MAX', 45)
    prepare(sim)
    sim.select(42, armed=True, settle=False)
    for axis in ('roll', 'pitch'):
        for sign in (-1, 1):
            setattr(sim, axis, angle.pwm(sign))
            msg = observe(sim, 3)
            assert abs(getattr(msg, axis) - math.radians(sign * 45)) < math.radians(3)
            sim.marks.append(dict(kind='angle_limit', axis=axis, sign=sign,
                                  actual_deg=math.degrees(getattr(msg, axis)), maximum_deg=45))
            sim.roll = sim.pitch = sim.yaw = 1500
            sim.rc()
            msg = observe(sim, 4)
            assert max(abs(msg.roll), abs(msg.pitch)) < math.radians(3)
    angle.finish_flight(sim)


def transitions(sim):
    """Direct switches at nonzero sticks and native STABILIZE handoff."""
    prepare(sim)
    sim.select(43, armed=True, settle=False)
    sim.roll, sim.pitch, sim.yaw = angle.pwm(.05), angle.pwm(-.05), angle.pwm(.03)
    for mode in (42, 43, 42, 0, 43, 42):
        observe(sim, .3)
        pilot.switch(sim, mode, rc=True, armed=True)
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.rc()
    msg = observe(sim, 4)
    assert max(abs(msg.roll), abs(msg.pitch)) < math.radians(3)
    angle.finish_flight(sim)


def failsafe(sim, mode, source):
    """Actual native RC or GCS loss must select RTL and stay there after contact returns."""
    prepare(sim)
    sim.enable_native_receiver()
    sim.select(mode, armed=True, settle=False)
    observe(sim, 1)
    sim.expected_mode = None
    sim.bench_altitude = None
    if source == 'rc':
        sim.param('SIM_RC_FAIL', 1)
        sim.send_rc = False
    else:
        sim.send_heartbeat = False
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 6, timeout=12)
    observe(sim, 1)
    sim.send_rc = sim.send_heartbeat = True
    if source == 'rc':
        sim.param('SIM_RC_FAIL', 0)
    observe(sim, 1)
    assert sim.wait('HEARTBEAT').custom_mode == 6
    angle.finish_flight(sim)


def analyse(records, disarmed):
    """Validate actual samples/one owner and native motor bounds, not legacy PID caps."""
    grouped = {}
    assert records['UARX'] and records['UAMT']
    for row in records['UARX']:
        assert row['Own'] == 3 and row['Flt'] == 0
        assert all(math.isfinite(row[k]) for k in ('Tar', 'Gyr', 'P', 'I', 'D', 'Out'))
        grouped.setdefault((row['TimeUS'], row['Seq']), []).append(row)
        if disarmed:
            assert row['Arm'] == 0
    assert all(len(rows) == 3 and {r['Ax'] for r in rows} == {0, 1, 2} for rows in grouped.values())
    for row in records['UAMT']:
        assert row['APN'] == 0 and row['UN'] == row['Seq'] and row['MN'] == row['Seq']
        assert all(0 <= row[f'C{i}'] <= (1000 if disarmed else 2000) for i in range(1, 5))
    return dict(complete_sets=len(grouped), disarmed=disarmed, reported_ap_calls=0,
                peak_gyro_dps=[max(abs(math.degrees(r['Gyr'])) for r in records['UARX'] if r['Ax'] == axis)
                               for axis in range(3)],
                peak_angle_deg=[max(abs(math.degrees(r[k])) for r in records['UAAT']) for k in ('Roll', 'Pitch')],
                modes=[dict(mode=r['ModeNum'], reason=r['Rsn']) for r in records['MODE']],
                drops=max((r['Dp'] for r in records['DSF']), default=None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--stage', choices=('gate', 'disabled', 'flight', 'rotation', 'transitions',
                                            'rc', 'gcs', 'ekf', 'angle-limits', 'throttle', 'ground-arm'), required=True)
    parser.add_argument('--mode', type=int, choices=(42, 43), default=43)
    parser.add_argument('--axis', type=int, choices=(0, 1), default=0)
    parser.add_argument('--sign', type=int, choices=(-1, 1), default=1)
    args = parser.parse_args()
    if args.stage not in ('gate', 'disabled'):
        pilot.require_gate(args.binary.resolve(), args.output.resolve())
    actions = dict(gate=gate, disabled=disabled, ekf=ekf_reset,
                   flight=partial(pilot_flight, mode=args.mode, axis=args.axis, sign=args.sign),
                   rotation=partial(rotation, axis=args.axis, sign=args.sign), transitions=transitions,
                   rc=partial(failsafe, mode=args.mode, source='rc'), gcs=partial(failsafe, mode=args.mode, source='gcs'))
    actions['angle-limits'] = angle_limits
    actions['throttle'] = manual_throttle
    actions['ground-arm'] = ground_arming
    extra = PARAMS + ('UTB_ENABLE 0\n' if args.stage == 'disabled' else '')
    records = pilot.launch(args.binary, args.output, args.name, actions[args.stage], extra=extra, speedup=2)
    if args.stage == 'disabled':
        assert not records['UARX'] and not records['UAMT']
        assert records['RCOU'] and all(max(r[f'C{i}'] for i in range(1, 5)) <= 1000 for r in records['RCOU'])
        result = dict(disabled_at_boot=True, disarmed=True, no_pilot_output=True)
    else:
        result = analyse(records, args.stage in ('gate', 'ekf'))
    if args.stage == 'throttle':
        groups = {}
        motors = {(r['TimeUS'], r['Seq']): r for r in records['UAMT']}
        rc_times = [r['TimeUS'] for r in records['RCIN']]
        assert rc_times
        for mode in (42, 43):
            groups_by_pwm = {1400: [], 1600: []}
            for row in records['UARC']:
                if row['Md'] != mode:
                    continue
                index = bisect.bisect_right(rc_times, row['TimeUS']) - 1
                if index >= 0:
                    pwm = records['RCIN'][index]['C3']
                    if pwm in groups_by_pwm:
                        groups_by_pwm[pwm].append(row)
            low, high = groups_by_pwm.values()
            assert len(low) > 100 and len(high) > 100
            assert sum(r['Out'] for r in high) / len(high) > sum(r['Out'] for r in low) / len(low)
            values = []
            for rows in (low, high):
                values.append(sum(sum(motors[(r['TimeUS'], r['Seq'])][f'M{i}'] for i in range(1, 5)) / 4
                                  for r in rows) / len(rows))
            assert values[1] > values[0], 'Actual motor collective failed to follow manual throttle'
            groups[mode] = dict(low_collective=values[0], high_collective=values[1])
        result['manual_throttle'] = groups
    result['binary_sha256'] = hashlib.sha256(args.binary.read_bytes()).hexdigest()
    (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
