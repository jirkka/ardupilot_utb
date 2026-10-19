#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# ----------------------------------------------------------------------------------------------------
# @file utb_pilot_rates_sitl.py
# @brief Short, owned SITL probes for Actual Rates; no serial or physical hardware API.
# @author Jiri Kovar
# @version 2026-10-19
# @date 2026-10-19
# GNU General Public License version 3 or later.
# ----------------------------------------------------------------------------------------------------
# Include Files
# Historical2I scenarios/profile; current3A pilot validation: utb_native_sitl.py.
import argparse
import hashlib
import json
import math
from functools import partial
from pathlib import Path

from pymavlink import mavutil

import utb_angle_sitl as angle
import utb_pilot_sitl as pilot
import utb_pilot_tuning as tuning
# End Include Files
# Public functions


# @brief Independently express Actual Rates in physical units for raw BIN verification.
# @param[in] stick Calibrated normalized stick.
# @param[in] center Center slope in deg/s.
# @param[in] maximum Endpoint in deg/s.
# @param[in] expo Fraction, not percent.
# @return Rate in rad/s.
# @details Global pure reference; no plant or controller model.
def actual(stick, center, maximum, expo):
    if not all(math.isfinite(v) for v in (stick, center, maximum, expo)):
        raise ValueError('Nonfinite rate input')
    if not (-1 <= stick <= 1 and 0 <= center <= maximum <= 670 and maximum >= 1 and 0 <= expo <= 1):
        raise ValueError('Invalid Actual domain')
    magnitude = abs(stick)
    rate = center * magnitude + (maximum - center) * ((1 - expo) * magnitude**2 + expo * magnitude**6)
    return math.radians(math.copysign(rate, stick))


# @brief Verify the expanded profile while physically simulated outputs remain DISARMED.
# @param[in] sim Owned local simulator.
# @return None.
# @details Global gate checks live-edit latching, opt-in anti-windup and direct selection.
def profile_gate(sim):
    sim.param('UTB_C_AW', 0)
    sim.mode(43, 0)
    sim.param('UTB_C_AW', 1)
    sim.select(43)
    for axis in ('roll', 'pitch', 'yaw'):
        for sign in (-1, 1):
            sim.sticks(**{axis: sign}, seconds=.3)
    sim.sticks(roll=.3, pitch=-.2, yaw=.1, seconds=.3)
    sim.select(42)
    sim.sticks(roll=1, pitch=1, seconds=.3)
    sim.sticks(seconds=.3)
    sim.select(43)
    sim.expected_mode = None
    sim.param('UTB_BF_R_E', .31)
    sim.duration(.3)
    sim.param('UTB_BF_R_E', .3)
    sim.duration(.3)
    assert sim.arm() != 0, 'Restoring a profile must not clear CONFIG latch'
    sim.select(0)
    sim.select(43)
    sim.sticks(seconds=.3)
    sim.select(0)


# @brief Apply a short RC pulse using fresh ATTITUDE timestamps instead of coarse SYSTEM_TIME.
# @param[in] sim Owned local simulator.
# @param[in] axis Body axis index.
# @param[in] stick Normalized input.
# @param[in] hold Duration in simulated seconds.
# @return None.
# @details Global scenario helper; actual reference durations are still measured from BIN.
def timed_pulse(sim, axis, stick, hold):
    if not math.isfinite(hold) or not 0 < hold <= 2:
        raise ValueError('Pulse duration outside bounded experiment')
    start = sim.wait('ATTITUDE').time_boot_ms
    setattr(sim, ('roll', 'pitch', 'yaw')[axis], angle.pwm(stick))
    sim.rc()
    sim.wait('ATTITUDE', lambda m: ((m.time_boot_ms - start) & 0xffffffff) >= hold * 1000)
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.rc()


# @brief Find a normalized input for a requested Actual rate without changing the frozen profile.
# @param[in] desired Positive requested rate in deg/s.
# @param[in] center Center sensitivity in deg/s.
# @param[in] maximum Endpoint in deg/s.
# @return Normalized input; PWM quantization is verified separately from logged RC.
# @details Global pure bisection, not PID or model tuning.
def inverse_actual(desired, center, maximum):
    if not math.isfinite(desired) or not 0 <= desired <= maximum:
        raise ValueError('Requested inverse rate is not achievable by the configured map')
    actual(0, center, maximum, .3)
    low, high = 0., 1.
    for _ in range(40):
        mid = (low + high) * .5
        if math.degrees(actual(mid, center, maximum, .3)) < desired:
            low = mid
        else:
            high = mid
    return high


# @brief Execute a signed rate ladder with explicit native GUIDED recovery between cases.
# @param[in] sim Owned local simulator.
# @return None.
# @details Only expected firmware attitude/vertical stops are recorded as unsupported maneuvers.
def rate_ladder(sim):
    sim.pilot_tilt_limit_deg = 75
    sim.pilot_gyro_limit_rads = math.radians(800) * math.sqrt(3)
    angle.prepare_flight(sim)
    tuning.recover(sim)
    for maximum in (90, 180, 360, 670):
        for axis in (0, 1):
            for sign in (1, -1):
                sim.select(43, armed=True, settle=False)
                sim.sticks(seconds=.3)
                sim.marks.append(dict(kind='rate_case', desired_dps=maximum, axis=axis, sign=sign))
                try:
                    timed_pulse(sim, axis, sign * inverse_actual(maximum, 160, 670), .12)
                except AssertionError as error:
                    if not any(str(error).startswith(f'UTB ACRO stop={code} ') for code in (4, 6)):
                        raise
                    sim.marks.append(dict(kind='expected_guard', reason=str(error)))
                sim.roll = sim.pitch = sim.yaw = 1500
                sim.rc()
                tuning.recover(sim)
    angle.finish_flight(sim)


# @brief Probe signed ANGLE references and center-stick self-level with fixed gains.
# @param[in] sim Owned local simulator.
# @return None.
# @details Guard-limited cases remain explicitly unsupported, followed by native recovery.
def angle_ladder(sim):
    sim.pilot_tilt_limit_deg = 75
    sim.pilot_gyro_limit_rads = math.radians(800) * math.sqrt(3)
    angle.prepare_flight(sim)
    tuning.recover(sim)
    for degrees in (10, 20, 30, 45, 60):
        for axis in (0, 1):
            for sign in (1, -1):
                sim.select(42, armed=True, settle=False)
                sim.sticks(seconds=.3)
                sim.marks.append(dict(kind='angle_case', desired_deg=degrees, axis=axis, sign=sign))
                try:
                    timed_pulse(sim, axis, sign * degrees / 60, .6)
                    timed_pulse(sim, axis, 0, 1.2)
                except AssertionError as error:
                    if not any(str(error).startswith(f'UTB ANGLE stop={code} ') for code in (4, 6)):
                        raise
                    sim.marks.append(dict(kind='expected_guard', reason=str(error)))
                sim.roll = sim.pitch = sim.yaw = 1500
                sim.rc()
                tuning.recover(sim)
    angle.finish_flight(sim)


# @brief Invert the native AP cubic throttle map for this calibrated SITL fixture.
# @param[in] collective Normalized thrust demand in 0..1.
# @param[in] hover Current AP hover parameter in 0..1.
# @return Integer PWM for the fixture's 1000/1500/2000 throttle calibration.
# @details ArduCopter/mode.cpp get_pilot_desired_throttle; not a vehicle controller.
def manual_throttle_pwm(collective, hover):
    if not math.isfinite(collective) or not 0 <= collective <= 1 or not math.isfinite(hover) or not 0 < hover < 1:
        raise ValueError('Invalid manual throttle map input')
    expo = max(-.5, min(1, -(hover - .5) / .375))
    low, high = 0., 1.
    for _ in range(30):
        middle = (low + high) * .5
        if middle * (1 - expo) + expo * middle**3 < collective:
            low = middle
        else:
            high = middle
    return round(1000 + 1000 * (low + high) * .5)


# @brief Supply explicit simulated pilot throttle compensation during a slow ANGLE maneuver.
# @param[in] sim Owned local simulator.
# @param[in] target Maximum requested angle in degrees.
# @param[in] axis Roll or pitch.
# @param[in] sign Direction.
# @return None.
# @details Test-input generator only: no height/position feedback, firmware angle boost or UTB altitude controller.
def angle_throttle(sim, target, axis, sign):
    sim.pilot_tilt_limit_deg = 75
    sim.pilot_gyro_limit_rads = math.radians(800) * math.sqrt(3)
    assert sim.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20000) == 0
    angle.prepare_flight(sim)
    tuning.recover(sim)
    assert sim.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET, 20000) == 0
    # Freeze the native GUIDED hover output before entering manual control.
    # ATTITUDE_TARGET.thrust is attitude_control->get_throttle_in(), not the hover estimate.
    samples = []
    first = sim.wait('ATTITUDE_TARGET')
    samples.append(first.thrust)
    while True:
        sample = sim.wait('ATTITUDE_TARGET')
        samples.append(sample.thrust)
        if ((sample.time_boot_ms - first.time_boot_ms) & 0xffffffff) >= 1000:
            break
    base = sum(samples) / len(samples)
    assert 0 < base < .6
    hover = tuning.read_parameter(sim, 'MOT_THST_HOVER')
    sim.throttle = manual_throttle_pwm(base, hover)
    sim.rc()
    sim.marks.append(dict(kind='frozen_native_hover', thrust=base, hover=hover,
                          samples=len(samples), throttle=sim.throttle))
    sim.select(42, armed=True, settle=False)
    sim.sticks(seconds=.3)
    start = sim.wait('ATTITUDE').time_boot_ms
    ramp = target / 15
    while True:
        msg = sim.wait('ATTITUDE')
        elapsed = ((msg.time_boot_ms - start) & 0xffffffff) * .001
        if elapsed >= 2 * ramp + 3:
            break
        requested = min(target, 15 * elapsed, max(0, 15 * (2 * ramp + 1.5 - elapsed)))
        setattr(sim, ('roll', 'pitch')[axis], angle.pwm(sign * requested / target))
        cosine = math.cos(msg.roll) * math.cos(msg.pitch)
        assert cosine > .4, 'Unexpected tilt in scripted throttle experiment'
        collective = base / cosine
        assert 0 < collective < .95, 'Scripted throttle exceeds bounded manual range'
        sim.throttle = manual_throttle_pwm(collective, hover)
        sim.rc()
    sim.roll = sim.pitch = sim.yaw = 1500
    sim.throttle = 1472
    sim.rc()
    tuning.recover(sim)
    angle.finish_flight(sim)


# @brief Probe one signed short pulse, then explicitly recover through native GUIDED.
# @param[in] sim Owned local simulator.
# @param[in] mode Pilot mode 42 or 43.
# @param[in] axis Body axis index.
# @param[in] sign Stick direction.
# @param[in] hold Requested duration; actual timestamps are retained.
# @return None.
# @details Global scenario. Standard vertical/health/envelope guards remain enforced.
def probe(sim, mode, axis, sign, hold):
    sim.pilot_tilt_limit_deg = 75
    sim.pilot_gyro_limit_rads = math.radians(800) * math.sqrt(3)
    angle.prepare_flight(sim)
    tuning.recover(sim)
    sim.select(mode, armed=True, settle=False)
    sim.sticks(seconds=.3, label='center_before')
    timed_pulse(sim, axis, sign, hold)
    sim.roll = sim.pitch = sim.yaw = 1500
    tuning.recover(sim)
    angle.finish_flight(sim)


# @brief Exercise nonzero direct switching with bounded partial Actual sticks.
# @param[in] sim Owned local simulator.
# @return None.
# @details Global scenario retains manual throttle and ends in explicit native recovery.
def switching(sim):
    sim.pilot_tilt_limit_deg = 75
    sim.pilot_gyro_limit_rads = math.radians(800) * math.sqrt(3)
    angle.prepare_flight(sim)
    tuning.recover(sim)
    sim.select(43, armed=True, settle=False)
    sim.sticks(roll=.1, pitch=-.08, yaw=.05, seconds=.2, label='actual_combined')
    sim.select(42, armed=True, settle=False)
    sim.duration(.2)
    sim.select(43, armed=True, settle=False)
    sim.duration(.2)
    sim.select(42, armed=True, settle=False)
    sim.sticks(seconds=.8, label='angle_self_level')
    tuning.recover(sim)
    angle.finish_flight(sim)


# @brief Verify references from real RC snapshots and summarize complete recorded sets.
# @param[in] records Raw BIN packets.
# @param[in] maximum Per-axis configured maximum in deg/s.
# @param[in] center Per-axis center slope in deg/s.
# @return Measurements; stop codes remain explicit and never become a plant PASS.
# @details Global analysis. Unknown logger drops stay null; incomplete axis sets reject evidence.
def analyse(records, maximum, center):
    maxima = (maximum,) * 3 if isinstance(maximum, (int, float)) else maximum
    centers = (center,) * 3 if isinstance(center, (int, float)) else center
    snapshots = {(r['TimeUS'], r['Seq']): r for r in records['UARC']}
    samples = []
    active = []
    grouped = {}
    for row in records['UARX']:
        key = row['TimeUS'], row['Seq']
        grouped.setdefault(key, []).append(row)
        rc = snapshots[key]
        if rc['Md'] == 43 and rc['V']:
            index = row['Ax']
            expected = actual(rc[('NR', 'NP', 'NY')[index]], centers[index], maxima[index], .3)
            assert abs(expected - row['Tar']) < 4e-6, 'Actual map / logged RC mismatch'
        if rc['V']:
            active.append(row)
        if rc['V'] and abs(row['Tar']) > 1e-5:
            samples.append(row)
    assert samples, 'Missing actual nonzero pilot reference'
    for row in records['UAMT']:
        rc = snapshots[(row['TimeUS'], row['Seq'])]
        if rc['V'] and rc['Stop'] == 0:
            assert row['APN'] == 0 and row['UN'] == row['Seq'] and row['MN'] == row['Seq'], 'Controller ownership'
    assert all(abs(r['I']) <= .190001 and math.isfinite(r['D']) for r in records['UARX'])
    assert all(len(rows) == 3 and {r['Ax'] for r in rows} == {0, 1, 2} for rows in grouped.values())
    return dict(
        reference_map_verified=True,
        maximum_target_dps=[max((abs(math.degrees(r['Tar'])) for r in samples if r['Ax'] == a), default=0)
                            for a in range(3)],
        maximum_gyro_dps=[max((abs(math.degrees(r['Gyr'])) for r in active if r['Ax'] == a), default=0)
                          for a in range(3)],
        maximum_angle_deg=[max((abs(math.degrees(r[k])) for r in records['UAAT']), default=0)
                           for k in ('Roll', 'Pitch')],
        stop_codes=sorted({r['Stop'] for r in records['UARC']}),
        rate_fault_codes=sorted({r['Flt'] for r in records['UAMT']}),
        allocation_masks=sorted({r['Lim'] for r in records['UAFB']}) if records['UAFB'] else [],
        logger_drops_max=max((r['Dp'] for r in records['DSF']), default=None),
        integration_blocked_masks=sorted({r['Blk'] for r in records['UAFB']}),
        maximum_i=max(abs(r['I']) for r in records['UARX']),
        maximum_d=max(abs(r['D']) for r in records['UARX']),
        maximum_raw=max((abs(r['Raw']) for r in records['UAFB']), default=None),
        maximum_vertical_speed=max((abs(r['VZ']) for r in records['UAST']), default=None),
        complete_axis_sets=len(grouped))


# @brief Run a single isolated experimental profile and retain all unsuccessful evidence.
# @param[in] None
# @return None; incomplete logging or setup failure produces nonzero exit.
# @details Global CLI. Requires a gate from this executable's SHA and never weakens guards.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--name', required=True)
    parser.add_argument('--stage', choices=('gate', 'profile-gate', 'rate', 'rate-ladder', 'angle', 'angle-ladder',
                                            'angle-throttle',
                                            'switches', 'rc-loss', 'gcs-loss'),
                        required=True)
    parser.add_argument('--maximum', type=float, default=90)
    parser.add_argument('--yaw-maximum', type=float, help='Independent yaw endpoint; default matches RP')
    parser.add_argument('--angle', type=float, default=45)
    parser.add_argument('--axis', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--hold', type=float, default=.12)
    args = parser.parse_args()
    args.binary = args.binary.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.stage == 'gate':
        records = pilot.launch(args.binary, args.output, 'disarmed', pilot.disarmed, speedup=2)
        result = pilot.analyse_disarmed(records)
        result['binary_sha256'] = hashlib.sha256(args.binary.read_bytes()).hexdigest()
        (args.output / 'disarmed/analysis.json').write_text(json.dumps(result, indent=2))
        return
    pilot.require_gate(args.binary, args.output)
    maxima = [args.maximum, args.maximum, args.maximum if args.yaw_maximum is None else args.yaw_maximum]
    centers = [min(160, value) for value in maxima]
    params = 'UTB_BF_TYPE 1\nUTB_BF_ANGLE %g\n' % args.angle
    for index, axis in enumerate(('R', 'P', 'Y')):
        params += (f'UTB_BF_{axis}_C {centers[index]}\nUTB_BF_{axis}_M {maxima[index]}\n'
                   f'UTB_BF_{axis}_E 0.3\n')
    if args.stage == 'profile-gate':
        records = pilot.launch(args.binary, args.output, args.name, profile_gate, extra=params, speedup=2)
        result = analyse(records, maxima, centers)
        assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
        assert any(r['Stop'] == 1 for r in records['UARC'])
        (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        return
    if args.stage in ('switches', 'rc-loss', 'gcs-loss'):
        action = switching if args.stage == 'switches' else partial(
            pilot.failsafe, source='rc' if args.stage == 'rc-loss' else 'gcs', mode=43)
        records = pilot.launch(args.binary, args.output, args.name, action, extra=params, speedup=2)
        if args.stage == 'switches':
            result = analyse(records, maxima, centers)
            assert result['stop_codes'] == [0]
        else:
            assert any(r['ModeNum'] == 6 for r in records['MODE'])
            result = dict(native_rtl=True, source=args.stage)
        (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        return
    if args.stage in ('rate-ladder', 'angle-ladder'):
        assert maxima[:2] == [670, 670] and centers[:2] == [160, 160]
        if args.stage == 'angle-ladder':
            assert args.angle == 60
        action = rate_ladder if args.stage == 'rate-ladder' else angle_ladder
        records = pilot.launch(args.binary, args.output, args.name, action, extra=params, speedup=2)
        result = analyse(records, maxima, centers)
        (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        return
    if args.stage == 'angle-throttle':
        assert args.axis in (0, 1) and args.angle in (45, 60)
        records = pilot.launch(args.binary, args.output, args.name,
                               partial(angle_throttle, target=args.angle, axis=args.axis, sign=args.sign),
                               extra=params, speedup=2)
        result = analyse(records, maxima, centers)
        assert result['stop_codes'] == [0]
        (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        return
    mode = 43 if args.stage == 'rate' else 42
    records = pilot.launch(args.binary, args.output, args.name,
                           partial(probe, mode=mode, axis=args.axis, sign=args.sign, hold=args.hold),
                           extra=params, speedup=2)
    result = analyse(records, maxima, centers)
    result.update(request=vars(args) | dict(binary=str(args.binary), output=str(args.output)))
    (args.output / args.name / 'analysis.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)


# End Public functions
# Private functions
# None
# End Private functions
if __name__ == '__main__':
    main()
