#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Isolated actual ArduPilot SITL experiments. Never accepts a device or remote endpoint."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

from pymavlink import mavutil
from test_utb_shadow import ShadowSITL
from utb_active_sitl_analysis import alignment, check_rows, metrics as response_metrics

BASE = ('FRAME_CLASS 1\nFRAME_TYPE 18\nSCHED_LOOP_RATE 400\nFSTRATE_ENABLE 0\n'
        'UTB_SHADOW 0\nUTB_S_OWNER 0\nUTB_S_PROFILE 0\nUTB_S_AMP 0.15\nUTB_S_RUN 0\n'
        'LOG_DISARMED 1\nLOG_BITMASK 131071\nFS_THR_ENABLE 1\nFS_GCS_ENABLE 1\nARMING_CHECK 1\n'
        'SIM_GYR1_RND 0\nSIM_GYR2_RND 0\nSIM_ACC1_RND 0\nSIM_ACC2_RND 0\nSIM_MAG_RND 0\n'
        'SIM_WIND_SPD 0\nACRO_TRAINER 0\n')
for _axis in ['RLL', 'PIT', 'YAW']:
    # SITL ONLY: one bounded half-scale adjustment after the archived nominal zero-reference failure.
    for _suffix, _value in [('P', .155), ('I', .115), ('D', .002), ('IMAX', .19), ('D_HZ', 20)]:
        BASE += f'UTB_RAT_{_axis}_{_suffix} {_value}\n'


class ActiveSITL(ShadowSITL):
    def __init__(self, *args, **kwargs):
        self.events, self.guard, self.heartbeat_wall = [], False, -1.
        self.last_boot_ms = 0
        super().__init__(*args, **kwargs)

    def wait(self, kind, predicate=lambda msg: True, timeout=30):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if time.monotonic()-self.heartbeat_wall >= .1:
                self.link.mav.heartbeat_send(6, 8, 0, 0, 0)
                self.heartbeat_wall = time.monotonic()
            self.rc()
            msg = self.link.recv_match(blocking=True, timeout=.05)
            if msg is None:
                if self.process.poll() is not None:
                    raise RuntimeError('Owned SITL process exited')
                continue
            typ = msg.get_type()
            if typ == 'SYSTEM_TIME':
                self.last_boot_ms = msg.time_boot_ms
            if typ == 'STATUSTEXT':
                self.events.append(dict(host=time.time(), boot_ms=self.last_boot_ms, text=msg.text))
                if self.guard and msg.text.startswith('UTB SITL fault'):
                    raise AssertionError(msg.text)
            if self.guard and typ == 'HEARTBEAT' and msg.custom_mode != 41:
                raise AssertionError('Experiment mode exited; no hidden AP fallback accepted')
            if self.guard and typ == 'ATTITUDE':
                if max(abs(msg.roll), abs(msg.pitch)) > math.radians(35):
                    raise AssertionError('SITL experiment tilt envelope exceeded')
            if typ == kind and predicate(msg):
                return msg
        raise AssertionError('Timeout waiting for '+kind)

    def command(self, command, *params):
        params = list(params)+[0.]*(7-len(params))
        self.link.mav.command_long_send(1, 1, command, 0, *params)
        return self.wait('COMMAND_ACK', lambda m: m.command == command).result

    def ready(self):
        self.wait('SYSTEM_TIME', lambda m: m.time_boot_ms >= 30000, timeout=60)
        self.wait('EKF_STATUS_REPORT', lambda m: m.flags & 1 and not m.flags & 1024, timeout=30)

    def standard_arm(self):
        # Positive experiments never use force-arm. Preserve ordinary prearm checks.
        result = self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise AssertionError('Standard arming rejected; inspect saved prearm messages, no force retry')
        self.wait('HEARTBEAT', lambda m: bool(m.base_mode & 128))


def read_packets(directory):
    records = {'UARX': [], 'UAMT': [], 'UAFB': [], 'UALT': [], 'UAAT': [], 'UAST': [], 'PM': [], 'PARM': []}
    for path in sorted((directory/'logs').glob('*.BIN')):
        reader = mavutil.mavlink_connection(str(path))
        while (msg := reader.recv_match()) is not None:
            if msg.get_type() in records:
                records[msg.get_type()].append(msg.to_dict())
        reader.close()
    return records


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # Preserve diagnostic nonfinite markers; never silently replace control data by zero.
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return value


def launch(binary, output, name, action, extra='', model='bfxrev', speedup=5):
    directory = output/name
    directory.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    sim = None
    error = None
    with (directory/'console.log').open('x') as console:
        try:
            sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', directory,
                             console, True, 1, extra_params=BASE+extra, model=model, speedup=speedup)
            sim.link.logfile = (directory/'receive.tlog').open('xb')
            sim.ready()
            action(sim)
            sim.duration(5)
        except Exception as failure:
            error = repr(failure)
        finally:
            if sim:
                (directory/'events.json').write_text(json.dumps(sim.events, indent=2)+'\n')
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
    records = read_packets(directory)
    (directory/'packets.json').write_text(json.dumps(json_safe(records), allow_nan=False)+'\n')
    if error:
        (directory/'failure.json').write_text(json.dumps(dict(error=error), indent=2)+'\n')
        raise AssertionError(name+': '+error)
    return records


def basic(sim):
    sim.mode(41, 0)  # default owner disabled, including skipped ordinary disarmed checks
    sim.param('UTB_S_OWNER', 1)
    sim.mode(41, 41)
    sim.duration(2)
    sim.mode(0, 0)
    for mode in [1, 2, 0]:
        sim.mode(mode, mode)
    sim.mode(40, 40)
    for force in [False, True]:
        assert sim.arm(force=force) != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.mode(0, 0)


def analyse_basic(records):
    axes, motors = records['UARX'], records['UAMT']
    assert len(axes) > 100 and len(motors) > 100, 'No actual experimental runtime data'
    assert all(r['Arm'] == 0 and r['Own'] == 1 and r['Flt'] == 0 for r in axes)
    assert all(r['Out'] == 0 for r in axes)
    assert all(r['APN'] == 0 and r['MN'] == r['Seq'] for r in motors)
    assert motors[-1]['UN'] > 100
    assert all(max(r[f'C{m}'] for m in range(1, 5)) <= 1000 for r in motors)
    assert all(.000625 <= r['Dt'] <= .01 and math.isfinite(r['Gyr']) for r in axes)
    return dict(pass_basic=True, axis_packets=len(axes), motor_packets=len(motors),
                ap_calls=motors[-1]['APN'], utb_calls=motors[-1]['UN'], virtual_arming=False)


def guards(sim):
    sim.param('UTB_S_OWNER', 1)
    # An RC mode request cannot activate the experimental authority.
    sim.param('FLTMODE6', 41)
    sim.rc5 = 2000
    sim.observe(0, False)
    sim.rc5 = 1000
    sim.duration(1)
    cases = [('FRAME_TYPE', 12, 18, 3), ('FRAME_CLASS', 2, 1, 3), ('FSTRATE_ENABLE', 1, 0, 4),
             ('UTB_S_OWNER', 2, 1, 5), ('UTB_S_PROFILE', 1, 0, 5), ('UTB_RAT_RLL_I', .12, .115, 5),
             ('UTB_SHADOW', 1, 0, 5), ('UTB_S_DLY', 1, 0, 5), ('UTB_S_AW', 1, 0, 5),
             ('UTB_S_AMP', .16, .15, 5)]
    for name, wrong, correct, fault in cases:
        sim.mode(41, 41)
        start = len(sim.events)
        sim.param(name, wrong)
        sim.duration(1)
        assert any(e['text'].startswith(f'UTB SITL fault={fault} ') for e in sim.events[start:]), name
        assert sim.arm(force=False) != mavutil.mavlink.MAV_RESULT_ACCEPTED
        sim.param(name, correct)
        sim.duration(1)  # restoring the parameter does not clear the latched fault
        sim.observe(41, False)
        sim.mode(0, 0)
    sim.mode(41, 41)
    sim.duration(2)
    sim.mode(0, 0)


def analyse_guards(records):
    assert {3, 4, 5} <= {r['Flt'] for r in records['UAMT']}
    assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
    assert all(r['APN'] == 0 and r['MN'] == r['Seq'] for r in records['UAMT'])
    assert all(max(r[f'C{m}'] for m in range(1, 5)) <= 1000 for r in records['UAMT'])
    entries, last_seq, latched, stopped_calls = 0, 0, 0, None
    for m in records['UAMT']:
        if m['Seq'] < last_seq:
            latched, stopped_calls = 0, None
            entries += 1
        if m['Flt']:
            latched = latched or m['Flt']
            stopped_calls = m['UN'] if stopped_calls is None else stopped_calls
            assert m['Flt'] == latched and m['UN'] == stopped_calls
        elif latched:
            raise AssertionError('Fault cleared without mode exit/entry')
        last_seq = m['Seq']
    assert entries >= 7 and records['UAMT'][-1]['Flt'] == 0, 'No recovery through explicit re-entry'
    return dict(disarmed=True, runtime_faults=[3, 4, 5], latched_stop=True,
                reentry_resets=True, competing_ap_calls=0, rc_activation_rejected=True)


def absence(binary, output, speedup):
    directory = output/'compiled-out'
    directory.mkdir()
    root = Path(__file__).resolve().parents[2]
    with (directory/'console.log').open('x') as console:
        sim = ActiveSITL(binary, root/'Tools/autotest/default_params/copter.parm', directory, console, False, 0,
                         extra_params='LOG_DISARMED 1\nARMING_CHECK 1\n', speedup=speedup)
        try:
            sim.mode(41, 0)
            for name in ['UTB_S_OWNER', 'UTB_S_PROFILE', 'UTB_S_AMP', 'UTB_S_RUN', 'UTB_S_DLY', 'UTB_S_AW',
                         'UTB_S_ATT', 'UTB_S_ANG', 'UTB_S_ARP', 'UTB_S_APP', 'UTB_S_RMAX',
                         'UTB_S_YRATE', 'UTB_S_TOFF']:
                sim.link.mav.param_request_read_send(1, 1, name.encode(), -1)
                try:
                    sim.wait('PARAM_VALUE', lambda m: m.param_id == name, timeout=2)
                except AssertionError:
                    pass
                else:
                    raise AssertionError('SITL authority parameter exists in compiled-out build: '+name)
            for mode in [1, 2, 0]:
                sim.mode(mode, mode)
        finally:
            sim.close()
    result = dict(mode_41_absent=True, authority_parameters_absent=True, original_manual_modes_available=True)
    (output/'absence.json').write_text(json.dumps(result, indent=2)+'\n')
    print('Compiled-out authority PASS', flush=True)


def flight(sim, duration=8, expected_fault=None):
    sim.wait('EKF_STATUS_REPORT', lambda m: m.flags & (16 | 512) and not m.flags & 1024, timeout=45)
    sim.mode(4, 4)
    sim.standard_arm()
    sim.throttle = 1500
    assert sim.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10) == 0
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt >= 9700, timeout=45)
    sim.duration(4)
    sim.wait('ATTITUDE', lambda m: max(abs(m.rollspeed), abs(m.pitchspeed), abs(m.yawspeed)) < .03
             and max(abs(m.roll), abs(m.pitch)) < math.radians(3), timeout=15)
    # Preserve the original AP tune for preparation; only remove target/error filtering and FF in the comparison.
    for axis in ['RLL', 'PIT', 'YAW']:
        for suffix in ['FLTT', 'FLTE', 'FF']:
            sim.param('ATC_RAT_'+axis+'_'+suffix, 0)
    sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 41)
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 41)
    sim.guard = expected_fault is None
    sim.duration(2)
    sim.param('UTB_S_RUN', 1)
    if expected_fault is not None:
        sim.wait('STATUSTEXT', lambda m: m.text.startswith(f'UTB SITL fault={expected_fault} '), timeout=45)
        sim.duration(3)  # observe the latched stop and allow the existing logger to drain
    else:
        sim.duration(duration+1)
    sim.guard = False
    # Explicit cleanup after the completed measurement; no AP fallback inside its window.
    sim.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9)
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 9)
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt < 200, timeout=40)
    sim.duration(3)
    assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)


def assemble(records):
    axes = {}
    for row in records['UARX']:
        key = (row['TimeUS'], row['Seq'])
        axis = axes.setdefault(key, {})
        assert row['Ax'] not in axis, 'Duplicate axis packet'
        axis[row['Ax']] = row
    complete = []
    for m in records['UAMT']:
        a = axes.get((m['TimeUS'], m['Seq']), {})
        if len(a) == 3:
            complete.append((m, a))
    return complete


def analyse_flight(records, owner, directory, expected_samples=3200, require_response=True):
    all_sets = assemble(records)
    samples = [(m, a) for m, a in all_sets if m['Ph'] == 1]
    final = next(((m, a) for m, a in all_sets if m['Ph'] == 2), None)
    if final is not None:
        samples.append(final)  # last generator sample is marked complete by the preceding mode update
    assert len(samples) >= expected_samples-5, ('Missing experiment samples', len(samples))
    assert all(m['Flt'] == 0 for m, _ in all_sets), 'Saturation/invalidity: stop, not a supported flight'
    assert any(m['Ph'] == 2 for m, _ in all_sets), 'No finite sequence completion'
    first = samples[0][0]['TimeUS']
    rows = []
    for m, axes in samples:
        assert m['MN'] == m['Seq'], 'Multiple or missing motor callbacks'
        assert (m['APN'] == 0 if owner == 1 else m['UN'] == 0), 'Concurrent rate controllers'
        row = dict(t=(m['TimeUS']-first)*1e-6, dt=axes[0]['Dt'], seq=m['Seq'],
                   execution_us=m['Ex'], ap_calls=m['APN'], utb_calls=m['UN'], motor_calls=m['MN'])
        for a, data in axes.items():
            assert data['Own'] == owner and data['Arm'] == 1 and data['Spl'] == 3
            for field, label in [('Ref', 'ref'), ('Tar', 'target'), ('Gyr', 'gyro'),
                                 ('P', 'p'), ('I', 'i'), ('D', 'd'), ('Out', 'out')]:
                row[label+str(a)] = data[field]
        for a in range(4):
            row['pwm'+str(a)] = m['C'+str(a+1)]
            row['actuator'+str(a)] = m['M'+str(a+1)]
        assert all(math.isfinite(v) for v in row.values())
        rows.append(row)
    with (directory/'response.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    check_rows(rows, owner, expected_samples)
    measurement = response_metrics(rows, directory.name.removesuffix('-utb').removesuffix('-ap'))
    for a in range(3):
        positive = [r for r in rows if r['ref'+str(a)] > .05]
        if positive and require_response:
            amp = max(r['ref'+str(a)] for r in positive)
            assert max(r['gyro'+str(a)] for r in positive) > .5*amp, ('No positive response', a)
        negative = [r for r in rows if r['ref'+str(a)] < -.05]
        if negative and require_response:
            assert min(r['gyro'+str(a)] for r in negative) < .5*min(r['ref'+str(a)] for r in negative)
    values = {p['Name']: p['Value'] for p in records['PARM']}
    prefix = 'UTB_RAT_' if owner == 1 else 'ATC_RAT_'
    gains = {axis: {suffix: values[prefix+axis+'_'+suffix] for suffix in
                    (['P', 'I', 'D', 'IMAX', 'D_HZ'] if owner == 1 else ['P', 'I', 'D', 'IMAX', 'FLTD', 'FLTT', 'FLTE', 'FF'])}
             for axis in ['RLL', 'PIT', 'YAW']}
    result = dict(owner=owner, samples=len(rows), observed_hz=(len(rows)-1)/(rows[-1]['t']-rows[0]['t']),
                  axis=measurement['axis'], timing=measurement['timing'],
                  max_host_execution_us=max(r['execution_us'] for r in rows),
                  reference_definition='IAE/ITAE use actual shared shaped target; step metrics use raw constant plateau',
                  gains=gains, pm=records['PM'],
                  disclaimer='ACTUAL ARDUPILOT SITL – NOT HARDWARE VERIFIED')
    (directory/'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result, rows


def responses(binary, output, speedup, perturbations_only=False):
    profiles = [('zero', 0), ('roll', 1), ('pitch', 2), ('yaw', 3), ('combined', 4), ('sine', 5), ('sign', 6)]
    cases = [] if perturbations_only else [(name, profile, '', 'bfxrev') for name, profile in profiles]
    # Model inertia perturbation uses the real SIM_Frame loader, not the earlier Python plant.
    model = output/'inertia-model.json'
    inertia = 3.*.25*(.35*.5)**2
    model.write_text(json.dumps(dict(mass=3., diagonal_size=.35,
                                     moment_inertia=[inertia*1.1, inertia*1.1, inertia*2.2]))+'\n')
    # AP_Filesystem_Posix maps leading '/' under the SITL working directory.
    cases.append(('inertia', 1, '', 'bfxrev:../inertia-model.json'))
    cases.append(('delay', 1, 'UTB_S_DLY 1\n', 'bfxrev'))
    results = {}
    for name, profile, extra, vehicle_model in cases:
        pair = {}
        for owner in [1, 2]:
            case = name+('-utb' if owner == 1 else '-ap')
            records = launch(binary, output, case, flight, extra+f'UTB_S_OWNER {owner}\nUTB_S_PROFILE {profile}\n',
                             model=vehicle_model, speedup=speedup)
            result, rows = analyse_flight(records, owner, output/case)
            pair[owner] = rows
            results[case] = result
            (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
            print(case, 'PASS', result['samples'], 'samples', result['observed_hz'], 'Hz', flush=True)
        results[name+'-alignment'] = alignment(pair[1], pair[2])
    (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', choices=['basic', 'guards', 'absence', 'responses', 'perturbations'], required=True)
    parser.add_argument('--speedup', type=float, default=5)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.suite == 'absence':
        absence(args.binary.resolve(), args.output, args.speedup)
    elif args.suite == 'basic':
        records = launch(args.binary.resolve(), args.output, 'disarmed', basic, speedup=args.speedup)
        result = analyse_basic(records)
        (args.output/'basic.json').write_text(json.dumps(result, indent=2)+'\n')
        print('DISARMED integration PASS', result, flush=True)
    elif args.suite == 'guards':
        records = launch(args.binary.resolve(), args.output, 'guards', guards, speedup=args.speedup)
        result = analyse_guards(records)
        (args.output/'guards.json').write_text(json.dumps(result, indent=2)+'\n')
        print('DISARMED lifecycle guards PASS', result, flush=True)
    else:
        responses(args.binary.resolve(), args.output, args.speedup, args.suite == 'perturbations')


if __name__ == '__main__':
    main()
