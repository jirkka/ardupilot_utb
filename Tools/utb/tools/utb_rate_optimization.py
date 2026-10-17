#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Bounded gains experiments in owned ArduPilot SITL processes. No hardware transport."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_rate_optimization.py
# @brief      Actual SITL gains, allocation/AW stress, Pareto, capture a I recurrence.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import csv
import json
import math
from pathlib import Path

from test_utb_active_sitl import flight, launch, analyse_flight
from utb_active_sitl_analysis import alignment

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
BASELINE = [0.155, 0.115, 0.002, 0.19, 20.]
RANGES = {'P': [0.1, 0.3], 'I': [0.02, 0.6], 'D': [0., 0.002], 'IMAX': [0.05, 0.19], 'D_HZ': [10., 30.]}
# A finite list, not an adaptive search towards saturation.
CANDIDATES = {
    'roll': [BASELINE, [.19, .115, .0015, .19, 20], [.22, .16, .001, .19, 20],
             [.19, .16, .001, .19, 20], [.19, .16, .0015, .19, 15]],
    'pitch': [BASELINE, [.19, .115, .0015, .19, 20], [.22, .16, .001, .19, 20],
              [.19, .16, .001, .19, 20], [.19, .16, .0015, .19, 15]],
    'yaw': [BASELINE, [.155, .23, .001, .19, 20], [.23, .3, .001, .19, 20],
            [.25, .45, .001, .19, 20], [.3, .45, .001, .19, 20], [.3, .6, 0., .19, 20],
            [.25, .45, 0., .19, 20]],
}
SELECTED = [[.22, .16, .001, .19, 20], [.19, .16, .001, .19, 20], [.3, .6, 0., .19, 20]]
FINAL = [[.19, .16, .0015, .19, 15], list(SELECTED[1]), list(SELECTED[2])]


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Independent projection of actual post-output captures; no achieved physical torque claim.
#
# @param[in]  records Decoded diagnostic records grouped by message type.
# @param[in]  rows Exported sample records.
# @param[in]  directory Experiment input/output directory.
# @param[in]  gains Per-axis controller gains.
# @param[in]  feedback Feedback.
# @param[in]  delay Delay.
# @return     Returned values: result.
#
# @details    Global module function. Calls: enumerate, dict, (directory / 'allocation-metrics.json').write_text,
#             axes.setdefault.
# --------------------------------------------------------------------------------------------------------------------
def allocation_analysis(records, rows, directory, gains, feedback, delay):
    """Independent projection of actual post-output captures; no achieved physical torque claim."""
    factors = [[-.5, -.5, .5, .5], [-.5, .5, -.5, .5], [.5, -.5, -.5, .5]]
    axes, motors = {}, {}
    control_times = {item['Seq']: item['TimeUS'] for item in records.get('UAMT', [])}
    for item in records['UAFB']:
        group = axes.setdefault(item['Seq'], {})
        if item['Ax'] in group:
            raise ValueError('Duplicate allocation axis')
        group[item['Ax']] = item
    for item in records['UALT']:
        if item['Seq'] in motors:
            raise ValueError('Duplicate allocation capture')
        motors[item['Seq']] = item
    enriched, max_projection_error, max_integrator_error = [], 0., 0.
    for n, row in enumerate(rows):
        seq = row['seq']
        a, m = axes.get(seq, {}), motors.get(seq)
        if set(a) != {0, 1, 2} or m is None or not m['Av']:
            raise ValueError('Missing/invalid allocation capture')
        if m['TimeUS'] != control_times.get(seq) or any(item['TimeUS'] != m['TimeUS'] for item in a.values()):
            raise ValueError('Allocation/control timestamp attribution mismatch')
        out = dict(row)
        out['limits'] = m['Lim']
        for k in range(3):
            data = a[k]
            if not data['Av'] or not data['PV'] or data['CSeq'] != seq-delay:
                raise ValueError('Invalid PID attribution')
            if abs(data['Cmd']-row[f'out{k}']) > 1e-7:
                raise ValueError('Applied command differs from captured command')
            projection = sum(b*m[f'T{j+1}']/m['G'] for j, b in enumerate(factors[k]))
            max_projection_error = max(max_projection_error, abs(data['Alc']-projection))
            for field in ['Raw', 'Cmd', 'Alc', 'Res']:
                out[field.lower()+str(k)] = data[field]
            out['blocked'+str(k)] = int(bool(data['Blk'] & (1 << k)))
            if feedback and n:
                previous = axes[seq-1][k]
                delta = gains[k][1]*(row[f'target{k}']-row[f'gyro{k}'])*row['dt']
                residual = previous['Res']
                expected_block = (residual > 1e-5 and delta > 0) or (residual < -1e-5 and delta < 0)
                if abs(abs(residual)-1e-5) > 2e-6 and expected_block != bool(out['blocked'+str(k)]):
                    raise ValueError('Directional integration gate mismatch')
                expected_i = max(-gains[k][3], min(gains[k][3],
                                 rows[n-1][f'i{k}']+(0 if out['blocked'+str(k)] else delta)))
                max_integrator_error = max(max_integrator_error, abs(expected_i-row[f'i{k}']))
        enriched.append(out)
    if max_projection_error > 2e-6 or max_integrator_error > 2e-6:
        raise ValueError('Allocation/integrator parity mismatch')
    with (directory/'allocation-response.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(enriched[0]))
        writer.writeheader()
        writer.writerows(enriched)
    result = dict(complete_capture_sets=len(enriched), max_projection_error=max_projection_error,
                  max_integrator_error=max_integrator_error, physical_moment_claimed=False,
                  axes=[dict(blocked_cycles=sum(r[f'blocked{k}'] for r in enriched),
                             residual_cycles=sum(abs(r[f'res{k}']) > 1e-5 for r in enriched),
                             saturation_s=sum(r['dt'] for r in enriched if abs(r[f'res{k}']) > 1e-5),
                             maximum_raw=max(abs(r[f'raw{k}']) for r in enriched),
                             maximum_integrator=max(abs(r[f'i{k}']) for r in enriched)) for k in range(3)],
                  allocator_limit_cycles=sum(bool(r['limits'] & 7) for r in enriched))
    (directory/'allocation-metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


# --------------------------------------------------------------------------------------------------------------------
# @brief      Experiment.
#
# @param[in]  binary Explicit local SITL executable.
# @param[in]  output Explicit destination for new experiment results.
# @param[in]  case Case.
# @param[in]  gains Per-axis controller gains.
# @param[in]  profile Explicit finite reference profile identifier.
# @param[in]  owner Owner.
# @param[in]  feedback Feedback.
# @param[in]  delay Delay.
# @param[in]  model Model.
# @param[in]  extra Extra.
# @param[in]  speedup Speedup.
# @param[in]  require_response Require response.
# @return     Returned values: (result, rows).
#
# @details    Global module function. Calls: launch, analyse_flight, allocation_analysis, dict.
# --------------------------------------------------------------------------------------------------------------------
def experiment(binary, output, case, gains, profile, owner=1, feedback=1, delay=0, model='bfxrev', extra='',
               speedup=5, require_response=True):
    duration = 20 if profile == 7 else 8
    config = parameters(gains)+f'UTB_S_OWNER {owner}\nUTB_S_PROFILE {profile}\nUTB_S_AW {feedback}\nUTB_S_DLY {delay}\n'+extra
    records = launch(binary, output, case, lambda sim: flight(sim, duration), config, model=model, speedup=speedup)
    measured, rows = analyse_flight(records, owner, output/case, int(duration*400), require_response)
    actual_gains = [[measured['gains'][axis][name] for name in ['P', 'I', 'D', 'IMAX', 'D_HZ']]
                    for axis in ['RLL', 'PIT', 'YAW']] if owner == 1 else gains
    allocation = allocation_analysis(records, rows, output/case, actual_gains, feedback, delay)
    result = dict(status='PASS', case=case, profile=profile, owner=owner, feedback=feedback, delay=delay,
                  metrics=measured, allocation=allocation)
    if profile in [1, 2, 3, 7]:
        axis = 2 if profile == 7 else profile-1
        result['quality'] = quality(rows, axis, measured)
    (output/case/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(case, 'PASS', measured['samples'], 'sets', flush=True)
    return result, rows


# --------------------------------------------------------------------------------------------------------------------
# @brief      Validation.
#
# @param[in]  binary Explicit local SITL executable.
# @param[in]  output Explicit destination for new experiment results.
# @param[in]  suite Suite.
# @param[in]  speedup Speedup.
# @return     None.
#
# @details    Global module function. Calls: output.mkdir, (output / 'plan.json').write_text, list, launch.
# --------------------------------------------------------------------------------------------------------------------
def validation(binary, output, suite, speedup):
    output.mkdir(parents=True, exist_ok=False)
    results = {}
    baseline = [list(BASELINE) for _ in range(3)]
    if suite == 'aw-guards':
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # End Public functions
        # ===================================================================================================================
        # Private functions
        # -------------------------------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------------------------------
        # @brief      Guard action.
        #
        # @param[in]  sim Owned local simulator connection.
        # @return     None.
        #
        # @details    Local nested helper. Calls: sim.param, sim.mode, sim.duration, len.
        # -------------------------------------------------------------------------------------------------------------------
        def guard_action(sim):
            sim.param('UTB_S_OWNER', 1)
            sim.param('UTB_S_AW', 1)
            sim.param('SCR_ENABLE', 1)
            sim.mode(41, 0)
            sim.param('SCR_ENABLE', 0)
            sim.mode(41, 41)
            sim.duration(1)
            begin = len(sim.events)
            sim.param('SCR_ENABLE', 1)
            sim.duration(1)
            assert any(e['text'].startswith('UTB SITL fault=5 ') for e in sim.events[begin:])
            sim.param('SCR_ENABLE', 0)
            sim.duration(1)
            sim.observe(41, False)
            sim.mode(0, 0)
            sim.param('UTB_S_OWNER', 2)
            sim.mode(41, 0)  # AP baseline cannot select UTB feedback policy.
            sim.param('UTB_S_OWNER', 1)
            sim.mode(41, 41)
            sim.duration(1)
            sim.mode(0, 0)
        records = launch(binary, output, 'disarmed-aw-guards', guard_action, speedup=speedup)
        assert any(m['Flt'] == 5 for m in records['UAMT'])
        assert all(m['Arm'] == 0 and m['Out'] == 0 for m in records['UARX'])
        assert all(m['APN'] == 0 and m['MN'] == m['Seq'] for m in records['UAMT'])
        results = dict(status='PASS', scripting_entry_and_runtime_rejected=True, ap_owner_feedback_rejected=True,
                       disarmed_only=True, ap_calls=0)
        (output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
        print('AW-specific DISARMED guards PASS', flush=True)
        return
    if suite == 'preliminary':
        plan = [('zero', SELECTED, 0, 1, 1, 0, 'bfxrev', ''),
                ('roll', SELECTED, 1, 1, 1, 0, 'bfxrev', ''),
                ('yaw', SELECTED, 3, 1, 1, 0, 'bfxrev', '')]
    elif suite == 'repeat':
        plan = [(f'{axis}-{label}-r{repeat}', gains, profile, owner, feedback, 0, 'bfxrev', '')
                for repeat in range(1, 4) for profile, axis in enumerate(['roll', 'pitch', 'yaw'], 1)
                for label, gains, owner, feedback in [('old', baseline, 1, 0), ('new', SELECTED, 1, 1),
                                                      ('ap', baseline, 2, 0)]]
    elif suite == 'repeat-final':
        plan = [(f'{axis}-final-r{repeat}', FINAL, profile, 1, 1, 0, 'bfxrev', '')
                for repeat in range(1, 4) for profile, axis in enumerate(['roll', 'pitch', 'yaw'], 1)]
    elif suite == 'causal':
        i_only = [list(g) for g in baseline]
        i_only[2][1] = .23
        d_only = [list(g) for g in baseline]
        d_only[2][2] = 0
        rp_d = [list(g) for g in baseline]
        rp_d[0][2] = .001
        rp_p = [list(g) for g in baseline]
        rp_p[0][0] = .22
        narrow = [list(g) for g in SELECTED]
        for g in narrow:
            g[3] = .1
        plan = [('yaw-long-old', baseline, 7, 1, 0, 0, 'bfxrev', ''),
                ('yaw-long-new', SELECTED, 7, 1, 1, 0, 'bfxrev', ''),
                ('yaw-long-ap', baseline, 7, 2, 0, 0, 'bfxrev', ''),
                ('yaw-I-only', i_only, 3, 1, 0, 0, 'bfxrev', ''),
                ('yaw-D-only', d_only, 3, 1, 0, 0, 'bfxrev', ''),
                ('roll-D-only', rp_d, 1, 1, 0, 0, 'bfxrev', ''),
                ('roll-P-only', rp_p, 1, 1, 0, 0, 'bfxrev', ''),
                ('yaw-imax-010', narrow, 3, 1, 1, 0, 'bfxrev', ''),
                ('roll-aw-off', SELECTED, 1, 1, 0, 0, 'bfxrev', '')]
        for profile, axis in [(1, 0), (2, 1)]:
            for cutoff in [15, 30]:
                gains = [list(g) for g in SELECTED]
                gains[axis][4] = cutoff
                plan.append((f'axis{axis}-cutoff-{cutoff}', gains, profile, 1, 1, 0, 'bfxrev', ''))
    elif suite == 'pareto':
        plan = []
        for label, candidate in [('d15', [.19, .16, .0015, .19, 15]), ('d20', [.19, .16, .002, .19, 20])]:
            gains = [list(candidate), list(candidate), list(SELECTED[2])]
            for axis, profile in [('roll', 1), ('pitch', 2)]:
                for delay in [0, 1]:
                    plan.append((f'{axis}-{label}-delay{delay}', gains, profile, 1, 1, delay, 'bfxrev', ''))
    elif suite in ['robust', 'robust-rest', 'robust-final']:
        inertia = 3.*.25*(.35*.5)**2
        models = {'inertia-plus': dict(moment_inertia=[inertia*1.1, inertia*1.1, inertia*2.2]),
                  'inertia-minus': dict(moment_inertia=[inertia*.9, inertia*.9, inertia*1.8]),
                  'motor-curve': dict(propExpo=.715), 'motor-slew': dict(slew_max=100.),
                  'motor-slew-low': dict(slew_max=.1)}
        if suite in ['robust-rest', 'robust-final']:
            del models['motor-slew-low']
            models['motor-slew-05'] = dict(slew_max=.5)
        for name, data in models.items():
            (output/(name+'.json')).write_text(json.dumps(data)+'\n')
        plan = [(f'{axis}-{name}', SELECTED, profile, 1, 1, 0, f'bfxrev:../{name}.json', '')
                for profile, axis in enumerate(['roll', 'pitch', 'yaw'], 1) for name in models]
        if suite == 'robust-rest':
            plan = [item for item in plan if not item[0].startswith('roll-') or item[0] == 'roll-motor-slew-05']
        plan += [(f'{axis}-delay', SELECTED, profile, 1, 1, 1, 'bfxrev', '')
                 for profile, axis in enumerate(['roll', 'pitch', 'yaw'], 1)]
        plan += [('combined', SELECTED, 4, 1, 1, 0, 'bfxrev', ''),
                 ('sine', SELECTED, 5, 1, 1, 0, 'bfxrev', ''), ('sign', SELECTED, 6, 1, 1, 0, 'bfxrev', ''),
                 ('combined-noise', SELECTED, 4, 1, 1, 0, 'bfxrev', 'SIM_GYR1_RND 0.5\nSIM_GYR2_RND 0.5\n')]
        if suite == 'robust-final':
            plan = [tuple([item[0], FINAL, *item[2:]]) for item in plan
                    if item[0].startswith('roll-') or item[0].endswith('-delay') or item[2] in [4, 5, 6]]
    elif suite in ['saturation', 'stop']:
        # Deliberately weak yaw authority is a stress plant, never a tuning candidate.
        model = dict(motor1_yaw=.01, motor2_yaw=-.01, motor3_yaw=-.01, motor4_yaw=.01)
        (output/'weak-yaw.json').write_text(json.dumps(model)+'\n')
        if suite == 'saturation':
            result, _ = experiment(binary, output, 'yaw-long-verified-feedback', SELECTED, 7,
                                   model='bfxrev:../weak-yaw.json', extra='UTB_S_AMP 0.3\nUTB_RAT_YAW_IMAX 1\n',
                                   speedup=speedup, require_response=False)
            results['yaw-long-verified-feedback'] = result
            if result['allocation']['axes'][2]['blocked_cycles'] < 100:
                raise ValueError('Stress plant did not produce sustained directional saturation')
        config = parameters(SELECTED)+'UTB_S_OWNER 1\nUTB_S_PROFILE 7\nUTB_S_AW 0\nUTB_S_AMP 0.3\nUTB_RAT_YAW_IMAX 1\n'
        records = launch(binary, output, 'yaw-long-default-stop', lambda sim: flight(sim, 20, expected_fault=6),
                         config, model='bfxrev:../weak-yaw.json', speedup=speedup)
        stopped = [m for m in records['UAMT'] if m['Flt'] == 6]
        if len(stopped) < 100 or any(m['UN'] != stopped[0]['UN'] for m in stopped):
            raise ValueError('Saturation stop not observed or PID continued after fault')
        if any(m['APN'] != 0 for m in records['UAMT']):
            raise ValueError('Concurrent AP authority during saturation stop')
        stop = dict(status='EXPECTED SATURATION STOP', fault=6, stopped_pid_calls=stopped[0]['UN'],
                    stopped_samples=len(stopped))
        results['yaw-long-default-stop'] = stop
        (output/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
        return
    else:
        raise ValueError('Unknown suite')
    (output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    pairs = {}
    for case, gains, profile, owner, feedback, delay, model, extra in plan:
        result, rows = experiment(binary, output, case, gains, profile, owner, feedback, delay,
                                  model, extra, speedup)
        results[case] = result
        if suite == 'repeat':
            axis, label, repeat = case.split('-')
            pairs.setdefault((axis, repeat), {})[label] = rows
            if label == 'ap':
                group = pairs[(axis, repeat)]
                results[case]['alignment_old_new'] = alignment(group['old'], group['new'])
                results[case]['alignment_new_ap'] = alignment(group['new'], group['ap'])
                differences = [abs(group['new'][0][f'gyro{k}'] - group['ap'][0][f'gyro{k}']) for k in range(3)]
                results[case]['initial_gyro_difference_rads'] = max(differences)
        (output/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private functions
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Parameters.
#
# @param[in]  gains Per-axis controller gains.
# @return     Parameter values prepared for the explicit experiment configuration.
#
# @details    Global module function. Calls: zip, '\n'.join, lines.append, ValueError.
# --------------------------------------------------------------------------------------------------------------------
def parameters(gains):
    lines = []
    for axis, row in zip(['RLL', 'PIT', 'YAW'], gains):
        for name, value in zip(['P', 'I', 'D', 'IMAX', 'D_HZ'], row):
            low, high = RANGES[name]
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError('Outside bounded tuning range: '+name)
            lines.append(f'UTB_RAT_{axis}_{name} {value}')
    return '\n'.join(lines)+'\n'


# --------------------------------------------------------------------------------------------------------------------
# @brief      Quality.
#
# @param[in]  rows Exported sample records.
# @param[in]  axis Axis.
# @param[in]  measured Measured.
# @return     Analysis result; assertions reject invalid or incomplete input evidence.
#
# @details    Global module function. Calls: max, math.sqrt, sum, dict.
# --------------------------------------------------------------------------------------------------------------------
def quality(rows, axis, measured):
    amplitude = max(abs(r[f'ref{axis}']) for r in rows)
    errors = [r[f'target{axis}']-r[f'gyro{axis}'] for r in rows]
    rms = math.sqrt(sum(e*e for e in errors)/len(errors))
    effort = math.sqrt(sum(r[f'out{axis}']**2 for r in rows)/len(rows))
    metrics = measured['axis'][axis]
    tail_error = max(abs(s['steady_error_rads']) for s in metrics['steps'])
    overshoot = max(s['overshoot_percent']/100 for s in metrics['steps'])
    duration = sum(r['dt'] for r in rows)
    terms = dict(rms=.4*rms/amplitude, iae=.25*metrics['iae_rad']/(amplitude*duration),
                 tail=.15*tail_error/amplitude, overshoot=.1*overshoot, effort=.1*effort)
    return dict(score=sum(terms.values()), terms=terms, rms_tracking_error_rads=rms,
                rms_command=effort, max_abs_integrator=max(abs(r[f'i{axis}']) for r in rows),
                weights=dict(rms=.4, iae=.25, tail=.15, overshoot=.1, effort=.1))


# --------------------------------------------------------------------------------------------------------------------
# @brief      Run.
#
# @param[in]  binary Explicit local SITL executable.
# @param[in]  output Explicit destination for new experiment results.
# @param[in]  speedup Speedup.
# @return     None.
#
# @details    Global module function. Calls: output.mkdir, (output / 'candidate-plan.json').write_text, enumerate, (output /
#             'nominal-winners.json').write_text.
# --------------------------------------------------------------------------------------------------------------------
def run(binary, output, speedup):
    output.mkdir(parents=True, exist_ok=False)
    (output/'candidate-plan.json').write_text(json.dumps(dict(candidates=CANDIDATES, bounds=RANGES), indent=2)+'\n')
    results = {}
    for axis, name in enumerate(['roll', 'pitch', 'yaw']):
        group = []
        for number, candidate in enumerate(CANDIDATES[name]):
            gains = [list(BASELINE) for _ in range(3)]
            gains[axis] = candidate
            case = f'{name}-{number:02d}'
            try:
                records = launch(binary, output, case, flight,
                                 parameters(gains)+f'UTB_S_OWNER 1\nUTB_S_PROFILE {axis+1}\n', speedup=speedup)
                measured, rows = analyse_flight(records, 1, output/case)
                result = dict(case=case, gains=candidate, status='PASS', metrics=measured,
                              quality=quality(rows, axis, measured))
            except (AssertionError, RuntimeError) as failure:
                result = dict(case=case, gains=candidate, status='FAIL', reason=repr(failure))
            group.append(result)
            results[name] = group
            (output/'results.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
            print(case, result['status'], result.get('quality', {}).get('score'), flush=True)
    winners = {name: min((r for r in values if r['status'] == 'PASS'), key=lambda r: r['quality']['score'])['gains']
               for name, values in results.items()}
    (output/'nominal-winners.json').write_text(json.dumps(winners, indent=2)+'\n')
    print('Nominal winners; robustness/repeat validation still required:', winners, flush=True)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, run.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speedup', type=float, default=3)
    suites = ['sweep', 'preliminary', 'repeat', 'repeat-final', 'causal', 'robust', 'robust-rest', 'robust-final']
    suites += ['saturation', 'stop', 'aw-guards', 'pareto']
    parser.add_argument('--suite', choices=suites, default='sweep')
    args = parser.parse_args()
    if args.suite == 'sweep':
        run(args.binary.resolve(), args.output.resolve(), args.speedup)
    else:
        validation(args.binary.resolve(), args.output.resolve(), args.suite, args.speedup)


if __name__ == '__main__':
    main()

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Data type, constant, macro definition
# ====================================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private function prototypes
