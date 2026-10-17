#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Illustrative plant uncertainty and identifiability analysis, without tuning PID."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_plant_sensitivity.py
# @brief      Ilustrativní nejistoty/identifikovatelnost plantu; neladí PID fyzické FC.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import csv
import dataclasses
import json
import math
from pathlib import Path

from utb_closed_loop import Model, metrics, simulate


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Variations.
#
# @param[in]  base Base.
# @return     Returned values: cases.
#
# @details    Global module function. Calls: enumerate, dataclasses.replace, list, cases.append.
# --------------------------------------------------------------------------------------------------------------------
def variations(base):
    cases = [('baseline', base)]
    for axis, name in enumerate(['Jxx', 'Jyy', 'Jzz']):
        for factor in [.9, 1.1]:
            j = list(base.inertia)
            j[axis] *= factor
            cases.append((name+'_'+str(factor), dataclasses.replace(base, inertia=tuple(j))))
    for name, field, factors in [('motor_tau', 'motor_tau', [.75, 1.25]), ('damping', 'damping', [.5, 1.5]),
                                 ('max_torque', 'thrust_max', [.8, 1.2]), ('voltage', 'voltage_ratio', [.9, 1.1])]:
        for factor in factors:
            value = getattr(base, field)
            value = tuple(v*factor for v in value) if isinstance(value, tuple) else value*factor
            cases.append((name+'_'+str(factor), dataclasses.replace(base, **{field: value})))
    common = dataclasses.replace(base, inertia=tuple(v*1.5 for v in base.inertia),
                                 damping=tuple(v*1.5 for v in base.damping), thrust_max=base.thrust_max*1.5)
    cases += [('sensor_2p5ms', dataclasses.replace(base, sensor_cycles=1)),
              ('actuator_2p5ms', dataclasses.replace(base, actuator_cycles=1)),
              ('common_scale_1p5', common)]
    return cases


# --------------------------------------------------------------------------------------------------------------------
# @brief      Torque bounds.
#
# @param[in]  model Model.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: range, dict, forces_and_torque, columns.append.
# --------------------------------------------------------------------------------------------------------------------
def torque_bounds(model):
    from utb_closed_loop import forces_and_torque
    columns = []
    for i in range(4):
        q = [0.]*4
        q[i] = model.actuator_max[i]
        _, torque = forces_and_torque(q, model)
        columns.append(torque)
    return dict(min_Nm=[sum(min(0, row[a]) for row in columns) for a in range(3)],
                max_Nm=[sum(max(0, row[a]) for row in columns) for a in range(3)],
                assumption='Independent motor box, no fixed collective/hover constraint; not simultaneous axis authority')


# --------------------------------------------------------------------------------------------------------------------
# @brief      Run.
#
# @param[in]  output Explicit destination for new experiment results.
# @return     None.
#
# @details    Global module function. Calls: output.mkdir, Model, dict, enumerate.
# --------------------------------------------------------------------------------------------------------------------
def run(output):
    output.mkdir(parents=True, exist_ok=False)
    base = Model()
    report = dict(disclaimer='SIMULATED – NOT HARDWARE VERIFIED', physical_evidence='ILLUSTRATIVE', cases={})
    stored = {}
    for index, (name, model) in enumerate(variations(base)):
        rows, _ = simulate(model, 'axes', scenario=index)
        stored[name] = rows
        with (output/(name+'.csv')).open('x', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        report['cases'][name] = dict(model=dataclasses.asdict(model), torque_bounds=torque_bounds(model),
                                     **metrics(rows, 'axes'))
    baseline = stored['baseline']
    for name, rows in stored.items():
        changes = [math.sqrt(sum((r[f'w{a}']-b[f'w{a}'])**2 for r, b in zip(rows, baseline))/len(rows))
                   for a in range(3)]
        report['cases'][name]['rms_change_from_baseline'] = changes
    common = stored['common_scale_1p5']
    delay1, delay2 = stored['sensor_2p5ms'], stored['actuator_2p5ms']
    common_error = max(abs(a[f'w{i}']-b[f'w{i}']) for a, b in zip(common, baseline) for i in range(3))
    delay_error = max(abs(a[f'meas{i}']-b[f'meas{i}']) for a, b in zip(delay1, delay2) for i in range(3))
    report['identifiability'] = {
        'common_scale_max_rate_difference': common_error,
        'sensor_vs_actuator_max_measured_gyro_difference': delay_error,
        'note': 'Common J/D/thrust scale and pure delay location are confounded. True omega is not observed.'}
    # Local finite differences use 1% perturbations. Every sample is kept; derivatives normalized by reference .3rad/s.
    columns, names = [], []
    for field in ['Jxx', 'Jyy', 'Jzz', 'motor_tau', 'damping', 'thrust_max', 'voltage_ratio']:
        pairs = []
        for factor in [.99, 1.01]:
            if field.startswith('J'):
                axis = ['Jxx', 'Jyy', 'Jzz'].index(field)
                j = list(base.inertia)
                j[axis] *= factor
                changed = dataclasses.replace(base, inertia=tuple(j))
            else:
                v = getattr(base, field)
                v = tuple(x*factor for x in v) if isinstance(v, tuple) else v*factor
                changed = dataclasses.replace(base, **{field: v})
            rows, _ = simulate(changed, 'axes')
            pairs.append(rows)
        columns.append([(plus[f'meas{a}']-minus[f'meas{a}'])/(.02*.3)
                        for minus, plus in zip(*pairs) for a in range(3)])
        names.append(field)
    with (output/'local-sensitivity.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(zip(*columns))
    report['local_columns'] = names
    report['local_fd_relative_step'] = .01
    (output/'sensitivity.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(len(report['cases']), 'uncertainty cases; seven local derivative columns')


# --------------------------------------------------------------------------------------------------------------------
# @brief      Svd.
#
# @param[in]  output Explicit destination for new experiment results.
# @return     None.
#
# @details    Global module function. Calls: np.linalg.svd, np.corrcoef, dict, (output /
#             'identifiability-svd.json').write_text.
# --------------------------------------------------------------------------------------------------------------------
def svd(output):
    import numpy as np
    with (output/'local-sensitivity.csv').open() as stream:
        reader = csv.reader(stream)
        names = next(reader)
        matrix = np.array([list(map(float, row)) for row in reader])
    _, singular, vt = np.linalg.svd(matrix, full_matrices=False)
    correlation = np.corrcoef(matrix.T)
    result = dict(parameters=names, singular_values=singular.tolist(), right_singular_vectors=vt.tolist(),
                  correlation=correlation.tolist(), numerical_rank_relative_1e3=int(sum(singular > singular[0]*1e-3)),
                  caveat='Local illustrative conditioning, not measured identifiability or confidence bounds')
    (output/'identifiability-svd.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('Singular values', singular.tolist())


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, svd.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--svd-only', action='store_true')
    args = parser.parse_args()
    if args.svd_only:
        svd(args.output)
    else:
        run(args.output)


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
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private function prototypes
# ====================================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private functions
