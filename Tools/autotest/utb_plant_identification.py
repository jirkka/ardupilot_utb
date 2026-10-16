#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SI measurement reduction and explicit-provenance import into the existing plant."""
import argparse
import csv
import dataclasses
import json
import math
from pathlib import Path
import statistics

from utb_closed_loop import DT, Model, metrics, simulate, validate_curves, validate_tensor

UNITS = {'mass': 'kg', 'cg': 'm', 'inertia': 'kg*m^2', 'motor_positions': 'm',
         'thrust_curves': 'N', 'torque_curves': 'N*m', 'motor_tau': 's', 'damping': 'N*m*s/rad',
         'sensor_delay': 's', 'computation_delay': 's', 'actuator_delay': 's',
         'voltage': 'V', 'reference_voltage': 'V', 'force_voltage_exponent': '1', 'torque_voltage_exponent': '1',
         'gyro_noise': 'rad/s', 'actuator_ceiling': '1', 'collective': '1'}


def finite_tree(value):
    if isinstance(value, (tuple, list)):
        return all(finite_tree(x) for x in value)
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def tuples(value):
    return tuple(tuples(v) for v in value) if isinstance(value, list) else value


def load_dataset(data):
    required = {'schema', 'dataset_type', 'configuration_id', 'position_reference', 'command_domain', 'parameters'}
    if set(data) != required or data['schema'] != 'UTB_PLANT_SI_V1':
        raise ValueError('Incomplete/unknown calibration schema')
    if data['dataset_type'] not in ['demonstration', 'measurement'] or not data['configuration_id']:
        raise ValueError('Configuration/provenance required')
    if data['command_domain'] != 'normalized_model_actuator_state_0_1':
        raise ValueError('Unknown actuator mapping: this is not automatically PWM/DShot')
    if data['position_reference'] not in ['center_of_mass', 'body_origin']:
        raise ValueError('Position reference required')
    p = data['parameters']
    if set(p) != set(UNITS):
        raise ValueError('Missing/unknown physical parameters')
    values = {}
    for name, unit in UNITS.items():
        q = p[name]
        if set(q) != {'value', 'unit', 'status', 'u', 'method', 'source'}:
            raise ValueError('Quantity metadata missing: '+name)
        if q['unit'] != unit or not finite_tree(q['value']):
            raise ValueError('Unit/nonfinite value mismatch: '+name)
        if q['status'] not in ['measured', 'estimated', 'illustrative'] or not q['method'] or not q['source']:
            raise ValueError('Unknown/incomplete provenance: '+name)
        if not finite_tree(q['u']) or not isinstance(q['u'], (float, int)) or q['u'] < 0:
            raise ValueError('Nonnegative standard uncertainty required: '+name)
        if data['dataset_type'] == 'demonstration' and q['status'] == 'measured':
            raise ValueError('Demonstration data cannot be labelled measured')
        if data['dataset_type'] == 'measurement' and q['status'] == 'illustrative':
            raise ValueError('Measurement file contains illustrative physical values')
        values[name] = tuples(q['value'])
    shapes = {'cg': (3,), 'inertia': (3, 3), 'motor_positions': (4, 3), 'motor_tau': (4,),
              'damping': (3,), 'actuator_ceiling': (4,)}

    def shaped(value, shape):
        return (isinstance(value, tuple) and len(value) == shape[0] and
                (len(shape) == 1 or all(shaped(v, shape[1:]) for v in value)))
    for name, shape in shapes.items():
        if not shaped(values[name], shape):
            raise ValueError('Wrong physical vector/matrix dimensions: '+name)
    for name in set(UNITS)-set(shapes)-{'thrust_curves', 'torque_curves'}:
        if not isinstance(values[name], (int, float)):
            raise ValueError('Scalar quantity required: '+name)
    if values['mass'] <= 0 or len(values['cg']) != 3:
        raise ValueError('Invalid mass/center of mass')
    validate_tensor(values['inertia'])
    validate_curves(values['thrust_curves'])
    validate_curves(values['torque_curves'])
    positions = values['motor_positions']
    if len(positions) != 4 or any(len(row) != 3 for row in positions):
        raise ValueError('Four FRD motor positions required')
    if data['position_reference'] == 'body_origin':
        positions = tuple(tuple(v-c for v, c in zip(row, values['cg'])) for row in positions)
    delays = {}
    quantization = {}
    for name in ['sensor_delay', 'computation_delay', 'actuator_delay']:
        if values[name] < 0:
            raise ValueError('Negative delay')
        nominal_period = 1/400
        cycles = math.ceil(values[name]/nominal_period-1e-12)
        delays[name] = cycles
        quantization[name] = dict(input_s=values[name], model_s=cycles*DT, nominal_rounded_s=cycles*nominal_period,
                                  rounding_s=cycles*nominal_period-values[name],
                                  policy='whole nominal 400Hz cycle ceiling; model interval stored as float32')
    if values['voltage'] <= 0 or values['reference_voltage'] <= 0:
        raise ValueError('Positive voltage required')
    arm = max(math.hypot(row[0], row[1]) for row in positions)
    model = Model(inertia=tuple(values['inertia'][i][i] for i in range(3)), inertia_tensor=values['inertia'],
                  motor_positions=positions, arm=arm,
                  thrust_max=max(row[-1][1] for row in values['thrust_curves']),
                  thrust_curves=values['thrust_curves'], torque_curves=values['torque_curves'],
                  motor_tau_per_motor=values['motor_tau'], damping=values['damping'],
                  sensor_cycles=delays['sensor_delay'], compute_cycles=delays['computation_delay'],
                  actuator_cycles=delays['actuator_delay'], voltage_ratio=values['voltage']/values['reference_voltage'],
                  voltage_exponent=values['force_voltage_exponent'], torque_voltage_exponent=values['torque_voltage_exponent'],
                  noise_sigma=values['gyro_noise'], actuator_max=values['actuator_ceiling'], collective=values['collective'])
    label = ('DEMONSTRATION – NOT A MEASUREMENT OF SKYSTARS' if data['dataset_type'] == 'demonstration'
             else 'MEASURED/ESTIMATED INPUTS – HARDWARE MODEL VALIDATION PENDING')
    return model, dict(label=label, configuration_id=data['configuration_id'], parameters=p,
                       delay_quantization=quantization, mass_used_in_rotational_ode=False,
                       poles='General body tensor/geometry/curve trim linearization is not implemented')


def bifilar(drone_mass, fixture_mass, top_half_spacing, bottom_half_spacing, vertical_height,
            period, fixture_inertia=0., cg_offset=0., damping_ratio=0., gravity=9.80665,
            uncertainties=None, covariances=None):
    vals = [drone_mass, top_half_spacing, bottom_half_spacing, vertical_height, period, gravity]
    if any(not math.isfinite(v) or v <= 0 for v in vals):
        raise ValueError('Positive finite SI pendulum inputs required')
    if any(not math.isfinite(v) or v < 0 for v in [fixture_mass, fixture_inertia, cg_offset, damping_ratio]):
        raise ValueError('Invalid pendulum correction')
    if damping_ratio >= 1:
        raise ValueError('Underdamped period required')
    m = drone_mass+fixture_mass
    a, b, h, t, z, g = top_half_spacing, bottom_half_spacing, vertical_height, period, damping_ratio, gravity
    c = g*a*b*t*t*(1-z*z)/(4*math.pi**2*h)
    total = m*c
    inertia = total-fixture_inertia-drone_mass*cg_offset**2
    if inertia <= 0:
        raise ValueError('Fixture/CG correction leaves nonpositive drone inertia')
    gradient = {'drone_mass': c-cg_offset**2, 'fixture_mass': c, 'top_half_spacing': total/a,
                'bottom_half_spacing': total/b, 'vertical_height': -total/h, 'period': 2*total/t,
                'fixture_inertia': -1., 'cg_offset': -2*drone_mass*cg_offset,
                'damping_ratio': -2*z*total/(1-z*z), 'gravity': total/g}
    u = uncertainties or {}
    if set(u)-set(gradient) or any(not math.isfinite(v) or v < 0 for v in u.values()):
        raise ValueError('Invalid uncertainty fields')
    covariance = covariances or {}
    variance = sum((gradient[k]*value)**2 for k, value in u.items())
    keys = list(gradient)
    used_pairs = set()
    corr = [[1. if i == j else 0. for j in keys] for i in keys]
    for pair, value in covariance.items():
        first, second = pair.split('|')
        if first not in gradient or second not in gradient or first == second or not math.isfinite(value):
            raise ValueError('Invalid covariance pair')
        key = frozenset([first, second])
        if key in used_pairs:
            raise ValueError('Duplicate covariance pair')
        used_pairs.add(key)
        scale = u.get(first, 0)*u.get(second, 0)
        if abs(value) > scale or (not scale and value):
            raise ValueError('Covariance exceeds standard-uncertainty bounds')
        i, j = keys.index(first), keys.index(second)
        corr[i][j] = corr[j][i] = value/scale if scale else 0.
        variance += 2*gradient[first]*gradient[second]*value
    # Semidefinite Cholesky check, including perfectly correlated measurements.
    chol = [[0.]*len(keys) for _ in keys]
    for i in range(len(keys)):
        for j in range(i+1):
            residual = corr[i][j]-sum(chol[i][k]*chol[j][k] for k in range(j))
            if i == j:
                if residual < -1e-12:
                    raise ValueError('Covariance matrix is not positive semidefinite')
                chol[i][j] = math.sqrt(max(0, residual))
            elif chol[j][j]:
                chol[i][j] = residual/chol[j][j]
            elif abs(residual) > 1e-12:
                raise ValueError('Inconsistent singular covariance')
    if variance < -1e-20:
        raise ValueError('Negative propagated variance')
    std = math.sqrt(max(0, variance))
    return dict(inertia_kgm2=inertia, total_axis_inertia_kgm2=total, standard_uncertainty_kgm2=std,
                gradient=gradient, covariance=covariance,
                uncertainty_assumption='First-order standard uncertainty; omitted correlations assumed zero',
                approximation='Small-angle, equal-length filars; h is vertical rest height, not slanted wire length')


def pendulum_period(time_s, angle_rad):
    if len(time_s) != len(angle_rad) or len(time_s) < 10 or not finite_tree([time_s, angle_rad]):
        raise ValueError('Finite equal-length samples required')
    if any(b <= a for a, b in zip(time_s, time_s[1:])):
        raise ValueError('Monotonic sample times required')
    crossing = []
    for t0, t1, a0, a1 in zip(time_s, time_s[1:], angle_rad, angle_rad[1:]):
        if a0 < 0 <= a1:
            crossing.append(t0-a0*(t1-t0)/(a1-a0))
    periods = [b-a for a, b in zip(crossing, crossing[1:])]
    if len(periods) < 4:
        raise ValueError('At least five same-direction zero crossings required')
    return dict(period_s=statistics.mean(periods), repeatability_std_s=statistics.stdev(periods),
                cycles=len(periods), crossings_s=crossing,
                note='Crossing intervals share timestamps, not independent repeats; '
                     'omitted bias/timebase/amplitude uncertainty')


def cad_mass_properties(components):
    if not components:
        raise ValueError('Empty component model')
    mass = sum(c['mass_kg'] for c in components)
    if (not math.isfinite(mass) or mass <= 0
            or any(not finite_tree(c['mass_kg']) or c['mass_kg'] <= 0 or len(c['position_m']) != 3
                   or not finite_tree(c['position_m']) for c in components)):
        raise ValueError('Invalid component mass/position')
    center = [sum(c['mass_kg']*c['position_m'][i] for c in components)/mass for i in range(3)]
    j = [[0.]*3 for _ in range(3)]
    for c in components:
        local = c['body_aligned_local_inertia_kgm2']
        validate_tensor(local)
        d = [v-x for v, x in zip(c['position_m'], center)]
        squared = sum(v*v for v in d)
        for i in range(3):
            for k in range(3):
                j[i][k] += local[i][k]+c['mass_kg']*((squared if i == k else 0)-d[i]*d[k])
    validate_tensor(j)
    return dict(mass_kg=mass, cg_m=center, inertia_kgm2=j,
                note='Local tensors must already be rotated to body FRD axes; point-mass approximation is not assumed')


def run_dataset(dataset, output):
    data = json.loads(dataset.read_text())
    model, report = load_dataset(data)
    output.mkdir(parents=True, exist_ok=False)
    report.update(source_file=str(dataset), model=dataclasses.asdict(model),
                  disclaimer='SIMULATED – NOT HARDWARE VERIFIED', cases={})
    for index, kind in enumerate(['zero', 'roll', 'pitch', 'yaw', 'axes', 'saturation', 'sine_2']):
        rows, _ = simulate(model, kind, scenario=index)
        with (output/(kind+'.csv')).open('x', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        report['cases'][kind] = metrics(rows, kind)
    (output/'calibration.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(report['label'], 'Seven comparable scenarios written')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pendulum-csv', type=Path, help='Columns time_s,angle_rad; no physical input/output')
    args = parser.parse_args()
    if bool(args.dataset) == bool(args.pendulum_csv):
        parser.error('Select exactly one dataset or pendulum CSV')
    if args.dataset:
        run_dataset(args.dataset, args.output)
    else:
        with args.pendulum_csv.open() as stream:
            rows = list(csv.DictReader(stream))
        result = pendulum_period([float(r['time_s']) for r in rows], [float(r['angle_rad']) for r in rows])
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output/'period.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
