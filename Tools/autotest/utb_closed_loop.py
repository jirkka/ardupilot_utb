#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline rotational model. SIMULATED – NOT HARDWARE VERIFIED.

Reuse utb_math_reference for PID/allocator. No MAVLink, SITL vehicle or device I/O.
The actuator conversion and every physical default below are illustrative.
"""
import argparse
import csv
import dataclasses
import json
import math
import random
from functools import cached_property
from pathlib import Path

from utb_math_reference import Reference, f32
from utb_math_validation import write_vectors

DT = f32(.0025)
GAINS = tuple(tuple(map(f32, (.31, .23, .004, .19, 20))) for _ in range(3))
DISCLAIMER = 'SIMULATED – NOT HARDWARE VERIFIED'


@dataclasses.dataclass(frozen=True)
class Model:
    inertia: tuple = (.002, .0025, .004)
    arm: float = .11
    thrust_max: float = 3.0
    reaction_length: float = .015
    thrust_exponent: float = 1.0
    motor_tau: float = .03
    damping: tuple = (.003, .003, .003)
    collective: float = .5
    actuator_max: tuple = (1., 1., 1., 1.)
    sensor_cycles: int = 0
    compute_cycles: int = 0
    actuator_cycles: int = 0
    noise_sigma: float = 0.
    jitter_fraction: float = 0.
    step: float = DT
    substeps: int = 4
    coupled: bool = True
    inertia_tensor: tuple | None = None
    motor_positions: tuple | None = None
    thrust_curves: tuple | None = None
    torque_curves: tuple | None = None
    motor_tau_per_motor: tuple | None = None
    voltage_ratio: float = 1.
    voltage_exponent: float = 2.
    torque_voltage_exponent: float = 2.

    def __post_init__(self):
        positive = (*self.inertia, self.arm, self.thrust_max, self.reaction_length, self.thrust_exponent, self.step)
        nonnegative = (*self.damping, self.motor_tau, self.noise_sigma, self.jitter_fraction)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError('Physical scales must be finite and positive')
        if any(not math.isfinite(v) or v < 0 for v in nonnegative):
            raise ValueError('Damping, lag, noise and jitter must be finite and nonnegative')
        if len(self.inertia) != 3 or len(self.damping) != 3 or len(self.actuator_max) != 4:
            raise ValueError('Invalid vector dimensions')
        if 2 * max(self.inertia) > sum(self.inertia):
            raise ValueError('Principal inertia triangle inequality')
        if not 0 < self.collective < 1 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in self.actuator_max):
            raise ValueError('Invalid collective or actuator ceiling')
        if self.jitter_fraction >= 1 or not 0.000625 <= self.step * (1-self.jitter_fraction):
            raise ValueError('dt lower bound outside production nominal400 contract')
        if self.step * (1+self.jitter_fraction) > .01 or self.substeps < 1:
            raise ValueError('dt upper bound/substeps invalid')
        for value in [self.sensor_cycles, self.compute_cycles, self.actuator_cycles, self.substeps]:
            if not isinstance(value, int) or value < 0:
                raise ValueError('Cycle counts must be nonnegative integers')
        if not math.isfinite(self.voltage_ratio) or self.voltage_ratio <= 0:
            raise ValueError('Positive voltage ratio required')
        if any(not math.isfinite(v) or v < 0 for v in [self.voltage_exponent, self.torque_voltage_exponent]):
            raise ValueError('Invalid voltage scaling exponent')
        if self.inertia_tensor is not None:
            validate_tensor(self.inertia_tensor)
        if self.motor_positions is not None:
            if len(self.motor_positions) != 4 or any(len(r) != 3 for r in self.motor_positions):
                raise ValueError('Four FRD motor positions required')
            if any(not math.isfinite(v) for r in self.motor_positions for v in r):
                raise ValueError('Nonfinite motor position')
        for curves in [self.thrust_curves, self.torque_curves]:
            if curves is not None:
                validate_curves(curves)
        if self.motor_tau_per_motor is not None:
            if len(self.motor_tau_per_motor) != 4 or any(not math.isfinite(v) or v < 0 for v in self.motor_tau_per_motor):
                raise ValueError('Four nonnegative motor lag values required')

    @cached_property
    def inverse_inertia(self):
        return inverse3(self.inertia_tensor) if self.inertia_tensor is not None else None


def determinant3(a):
    return (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) -
            a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0]) + a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))


def validate_tensor(a):
    if (not isinstance(a, (tuple, list)) or len(a) != 3
            or any(not isinstance(row, (tuple, list)) or len(row) != 3 for row in a)):
        raise ValueError('3x3 inertia tensor required')
    if any(not math.isfinite(v) for row in a for v in row):
        raise ValueError('Finite 3x3 inertia tensor required')
    if any(a[i][j] != a[j][i] for i in range(3) for j in range(3)):
        raise ValueError('Inertia tensor must be symmetric')
    scale = max(abs(v) for row in a for v in row)
    if scale == 0:
        raise ValueError('Zero inertia tensor')
    b = [[v/scale for v in row] for row in a]
    if b[0][0] <= 0 or b[0][0]*b[1][1]-b[0][1]**2 <= 0 or determinant3(b) <= 0:
        raise ValueError('Inertia tensor is not positive definite')
    # A physically realizable inertia has a positive semidefinite second-moment tensor.
    trace = sum(b[i][i] for i in range(3))
    q = [[(trace/2 if i == j else 0)-b[i][j] for j in range(3)] for i in range(3)]
    minors = [q[i][i] for i in range(3)]
    minors += [q[i][i]*q[j][j]-q[i][j]**2 for i, j in [(0, 1), (0, 2), (1, 2)]]
    if min(minors+[determinant3(q)]) < -1e-12:
        raise ValueError('Inertia tensor violates realizability/triangle condition')


def inverse3(a):
    scale = max(abs(v) for row in a for v in row)
    b = [[v/scale for v in row] for row in a]
    det = determinant3(b)
    result = []
    for i in range(3):
        row = []
        for j in range(3):
            rr, cc = [k for k in range(3) if k != j], [k for k in range(3) if k != i]
            row.append((-1)**(i+j)*(b[rr[0]][cc[0]]*b[rr[1]][cc[1]]-b[rr[0]][cc[1]]*b[rr[1]][cc[0]])/det/scale)
        result.append(row)
    if any(not math.isfinite(v) for row in result for v in row):
        raise ValueError('Numerically unrepresentable inverse inertia')
    return result


def validate_curves(curves):
    if not isinstance(curves, (tuple, list)) or len(curves) != 4:
        raise ValueError('Four actuator curves required')
    for curve in curves:
        if not isinstance(curve, (tuple, list)) or len(curve) < 2:
            raise ValueError('At least two curve points required')
        if any(not isinstance(point, (tuple, list)) or len(point) != 2 or
               not all(math.isfinite(v) for v in point) or point[1] < 0 for point in curve):
            raise ValueError('Invalid actuator curve')
        if curve[0][0] != 0 or curve[-1][0] != 1 or curve[0][1] != 0:
            raise ValueError('Curve must span normalized command 0..1 and start with zero force/torque')
        if any(b[0] <= a[0] or b[1] < a[1] for a, b in zip(curve, curve[1:])):
            raise ValueError('Curve must be monotone, without duplicate commands')


def interpolate_curve(curve, value):
    value = max(0., min(1., value))
    for a, b in zip(curve, curve[1:]):
        if value <= b[0]:
            return a[1]+(b[1]-a[1])*(value-a[0])/(b[0]-a[0])
    return curve[-1][1]


def validate_gains(gains):
    if len(gains) != 3 or any(len(g) != 5 for g in gains):
        raise ValueError('Three complete gain tuples required')
    for p, i, d, imax, hz in gains:
        if any(not math.isfinite(v) for v in [p, i, d, imax, hz]):
            raise ValueError('Nonfinite gains')
        if not (0 <= p <= 5 and 0 <= i <= 5 and 0 <= d <= 1 and 0 <= imax <= 1 and 0 <= hz <= 200):
            raise ValueError('Production gain limits')
        if d > 0 and not 0 < hz <= min(200, .1 / DT):
            raise ValueError('Production D cutoff limit')


def forces_and_torque(motors, model):
    """FRD body, M1 rear-right, M2 front-right, M3 rear-left, M4 front-left."""
    a = model.arm / math.sqrt(2)
    positions = model.motor_positions or [(-a, a, 0), (a, a, 0), (-a, -a, 0), (a, -a, 0)]
    reaction = [1, -1, -1, 1]
    force = [model.thrust_max * max(0., min(1., q)) ** model.thrust_exponent for q in motors]
    if model.thrust_curves is not None:
        force = [interpolate_curve(curve, q) for curve, q in zip(model.thrust_curves, motors)]
    if model.voltage_ratio != 1:
        force = [f*model.voltage_ratio**model.voltage_exponent for f in force]
    yaw = [model.reaction_length*f for f in force]
    if model.torque_curves is not None:
        yaw = [interpolate_curve(curve, q)*model.voltage_ratio**model.torque_voltage_exponent
               for curve, q in zip(model.torque_curves, motors)]
    torque = [sum(-y * f for (_, y, _), f in zip(positions, force)),
              sum(x * f for (x, _, _), f in zip(positions, force)),
              sum(sign * value for sign, value in zip(reaction, yaw))]
    return force, torque


def derivative(state, command, model):
    w, q = state[:3], state[3:]
    _, torque = forces_and_torque(q, model)
    j = model.inertia
    cross = [w[1]*w[2]*(j[2]-j[1]), w[2]*w[0]*(j[0]-j[2]), w[0]*w[1]*(j[1]-j[0])]
    dw = [(torque[a] - model.damping[a]*w[a] - (cross[a] if model.coupled else 0)) / j[a] for a in range(3)]
    if model.inertia_tensor is not None:
        jw = [sum(v*x for v, x in zip(row, w)) for row in model.inertia_tensor]
        cross = [w[1]*jw[2]-w[2]*jw[1], w[2]*jw[0]-w[0]*jw[2], w[0]*jw[1]-w[1]*jw[0]]
        rhs = [torque[a]-model.damping[a]*w[a]-(cross[a] if model.coupled else 0) for a in range(3)]
        dw = [sum(v*x for v, x in zip(row, rhs)) for row in model.inverse_inertia]
    lag = model.motor_tau_per_motor or (model.motor_tau,)*4
    dq = [(command[a]-q[a])/lag[a] if lag[a] else 0. for a in range(4)]
    return dw + dq


def advance(state, command, dt, model):
    if not math.isfinite(dt) or dt <= 0 or len(state) != 7 or len(command) != 4:
        raise ValueError('Invalid plant interval or vector size')
    if any(not math.isfinite(v) for v in [*state, *command]):
        raise ValueError('Nonfinite plant state/command')
    command = [min(model.actuator_max[a], max(0., command[a])) for a in range(4)]
    state = list(state)
    lag = model.motor_tau_per_motor or (model.motor_tau,)*4
    for a in range(4):
        if not lag[a]:
            state[3+a] = command[a]
    count = model.substeps
    for tau in lag:
        if tau:
            count = max(count, math.ceil(dt/(tau/8)))
    h = dt / count
    for _ in range(count):
        k1 = derivative(state, command, model)
        k2 = derivative([v+h*k/2 for v, k in zip(state, k1)], command, model)
        k3 = derivative([v+h*k/2 for v, k in zip(state, k2)], command, model)
        k4 = derivative([v+h*k for v, k in zip(state, k3)], command, model)
        state = [v+h*(a+2*b+2*c+d)/6 for v, a, b, c, d in zip(state, k1, k2, k3, k4)]
    if any(not math.isfinite(v) for v in state) or max(map(abs, state[:3])) > 1e4:
        raise ArithmeticError('Numerical/model divergence; no silent clipping of body rates')
    return state


def reference(t, kind):
    if kind == 'zero' or t < 1:
        return [0., 0., 0.]
    if kind in ['roll', 'pitch', 'yaw', 'negative']:
        index = {'roll': 0, 'pitch': 1, 'yaw': 2, 'negative': 0}[kind]
        r = [0., 0., 0.]
        r[index] = -.3 if kind == 'negative' else .3
        return r
    if kind == 'reversal':
        return [.3 if t < 4 else -.3, 0., 0.]
    if kind == 'saturation':
        return [15., -10., 8.] if t < 4 else [0., 0., 0.]
    if kind == 'axes':
        return [.3, -.3, .3]
    if kind.startswith('sine'):
        hz = float(kind.split('_')[1])
        return [.3 * math.sin(2*math.pi*hz*(t-1)), 0., 0.]
    raise ValueError(kind)


def simulate(model=Model(), kind='roll', seconds=8., seed=1729, gains=GAINS, scenario=0):
    validate_gains(gains)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('Duration must be positive')
    rng = random.Random(seed)
    controller, t, sample = Reference(), 0., 0
    state = [0., 0., 0.] + [min(model.collective, ceiling) for ceiling in model.actuator_max]
    sensor = [[0., 0., 0.] for _ in range(model.sensor_cycles)]
    delay = [[model.collective]*4 for _ in range(model.compute_cycles+model.actuator_cycles)]
    rows, inputs = [], []
    while t < seconds:
        dt = f32(model.step * (1 + rng.uniform(-model.jitter_fraction, model.jitter_fraction)))
        measured = sensor.pop(0) if sensor else state[:3]
        if model.sensor_cycles:
            sensor.append(state[:3])
        clean_measurement = measured[:]
        measured = [f32(v+rng.gauss(0, model.noise_sigma)) for v in measured]
        r = list(map(f32, reference(t, kind)))
        inp = dict(scenario=scenario, sample=sample, action=0, frame=18, dt=dt, thrust=f32(model.collective),
                   target=r, measurement=measured, gains=gains)
        result = controller.step(inp)
        command = [result['m'+str(a)].value for a in range(4)] if result['mvalid'] else [model.collective]*4
        applied = delay.pop(0) if delay else command
        if model.compute_cycles+model.actuator_cycles:
            delay.append(command)
        force, torque = forces_and_torque(state[3:], model)
        row = dict(t=t, dt=dt, valid=result['valid'], mvalid=result['mvalid'], prime=result['prime'],
                   scaled=result['scaled'], shifted=result['shifted'])
        for a in range(3):
            row.update({f'r{a}': r[a], f'w{a}': state[a], f'meas{a}': measured[a], f'torque{a}': torque[a]})
            # Difference from the delayed noiseless sample; separate transport error from noise.
            row['noise'+str(a)] = measured[a] - clean_measurement[a]
            for name in ['e', 'p', 'i', 'd', 'raw', 'out', 'a']:
                row[name+str(a)] = result[name+str(a)].value
            row['blocked'+str(a)] = result['blocked'+str(a)]
        for a in range(4):
            row.update({f'm{a}': command[a], f'q{a}': state[3+a], f'force{a}': force[a],
                        f'actuator_limited{a}': int(applied[a] > model.actuator_max[a])})
        state = advance(state, applied, dt, model)
        rows.append(row)
        inputs.append(inp)
        sample, t = sample+1, t+dt
    return rows, inputs


def metrics(rows, kind):
    results = []
    for a in range(3):
        iae = sum(abs(r[f'r{a}']-r[f'w{a}'])*r['dt'] for r in rows)
        itae = sum(r['t']*abs(r[f'r{a}']-r[f'w{a}'])*r['dt'] for r in rows)
        result = dict(iae=iae, itae=itae, max_abs_rate=max(abs(r[f'w{a}']) for r in rows),
                      max_abs_raw=max(abs(r[f'raw{a}']) for r in rows),
                      max_abs_integral=max(abs(r[f'i{a}']) for r in rows),
                      integral_gate_s=sum(r['dt'] for r in rows if r[f'blocked{a}']),
                      output_saturation_s=sum(r['dt'] for r in rows if abs(r[f'raw{a}']) > 1),
                      raw_rms=math.sqrt(sum(r[f'raw{a}']**2 for r in rows)/len(rows)),
                      measurement_noise_rms=math.sqrt(sum(r[f'noise{a}']**2 for r in rows)/len(rows)),
                      sensor_error_rms=math.sqrt(sum((r[f'meas{a}']-r[f'w{a}'])**2 for r in rows)/len(rows)))
        steady = [r for r in rows if r['t'] >= rows[-1]['t']-1]
        result['last_second_mean_error'] = sum(r[f'r{a}']-r[f'w{a}'] for r in steady)/len(steady)
        result['last_second_rate_peak_to_peak'] = max(r[f'w{a}'] for r in steady)-min(r[f'w{a}'] for r in steady)
        target = rows[-1][f'r{a}']
        # Metrics apply only to a single nonzero step; no false settling times for sine/reversal.
        result.update(rise_s=None, overshoot_percent=None, settle_2percent_s=None)
        if kind in ['roll', 'pitch', 'yaw', 'negative', 'axes'] and target:
            step = [r for r in rows if r['t'] >= 1]
            event = step[0]['t']
            crossing = [next((r['t']-event for r in step if r[f'w{a}']/target >= fraction), None) for fraction in [.1, .9]]
            if None not in crossing:
                result['rise_s'] = crossing[1]-crossing[0]
            result['overshoot_percent'] = max(0., 100*(max(r[f'w{a}']/target for r in step)-1))
            bad = [i for i, r in enumerate(step) if abs(r[f'w{a}']-target) > .02*abs(target)]
            if bad and bad[-1]+1 < len(step):
                result['settle_2percent_s'] = step[bad[-1]+1]['t']-event
            if not bad:
                result['settle_2percent_s'] = 0.
        if kind.startswith('sine') and a == 0:
            hz = float(kind.split('_')[1])
            selected = [r for r in rows if r['t'] >= rows[-1]['t']-4]

            def harmonic(field):
                return sum(r[field]*complex(math.cos(2*math.pi*hz*r['t']), -math.sin(2*math.pi*hz*r['t']))
                           for r in selected) * 2/len(selected)
            ratio = harmonic('w0') / harmonic('r0')
            result.update(closed_loop_gain=abs(ratio), phase_deg=math.degrees(math.atan2(ratio.imag, ratio.real)))
        results.append(result)
    return dict(axes=results, mixer_scaled_s=sum(r['dt'] for r in rows if r['scaled']),
                collective_shift_s=sum(r['dt'] for r in rows if r['shifted']),
                actuator_limited_s=sum(r['dt'] for r in rows if any(r[f'actuator_limited{a}'] for a in range(4))),
                samples=len(rows), invalid=sum(not r['valid'] for r in rows), duration_s=sum(r['dt'] for r in rows))


def linear_step(x, model, axis, gains=GAINS, dt=DT):
    """Exact ZOH unsaturated single-axis map, gamma linearized at collective.

    State = omega, normalized actuator moment, I, filtered acceleration,
    previous measurement, sensor FIFO, command FIFO. No priming transient.
    """
    p, ki, kd, _, hz = gains[axis]
    n, nd = model.sensor_cycles, model.compute_cycles+model.actuator_cycles
    w, q, integral, accel, previous = x[:5]
    measured = x[5] if n else w
    new_sensor = list(x[6:5+n]) + [w] if n else []
    integral -= ki*measured*dt
    beta = dt / (1/(2*math.pi*hz)+dt) if kd else 1.
    accel = (1-beta)*accel+beta*(measured-previous)/dt if kd else 0.
    u = -p*measured+integral-kd*accel
    applied = x[5+n] if nd else u
    new_command = list(x[6+n:5+n+nd])+[u] if nd else []
    k = (math.sqrt(2)*model.arm*model.thrust_max if axis < 2 else 2*model.reaction_length*model.thrust_max)
    k *= model.thrust_exponent*model.collective**(model.thrust_exponent-1)
    k *= model.voltage_ratio**model.voltage_exponent
    lam = model.damping[axis]/model.inertia[axis]
    decay = math.exp(-lam*dt)
    uniform = -math.expm1(-lam*dt)/lam if lam else dt
    if model.motor_tau:
        mu = 1/model.motor_tau
        lag = math.exp(-mu*dt)
        mixed = (lag-decay)/(lam-mu) if abs(lam-mu) > 1e-9 else dt*decay
        w = w*decay+k/model.inertia[axis]*(applied*uniform+(q-applied)*mixed)
        q = applied+(q-applied)*lag
    else:
        w = w*decay+k/model.inertia[axis]*applied*uniform
        q = applied
    return [w, q, integral, accel, measured]+new_sensor+new_command


def poles(model, axis):
    if any(v is not None for v in [model.inertia_tensor, model.motor_positions, model.thrust_curves,
                                   model.torque_curves, model.motor_tau_per_motor]):
        raise ValueError('Single-axis poles require the original symmetric/diagonal plant; general trim not implemented')
    import numpy as np
    size = 5+model.sensor_cycles+model.compute_cycles+model.actuator_cycles
    matrix = np.array([linear_step(np.eye(size)[i], model, axis, dt=model.step) for i in range(size)]).T
    values = np.linalg.eigvals(matrix)
    return dict(radius=float(max(abs(values))), poles=[[float(v.real), float(v.imag)] for v in values])


def scenarios():
    base = Model()
    kinds = ['zero', 'roll', 'pitch', 'yaw', 'negative', 'reversal', 'axes',
             'saturation', 'sine_0.5', 'sine_2', 'sine_10']
    result = [(kind, kind, base) for kind in kinds]
    result += [('delay_5ms', 'roll', dataclasses.replace(base, sensor_cycles=1, compute_cycles=1)),
               ('delay_30ms', 'roll', dataclasses.replace(base, sensor_cycles=4, compute_cycles=4, actuator_cycles=4)),
               ('jitter20percent', 'axes', dataclasses.replace(base, jitter_fraction=.2)),
               ('dt_5ms', 'roll', dataclasses.replace(base, step=f32(.005))),
               ('noise', 'zero', dataclasses.replace(base, noise_sigma=.02)),
               ('actuator_ceiling', 'saturation', dataclasses.replace(base, actuator_max=(.6,)*4)),
               ('one_motor_weak', 'axes', dataclasses.replace(base, actuator_max=(.3, 1., 1., 1.))),
               ('quadratic_thrust', 'axes', dataclasses.replace(base, thrust_exponent=2)),
               ('inertia_half', 'roll', dataclasses.replace(base, inertia=tuple(v/2 for v in base.inertia))),
               ('inertia_double', 'roll', dataclasses.replace(base, inertia=tuple(v*2 for v in base.inertia))),
               ('lag_80ms', 'roll', dataclasses.replace(base, motor_tau=.08)),
               ('no_damping', 'roll', dataclasses.replace(base, damping=(0.,)*3)),
               ('decoupled', 'saturation', dataclasses.replace(base, coupled=False))]
    return result


def run_suite(output):
    output.mkdir(parents=True, exist_ok=False)
    report = {'disclaimer': DISCLAIMER, 'nominal_controller_dt': DT, 'gains': GAINS, 'seed': 1729, 'cases': {}}
    replay = []
    for index, (name, kind, model) in enumerate(scenarios()):
        rows, inputs = simulate(model, kind, scenario=index)
        replay.extend(inputs)
        with (output / (name+'.csv')).open('x', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        report['cases'][name] = dict(model=dataclasses.asdict(model), reference=kind, **metrics(rows, kind))
    write_vectors(replay, output/'pid-replay.txt')
    (output/'simulation.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('Simulation cases:', len(report['cases']), 'Replay samples:', len(replay))


def stability_suite(output):
    base = Model()
    report = {'disclaimer': DISCLAIMER, 'method': 'Exact discrete unsaturated ZOH, single axis, diagonal J, gamma slope',
              'nominal': [poles(base, a) for a in range(3)], 'delay_sweep': [], 'parameter_sweep': [],
              'case_radii': {name: [poles(model, a)['radius'] for a in range(3)]
                             for name, _, model in scenarios() if model.actuator_max == (1.,)*4}}
    for delay in range(0, 81):
        m = dataclasses.replace(base, sensor_cycles=delay)
        report['delay_sweep'].append(dict(sensor_cycles=delay, delay_s=delay*DT,
                                          radius=[poles(m, a)['radius'] for a in range(3)]))
    for jfactor in [.5, 1., 2.]:
        for dfactor in [0., 1., 3.]:
            for lag in [0., .01, .03, .08]:
                for delay in [0, 2, 8]:
                    m = dataclasses.replace(base, inertia=tuple(v*jfactor for v in base.inertia),
                                            damping=tuple(v*dfactor for v in base.damping), motor_tau=lag, sensor_cycles=delay)
                    record = dict(jfactor=jfactor, damping_factor=dfactor, lag_s=lag, delay_cycles=delay,
                                  radius=[poles(m, a)['radius'] for a in range(3)])
                    report['parameter_sweep'].append(record)
    (output/'stability.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('Local stability parameter cases:', len(report['parameter_sweep']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stability-only', action='store_true', help='Requires existing NumPy; writes into suite directory')
    args = parser.parse_args()
    if args.stability_only:
        stability_suite(args.output)
    else:
        run_suite(args.output)


if __name__ == '__main__':
    main()
