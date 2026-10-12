#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Generate synthetic inputs, compare the real C++ implementation with offline mathematics."""
import argparse
import csv
import itertools
import json
import math
import os
from pathlib import Path
import subprocess

from utb_math_reference import Bound, EPS, NEAR_ONE, Reference, f32

FIELDS = ['scenario', 'sample', 'valid', 'prime', 'mvalid']
for axis in range(3):
    FIELDS += [f'{name}{axis}' for name in ('e', 'p', 'i', 'd', 'raw', 'out', 'acc', 'blocked')]
FIELDS += [f'm{i}' for i in range(4)] + [f'a{i}' for i in range(3)]
FIELDS += ['at', 'scale', 'shift', 'pos', 'neg', 'lo', 'hi', 'scaled', 'shifted']


def scenarios():
    rows, names = [], []

    def append(name, inputs, gains, frame=18):
        scenario = len(names)
        names.append(name)
        for sample, (target, measurement, dt, thrust, action) in enumerate(inputs):
            rows.append(dict(scenario=scenario, sample=sample, action=action, frame=frame, dt=f32(dt),
                             thrust=f32(thrust), target=list(map(f32, target)), measurement=list(map(f32, measurement)),
                             gains=[tuple(map(f32, g)) for g in gains]))

    combined = [(0.31, .23, .004, .19, 20), (.27, .31, .003, .17, 10), (.19, .17, .002, .13, 5)]
    for name in ('constant', 'positive-step', 'negative-step', 'sign-reversal', 'ramp', 'reference-sine',
                 'measurement-sine', 'impulse', 'saturation-recovery', 'jitter', 'invalidation-recovery'):
        inputs = []
        for n in range(1200):
            t = n * .0025
            value, measured, dt, action = .2, .05, .0025, 0
            if name == 'positive-step':
                value = .9 if n >= 80 else 0
            elif name == 'negative-step':
                value = -.9 if n >= 80 else 0
            elif name == 'sign-reversal':
                value = .8 if n < 600 else -.8
            elif name == 'ramp':
                value, measured = .2 * t, -.1 * t
            elif name == 'reference-sine':
                value = .8 * math.sin(2 * math.pi * 2 * t)
            elif name == 'measurement-sine':
                value, measured = .6 * math.sin(4 * math.pi * t), .4 * math.sin(4 * math.pi * t + .7)
            elif name == 'impulse':
                value = 1.5 if n == 300 else 0
            elif name == 'saturation-recovery':
                value = (8, -.1, -8, .1)[n // 300]
            elif name == 'jitter':
                dt = .0025 + (n % 17 - 8) * .000003
                value, measured = .8 * math.sin(4 * math.pi * t), .1 * math.cos(4 * math.pi * t)
            elif name == 'invalidation-recovery':
                measured = .2 * math.sin(3 * t)
                action = 3 if 300 <= n < 350 else 2 if n == 700 else 0
                if n in (900, 950):
                    dt = 0 if n == 900 else math.nan
            inputs.append(([value, -value * .7, value * .4], [measured, -measured * .5, measured * .3],
                           dt, (.03, .5, .97)[n // 400], action))
        append(name, inputs, combined)
    for cutoff in (5, 20, 40):
        for signal in ('measurement-step', 'measurement-ramp', 'reference-step'):
            inputs = []
            for n in range(800):
                target = [1 if signal == 'reference-step' and n >= 50 else 0, 0, 0]
                measured = [(.1 if n >= 50 else 0) if signal == 'measurement-step'
                            else n * .0025 * .1 if signal == 'measurement-ramp' else 0, 0, 0]
                inputs.append((target, measured, .0025 if n < 400 else .005, .5, 0))
            append(f'D-{signal}-{cutoff}Hz', inputs, [(0, 0, .02, 0, cutoff), (0, 0, 0, 0, 0), (0, 0, 0, 0, 0)])
    for sign, ki in itertools.product((-1, 1), (0, 1)):
        inputs = [([sign, 0, 0], [0, 0, 0], (.0025, .005)[n % 2], .5, 2 if n == 400 else 0)
                  for n in range(800)]
        append(f'I-sign{sign}-Ki{ki}', inputs, [(0, ki, 0, .03, 0)] * 3)
    for kp in (0, .1, .2):
        inputs = [([v, -2 * v, 3 * v], [0, 0, 0], .0025, .5, 0)
                  for v in [0, 1, -1, 0, 2, -2]]
        append(f'P-Kp{kp}', inputs, [(kp, 0, 0, 0, 0)] * 3)
    for frame in (12, 18):
        inputs = [(list(command), [0, 0, 0], .0025, thrust, 1)
                  for command in itertools.product((-1, -.75, -.5, -.25, 0, .25, .5, .75, 1), repeat=3)
                  for thrust in (0, .03, .5, .97, 1)]
        inputs += [(command, [0, 0, 0], .0025, thrust, 1)
                   for command, thrust in [([math.nan, 0, 0], .5), ([math.inf, 0, 0], .5),
                                           ([1.01, 0, 0], .5), ([0, 0, 0], -.01), ([0, 0, 0], math.nan)]]
        inputs += [(command, [0, 0, 0], .0025, thrust, 1)
                   for command, thrust in [([1 - 2 ** -24, 0, 0], .5), ([1 + 2 ** -23, 0, 0], .5),
                                           ([-1 - 2 ** -23, 0, 0], .5), ([1 - 2 ** -24] * 3, .5),
                                           ([2 ** -126, 0, 0], 0), ([0, 0, 0], 2 ** -149),
                                           ([0, 0, 0], 1 - 2 ** -24), ([0, 0, 0], 1 + 2 ** -23)]]
        inputs += [([0, 0, 0], [0, 0, 0], .0025, thrust, 1)
                   for thrust in (EPS, EPS * (1 - 2 ** -23), EPS * (1 + 2 ** -23),
                                  NEAR_ONE, NEAR_ONE - 2 ** -24, NEAR_ONE + 2 ** -24)]
        append(f'mixer-grid-frame{frame}', inputs, [(0, 0, 0, 0, 0)] * 3, frame)
    return rows, names


def write_vectors(rows, path):
    with path.open('x') as stream:
        for row in rows:
            values = [row[k] for k in ('scenario', 'sample', 'action', 'frame', 'dt', 'thrust')]
            values += row['target'] + row['measurement'] + list(itertools.chain.from_iterable(row['gains']))
            stream.write(' '.join(str(v) for v in values) + '\n')


def plot_svg(rows, results, path, scenario=8):
    selected = [(r, v) for r, v in zip(rows, results) if r['scenario'] == scenario]
    if not selected:
        return
    pid_curves = [('p0', '#cc3333'), ('i0', '#2266bb'), ('d0', '#339933'), ('out0', '#111111')]
    groups = [('Reference / measurement [rad/s]', [('reference', '#111111'), ('measurement', '#aaaaaa')]),
              ('Synthetic P / I / D / output [normalized]', pid_curves),
              ('Synthetic diagnostic motor allocation [0..1]', [(f'm{i}', c) for i, c in enumerate(
                  ('#cc3333', '#2266bb', '#339933', '#9944aa'))])]
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="800" viewBox="0 0 1000 800">',
           '<rect width="1000" height="800" fill="white"/>',
           '<text x="40" y="30" font-family="sans-serif" font-size="18">Synthetic inputs; no drone plant model</text>']
    for panel, (title, curves) in enumerate(groups):
        top = 75 + panel * 235
        series = {field: [row['target'][0] if field == 'reference' else row['measurement'][0]
                          if field == 'measurement' else result[field].value for row, result in selected]
                  for field, _ in curves}
        low, high = min(map(min, series.values())), max(map(max, series.values()))
        if high == low:
            high = low + 1
        svg.append(f'<text x="40" y="{top - 10}" font-family="sans-serif">{title}; range {low:.3g}..{high:.3g}</text>')
        svg.append(f'<path d="M 60 {top} V {top + 170} H 950" fill="none" stroke="#555"/>')
        for field, colour in curves:
            points = ' '.join(f'{60 + i * 890 / (len(selected) - 1):.2f},{top + 170 * (high - v) / (high - low):.2f}'
                              for i, v in enumerate(series[field]))
            svg.append(f'<polyline points="{points}" fill="none" stroke="{colour}" stroke-width="1.5"/>')
        legend = ' / '.join(field for field, _ in curves)
        svg.append(f'<text x="60" y="{top + 194}" font-family="sans-serif">0 .. 3 s; {legend}</text>')
    path.write_text('\n'.join(svg + ['</svg>']))


def validate(binary, output):
    output.mkdir(parents=True, exist_ok=False)
    rows, names = scenarios()
    write_vectors(rows, output / 'inputs.txt')
    env = os.environ.copy()
    env.update(UTB_MATH_INPUT=str((output / 'inputs.txt').resolve()), UTB_MATH_OUTPUT=str((output / 'cpp.csv').resolve()))
    with (output / 'cpp-test.log').open('x') as log:
        subprocess.run([str(binary.resolve())], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    models = {}
    expected = []
    maxima, normalized, failures = {}, {}, []
    with (output / 'cpp.csv').open() as stream:
        actual = list(csv.reader(stream))
    assert len(actual) == len(rows), (len(actual), len(rows))
    for row, values in zip(rows, actual):
        assert len(values) == len(FIELDS), (len(values), len(FIELDS))
        measured = dict(zip(FIELDS, map(float, values)))
        result = models.setdefault(row['scenario'], Reference()).step(row)
        expected.append(result)
        assert measured['scenario'] == row['scenario'] and measured['sample'] == row['sample']
        for field, value in result.items():
            if isinstance(value, Bound):
                # Nine significant decimal digits recover float32 exactly; compare recovered bits, not CSV text.
                error = abs(f32(measured[field]) - value.value)
                maxima[field] = max(maxima.get(field, 0), error)
                ratio = error / value.error if value.error else 0 if error == 0 else math.inf
                normalized[field] = max(normalized.get(field, 0), ratio)
                if error > value.error:
                    failures.append((names[row['scenario']], row['sample'], field,
                                     measured[field], value.value, value.error))
            elif measured[field] != value:
                failures.append((names[row['scenario']], row['sample'], field, measured[field], value, 'exact flag'))
    plot_svg(rows, expected, output / 'synthetic-pid-mixer.svg')
    plot_svg(rows, expected, output / 'synthetic-sine-pid-mixer.svg', scenario=6)
    report = dict(samples=len(rows), scenarios=names, max_absolute_error=maxima, max_error_over_bound=normalized,
                  failures=failures, float32_unit_roundoff=2 ** -24,
                  tolerance='Forward absolute error propagation; input float32 exact, gamma_n summation; '
                            'clipping nonexpansive')
    (output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
    with (output / 'reference.csv').open('x') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for row, result in zip(rows, expected):
            writer.writerow(dict(scenario=row['scenario'], sample=row['sample'],
                                 **{k: v.value if isinstance(v, Bound) else v for k, v in result.items()}))
    short_report = dict(samples=len(rows), scenarios=len(names), failures=len(failures), first_failures=failures[:10])
    print(json.dumps(short_report, indent=2))
    if failures:
        raise AssertionError('C++ versus independent mathematics mismatch; inspect comparison.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    validate(args.binary, args.output)


if __name__ == '__main__':
    main()
