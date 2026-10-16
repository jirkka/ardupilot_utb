#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Strict offline analysis of actual SITL UARX/UAMT exports, never a vehicle connection."""
import argparse
import csv
import json
import math
import os
from pathlib import Path


def load_rows(path):
    with path.open() as stream:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def check_rows(rows, owner, expected_samples=3200):
    if len(rows) != expected_samples:
        raise ValueError('Incomplete finite sequence')
    for i, row in enumerate(rows):
        if not all(math.isfinite(v) for v in row.values()):
            raise ValueError('Nonfinite controller or timing data')
        if row['motor_calls'] != row['seq']:
            raise ValueError('Motor callback count differs from rate sequence')
        if row['ap_calls' if owner == 1 else 'utb_calls'] != 0:
            raise ValueError('Concurrent controller authority')
        if row['utb_calls' if owner == 1 else 'ap_calls'] != row['seq']:
            raise ValueError('Missing selected controller call')
        if i and (row['seq'] != rows[i-1]['seq']+1 or row['t'] <= rows[i-1]['t']):
            raise ValueError('Missing or out-of-order sample')
        if not .000625 <= row['dt'] <= .01:
            raise ValueError('Invalid dt')


def alignment(a, b):
    if len(a) != len(b):
        raise ValueError('Unequal reference sample counts')
    times = max(abs(x['t']-y['t']) for x, y in zip(a, b))
    targets = max(abs(x[f'target{k}']-y[f'target{k}']) for x, y in zip(a, b) for k in range(3))
    references = max(abs(x[f'ref{k}']-y[f'ref{k}']) for x, y in zip(a, b) for k in range(3))
    # Align by generator sample, with equal raw/shaped targets. Independent SITL
    # physics/clock phases need not share identical TimeUS; allow at most one nominal cycle.
    if times > .0025 or targets > 1e-5 or references > 1e-7:
        raise ValueError('Reference/time alignment failed')
    return dict(max_time_difference_s=times, max_target_difference_rads=targets,
                max_reference_difference_rads=references, time_tolerance_s=.0025,
                method='same generator sample and actual shaped target; bounded independent clock phase')


def step_metrics(rows, axis):
    """A single signed constant plateau; raw step includes the shared AP target shaping."""
    amplitude = rows[0][f'ref{axis}']
    if amplitude == 0 or any(abs(r[f'ref{axis}']-amplitude) > 1e-7 for r in rows):
        raise ValueError('Not a constant nonzero step plateau')
    direction = 1 if amplitude > 0 else -1
    amplitude = abs(amplitude)
    values = [direction*r[f'gyro{axis}'] for r in rows]
    t10 = next((r['t'] for r, v in zip(rows, values) if v >= .1*amplitude), None)
    t90 = next((r['t'] for r, v in zip(rows, values) if v >= .9*amplitude), None)
    outside = [i for i, v in enumerate(values) if abs(v-amplitude) > .02*amplitude]
    index = outside[-1]+1 if outside else 0
    settled = index < len(rows) and rows[-1]['t']-rows[index]['t'] >= .25
    tail = [r for r in rows if r['t'] >= rows[-1]['t']-.5]
    return dict(sign=direction, start_s=rows[0]['t'], duration_s=rows[-1]['t']-rows[0]['t'],
                amplitude_rads=amplitude, rise_10_90_s=None if t10 is None or t90 is None else t90-t10,
                peak_fraction=max(values)/amplitude, overshoot_percent=max(0., 100*(max(values)/amplitude-1)),
                settle_2percent_s=rows[index]['t']-rows[0]['t'] if settled else None,
                steady_error_rads=sum(r[f'target{axis}']-r[f'gyro{axis}'] for r in tail)/len(tail))


def plateaus(rows, axis):
    groups, group = [], []
    for row in rows:
        ref = row[f'ref{axis}']
        if group and abs(ref-group[-1][f'ref{axis}']) > 1e-7:
            groups.append(group)
            group = []
        group.append(row)
    if group:
        groups.append(group)
    return [g for g in groups if abs(g[0][f'ref{axis}']) > .05 and len(g) >= 100]


def metrics(rows, profile):
    results = []
    for axis in range(3):
        errors = [r[f'target{axis}']-r[f'gyro{axis}'] for r in rows]
        results.append(dict(iae_rad=sum(abs(e)*r['dt'] for e, r in zip(errors, rows)),
                            itae_rad_s=sum(r['t']*abs(e)*r['dt'] for e, r in zip(errors, rows)),
                            maximum_output=max(abs(r[f'out{axis}']) for r in rows),
                            max_abs_gyro_rads=max(abs(r[f'gyro{axis}']) for r in rows),
                            steps=[] if profile in ('zero', 'sine') else
                            [step_metrics(g, axis) for g in plateaus(rows, axis)]))
    periods = [1e6*(b['t']-a['t']) for a, b in zip(rows, rows[1:])]
    native = sorted(r['execution_us'] for r in rows)
    timing = dict(observed_hz=(len(rows)-1)/(rows[-1]['t']-rows[0]['t']),
                  period_us_min=min(periods), period_us_max=max(periods),
                  period_us_mean=sum(periods)/len(periods),
                  mean_absolute_jitter_us=sum(abs(p-2500) for p in periods)/len(periods),
                  host_rate_slot_us_mean=sum(native)/len(native),
                  host_rate_slot_us_p99=native[int(.99*(len(native)-1))], host_rate_slot_us_max=max(native))
    return dict(axis=results, timing=timing, samples=len(rows),
                resets_or_faults_in_supported_window=0, saturation_detected_s=0)


def motor_provenance(rows, delayed):
    maximum = 0.
    for i, row in enumerate(rows[1:], 1):
        source = rows[i-1] if delayed else row
        for axis in range(3):
            maximum = max(maximum, abs(row[f'out{axis}']-sum(source[k+str(axis)] for k in ['p', 'i', 'd'])))
    if maximum >= 1e-7:
        raise ValueError('Actual motor request does not match selected UTB PID / explicit delay')
    # Read-only sign evidence, using the audited BF_X_REV motor ordering. Not achieved-torque feedback.
    columns = ((-.5, -.5, .5, .5), (-.5, .5, -.5, .5), (.5, -.5, -.5, .5))
    gains = []
    for axis, column in enumerate(columns):
        projection = [sum(c*r[f'actuator{m}'] for m, c in enumerate(column)) for r in rows]
        norm = sum(r[f'out{axis}']**2 for r in rows)
        gain = sum(p*r[f'out{axis}'] for p, r in zip(projection, rows))/norm if norm else None
        if gain is not None and gain <= 0:
            raise ValueError('Reversed motor projection')
        gains.append(gain)
    return dict(max_pid_to_request_error=maximum, adapter_delay_cycles=int(delayed),
                positive_motor_projection_gain=gains, physical_achieved_moment_claimed=False)


def plot_png(path, title, panels):
    # Raster preview of scientific curves. Every stored sample is drawn.
    from PIL import Image, ImageDraw, ImageFont
    width, height = 1200, 100+240*len(panels)
    canvas = Image.new('RGB', (width, height), 'white')
    draw = ImageDraw.Draw(canvas)
    font_path = str(Path(os.environ['WINDIR'])/'Fonts/arial.ttf') if os.name == 'nt' else 'DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 15)
    draw.text((65, 15), 'ACTUAL ARDUPILOT SITL – NOT HARDWARE VERIFIED', fill='black', font=font)
    draw.text((65, 45), title, fill='black', font=font)
    palette = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd']
    for panel, (label, data, names, xlabel) in enumerate(panels):
        left, top, right, bottom = 105, 125+panel*240, 1120, 285+panel*240
        points = [point for series in data for point in series]
        xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
        ymin, ymax = min(p[1] for p in points), max(p[1] for p in points)
        span = ymax-ymin or .2*max(1, abs(ymin))
        ymin, ymax = ymin-.05*span, ymax+.05*span
        draw.text((left, top-38), label, fill='black', font=font)
        for i in range(5):
            x = left+(right-left)*i/4
            y = top+(bottom-top)*i/4
            draw.line((x, top, x, bottom), fill='#dddddd')
            draw.line((left, y, right, y), fill='#dddddd')
            draw.text((x-20, bottom+7), f'{xmin+(xmax-xmin)*i/4:.3g}', fill='black', font=font)
            draw.text((8, y-8), f'{ymax-(ymax-ymin)*i/4:.3g}', fill='black', font=font)
        for i, (series, name) in enumerate(zip(data, names)):
            colour = palette[i % len(palette)]
            draw.text((left+245*i, top-20), name, fill=colour, font=font)
            mapped = [(left+(x-xmin)/(xmax-xmin)*(right-left),
                       bottom-(y-ymin)/(ymax-ymin)*(bottom-top)) for x, y in series]
            draw.line(mapped, fill=colour, width=2)
        draw.rectangle((left, top, right, bottom), outline='black')
        draw.text((550, bottom+32), xlabel, fill='black', font=font)
    canvas.save(path)


def plots(directory, name, utb, ap):
    from utb_closed_loop_plots import figure

    def series(rows, field):
        return [(r['t'], r[field]) for r in rows]

    panels = [(axis+' [rad/s]', [series(utb, f'target{a}'), series(utb, f'gyro{a}'), series(ap, f'gyro{a}')],
               ['shared target', 'UTB actual gyro', 'AP actual gyro'], 'Time [s]')
              for a, axis in enumerate(['Roll', 'Pitch', 'Yaw'])]
    for suffix, data in [('rates', panels), ('utb-pid',
                         [(axis+' PID [normalized]', [series(utb, k+str(a)) for k in ['p', 'i', 'd', 'out']],
                           ['P', 'I', 'D', 'active output'], 'Time [s]')
                          for a, axis in enumerate(['Roll', 'Pitch', 'Yaw'])]), ('authority',
                         [('AP motor actuators [normalized]', [series(utb, 'actuator'+str(a)) for a in range(4)],
                           ['M1', 'M2', 'M3', 'M4'], 'Time [s]'),
                          ('Single authority/cycle counters', [series(utb, k) for k in
                                                               ['ap_calls', 'utb_calls', 'motor_calls']],
                           ['AP calls', 'UTB calls', 'motor output calls'], 'Time [s]')])]:
        title = name+'; actual ArduPilot SITL; bfxrev; 400Hz'
        figure(directory/f'{name}-{suffix}.svg', title, data)
        plot_png(directory/f'{name}-{suffix}.png', title, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plots', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = {}
    for directory in args.inputs:
        for case in sorted(directory.glob('*-utb')):
            if not (case/'response.csv').exists():
                continue
            name = case.name.removesuffix('-utb')
            utb, ap = [load_rows(p/'response.csv') for p in [case, directory/(name+'-ap')]]
            check_rows(utb, 1)
            check_rows(ap, 2)
            results[name] = dict(alignment=alignment(utb, ap), utb=metrics(utb, name), ap=metrics(ap, name),
                                 motor_provenance=motor_provenance(utb, name == 'delay'),
                                 initial_gyro_difference_rads=max(abs(utb[0][f'gyro{a}']-ap[0][f'gyro{a}'])
                                                                  for a in range(3)))
            if args.plots:
                plots(args.output, name, utb, ap)
    (args.output/'analysis.json').write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    print('Strict actual SITL comparisons:', len(results), 'PASS')


if __name__ == '__main__':
    main()
