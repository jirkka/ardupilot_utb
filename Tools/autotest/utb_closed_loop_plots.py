#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Scientific SVG output with existing ReportLab; all stored samples retained."""
import argparse
import csv
import json
from pathlib import Path

from reportlab.graphics import renderSVG
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics.shapes import Drawing, Line, String
from reportlab.lib import colors

DISCLAIMER = 'SIMULATED – NOT HARDWARE VERIFIED'


def figure(path, title, panels):
    height = 90 + 220 * len(panels)
    drawing = Drawing(1100, height)
    drawing.add(String(65, height-27, DISCLAIMER, fontSize=16))
    drawing.add(String(65, height-50, title, fontSize=13))
    palette = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd']
    for number, (label, data, names, xlabel) in enumerate(panels):
        bottom = height-90-(number+1)*220
        plot = LinePlot()
        plot.x, plot.y, plot.width, plot.height = 100, bottom+42, 915, 145
        plot.joinedLines, plot.data = True, data
        values = [p[1] for series in data for p in series]
        low, high = min(values), max(values)
        span = high-low or .2*max(1, abs(low))
        plot.yValueAxis.valueMin, plot.yValueAxis.valueMax = low-.05*span, high+.05*span
        plot.yValueAxis.labelTextFormat = '%.4g'
        plot.xValueAxis.labels.fontSize = plot.yValueAxis.labels.fontSize = 8
        for n, name in enumerate(names):
            colour = colors.HexColor(palette[n % len(palette)])
            plot.lines[n].strokeColor, plot.lines[n].strokeWidth = colour, .8
            drawing.add(Line(110+n*210, bottom+199, 127+n*210, bottom+199, strokeColor=colour))
            drawing.add(String(135+n*210, bottom+196, name, fontSize=10))
        drawing.add(String(100, bottom+215, label, fontSize=10))
        drawing.add(plot)
        drawing.add(String(450, bottom+12, xlabel, fontSize=10))
    renderSVG.drawToFile(drawing, str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True, type=Path)
    root = parser.parse_args().directory
    for name in ['roll', 'yaw', 'axes', 'saturation', 'delay_30ms', 'noise', 'actuator_ceiling', 'one_motor_weak', 'sine_10']:
        with (root/(name+'.csv')).open() as stream:
            rows = list(csv.DictReader(stream))

        def series(field):
            return [(float(row['t']), float(row[field])) for row in rows]

        panels = [(axis+' [rad/s]', [series('r'+str(a)), series('w'+str(a))], ['reference', 'body rate'], 'Time [s]')
                  for a, axis in enumerate(['Roll', 'Pitch', 'Yaw'])]
        panels += [('Roll PID normalized', [series(k+'0') for k in ['p', 'i', 'd', 'raw']],
                    ['P', 'I', 'D', 'raw'], 'Time [s]'),
                   ('Model actuator states', [series('q'+str(a)) for a in range(4)], ['M1', 'M2', 'M3', 'M4'], 'Time [s]')]
        figure(root/(name+'.svg'), name+'; illustrative physical parameters', panels)
    stability = json.loads((root/'stability.json').read_text())
    values = stability['delay_sweep']
    data = [[(r['delay_s']*1000, r['radius'][a]) for r in values] for a in range(3)]
    data.append([(0, 1), (values[-1]['delay_s']*1000, 1)])
    figure(root/'delay-poles.svg', 'Local unsaturated discrete poles; sensor delay in whole 400Hz cycles',
           [('Spectral radius', data, ['Roll', 'Pitch', 'Yaw', 'unit circle'], 'Sensor delay [ms]')])
    print('Ten SVG figures written, every stored sample retained')


if __name__ == '__main__':
    main()
