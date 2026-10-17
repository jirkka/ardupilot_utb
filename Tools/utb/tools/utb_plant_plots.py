#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Reuse the existing scientific plotter for illustrative calibration/sensitivity."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_plant_plots.py
# @brief      Grafy demonstrační kalibrace a sensitivity; využívá existující plotter.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import csv
from pathlib import Path

from utb_closed_loop_plots import figure


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Read.
#
# @param[in]  path Input or output path used by this operation.
# @return     Returned values: list(csv.DictReader(stream)).
#
# @details    Global module function. Calls: path.open, list, csv.DictReader.
# --------------------------------------------------------------------------------------------------------------------
def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


# --------------------------------------------------------------------------------------------------------------------
# @brief      Series.
#
# @param[in]  rows Exported sample records.
# @param[in]  key Key.
# @return     Returned values: [(float(row['t']), float(row[key])) for row in rows].
#
# @details    Global module function. Calls: float.
# --------------------------------------------------------------------------------------------------------------------
def series(rows, key):
    return [(float(row['t']), float(row[key])) for row in rows]


# --------------------------------------------------------------------------------------------------------------------
# @brief      Run.
#
# @param[in]  root Root.
# @param[in]  baseline Baseline.
# @return     None.
#
# @details    Global module function. Calls: output.mkdir, print, figure, read.
# --------------------------------------------------------------------------------------------------------------------
def run(root, baseline):
    output = root/'figures'
    output.mkdir(exist_ok=False)
    for case in ['roll', 'yaw', 'axes']:
        old, calibrated = read(baseline/(case+'.csv')), read(root/'demo-calibration'/(case+'.csv'))
        panels = [(axis+' [rad/s]', [series(old, 'r'+str(a)), series(old, 'w'+str(a)), series(calibrated, 'w'+str(a))],
                   ['reference', 'original baseline', 'demo import'], 'Time [s]')
                  for a, axis in enumerate(['Roll', 'Pitch', 'Yaw'])]
        figure(output/(case+'-calibration.svg'), case+'; demonstration input, not measured Skystars', panels)
    groups = [(['baseline', 'Jxx_0.9', 'Jxx_1.1'], 'inertia'),
              (['baseline', 'motor_tau_0.75', 'motor_tau_1.25'], 'motor-lag'),
              (['baseline', 'max_torque_0.8', 'max_torque_1.2'], 'torque'),
              (['baseline', 'voltage_0.9', 'voltage_1.1'], 'voltage'),
              (['baseline', 'damping_0.5', 'damping_1.5'], 'damping'),
              (['baseline', 'sensor_2p5ms', 'actuator_2p5ms', 'common_scale_1p5'], 'identifiability')]
    for names, label in groups:
        rows = [read(root/'sensitivity'/(name+'.csv')) for name in names]
        key = 'meas' if label == 'identifiability' else 'w'
        panels = [(axis+' [rad/s]', [series(r, key+str(a)) for r in rows], names, 'Time [s]')
                  for a, axis in enumerate(['Roll', 'Pitch', 'Yaw'])]
        figure(output/(label+'.svg'), label+'; illustrative uncertainty, fixed PID, no hardware data', panels)
    print('Nine SVG figures; all samples kept; reuse original plot implementation')


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
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    args = parser.parse_args()
    run(args.directory, args.baseline)


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
