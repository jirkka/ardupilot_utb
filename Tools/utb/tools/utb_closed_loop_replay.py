#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Replay simulated measurements through the existing C++ PID/mixer unit runner."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_closed_loop_replay.py
# @brief      Replay offline model vstupů proti skutečnému C++ test runneru.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess

from utb_math_reference import Bound, Reference, f32
from utb_math_validation import FIELDS


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse vectors.
#
# @param[in]  path Input or output path used by this operation.
# @return     Iterator over the generated records.
#
# @details    Global module function. Calls: path.read_text().splitlines, list, path.read_text, map.
# --------------------------------------------------------------------------------------------------------------------
def parse_vectors(path):
    for line in path.read_text().splitlines():
        v = list(map(float, line.split()))
        if len(v) != 27:
            raise ValueError('Incomplete replay vector')
        yield dict(scenario=int(v[0]), sample=int(v[1]), action=int(v[2]), frame=int(v[3]), dt=v[4], thrust=v[5],
                   target=v[6:9], measurement=v[9:12], gains=[tuple(v[a:a+5]) for a in [12, 17, 22]])


# --------------------------------------------------------------------------------------------------------------------
# @brief      Replay.
#
# @param[in]  binary Explicit local SITL executable.
# @param[in]  inputs Inputs.
# @param[in]  output Explicit destination for new experiment results.
# @return     None.
#
# @details    Global module function. Calls: output.mkdir, dict, (output / 'comparison.json').write_text, print.
# --------------------------------------------------------------------------------------------------------------------
def replay(binary, inputs, output):
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, UTB_MATH_INPUT=str(inputs.resolve()), UTB_MATH_OUTPUT=str((output/'cpp.csv').resolve()))
    with (output/'cpp-test.log').open('x') as stream:
        subprocess.run([str(binary.resolve())], env=env, stdout=stream, stderr=subprocess.STDOUT, check=True)
    models, maxima, failures, count = {}, {}, [], 0
    with (output/'cpp.csv').open() as stream:
        for row, actual in itertools.zip_longest(parse_vectors(inputs), csv.reader(stream)):
            if row is None or actual is None or len(actual) != len(FIELDS):
                raise AssertionError('Missing/extra C++ rows or fields')
            measured = dict(zip(FIELDS, map(float, actual)))
            expected = models.setdefault(row['scenario'], Reference()).step(row)
            if measured['scenario'] != row['scenario'] or measured['sample'] != row['sample']:
                raise AssertionError('Sample identity mismatch')
            for field, value in expected.items():
                if isinstance(value, Bound):
                    error = abs(f32(measured[field])-value.value)
                    ratio = error/value.error if value.error else 0 if error == 0 else math.inf
                    maxima[field] = max(maxima.get(field, 0.), error)
                    if ratio > 1:
                        failures.append([row['scenario'], row['sample'], field, error, value.error])
                elif measured[field] != value:
                    failures.append([row['scenario'], row['sample'], field, measured[field], value])
            count += 1
    report = dict(samples=count, scenarios=len(models), failures=failures, max_absolute_error=maxima,
                  tolerance='Existing forward float32 roundoff bound; exact flags',
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  inputs_sha256=hashlib.sha256(inputs.read_bytes()).hexdigest())
    (output/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('C++ closed-loop replay:', count, 'samples,', len(failures), 'mismatches')
    if failures:
        raise AssertionError('C++ parity regression')


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, replay.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    replay(args.binary, args.inputs, args.output)


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
