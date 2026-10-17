#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Reproduce 1H-E plans, mock timelines, archived replay and pendulum demo offline."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_1he_offline.py
# @brief      Reprodukce 1H-E plánů, mock timelines, archivního replay a demonstračního kyvadla.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import statistics
import sys

from utb_download_experiment import Engine, Journal, replay_archive, validate_plan
from utb_plant_identification import bifilar, pendulum_period

# Reuse the tested mock peer; this module has no live-device entry point.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tests'))
from test_utb_download_experiment import IDENTITY, MockClock, Peer, plan  # noqa: E402


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
ARCHIVED_LOGS = [
    dict(id=2, size=2569496, time_utc=1791811304,
         archive_sha256='d45167bd5832efb08cf2b0350fd1a9cd29388a48828aa4ffa0b2bd0bc63f040e'),
    dict(id=3, size=4349246, time_utc=1791812260,
         archive_sha256='0677617bfac02af3f3b44d0038bb614ed147e1eeb3812d0b96b80d6e565786b6')]


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Write json.
#
# @param[in]  path Input or output path used by this operation.
# @param[in]  data Data.
# @return     None.
#
# @details    Global module function. Calls: path.write_text, json.dumps.
# --------------------------------------------------------------------------------------------------------------------
def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


# --------------------------------------------------------------------------------------------------------------------
# @brief      Prepare plans.
#
# @param[in]  output Explicit destination for new experiment results.
# @return     Returned values: plans.
#
# @details    Global module function. Calls: output.mkdir, write_json, dict, validate_plan.
# --------------------------------------------------------------------------------------------------------------------
def prepare_plans(output):
    output.mkdir()
    cases = [('idle', 'idle', 1, 0, None), ('list', 'list', 1, 0, None)]
    for log in ARCHIVED_LOGS:
        cases += [(f"log{log['id']}-one", 'data', 1, 0, log)]
        cases += [(f"log{log['id']}-paced{int(pacing*1000)}", 'paced', 4, pacing, log) for pacing in [.02, .1]]
    plans = {}
    for name, scenario, chunks, pacing, log in cases:
        p = dict(scenario=scenario, identity=copy.deepcopy(IDENTITY), baseline_s=60, recovery_s=20,
                 duration_limit_s=100, chunk_bytes=8190, chunks=chunks, pacing_s=pacing,
                 request_timeout_s=2, retries=0, log=log)
        plans[name] = validate_plan(p)
        write_json(output/(name+'.json'), p)
    write_json(output/'approval-NOT-GRANTED.json', dict(approved=False, props_removed=False, exclusive_port=False,
               port='COM5', expires_utc='2000-01-01T00:00:00+00:00', plan_sha256='NOT_APPROVED'))
    return plans


# --------------------------------------------------------------------------------------------------------------------
# @brief      Pendulum demo.
#
# @param[in]  output Explicit destination for new experiment results.
# @return     None.
#
# @details    Global module function. Calls: output.mkdir, dict, bifilar, random.Random.
# --------------------------------------------------------------------------------------------------------------------
def pendulum_demo(output):
    output.mkdir()
    period = .9
    times = [i*.005 for i in range(3000)]
    angles = [.06*math.exp(-.02*t)*math.sin(2*math.pi*t/period+.2) for t in times]
    with (output/'DEMO_pendulum.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'angle_rad'])
        writer.writerows(zip(times, angles))
    values = dict(drone_mass=.65, fixture_mass=.05, top_half_spacing=.1, bottom_half_spacing=.1,
                  vertical_height=.6, period=period, fixture_inertia=.0002, cg_offset=.003,
                  damping_ratio=.02/(2*math.pi/period))
    uncertainty = dict(drone_mass=.001, fixture_mass=.001, top_half_spacing=.0005, bottom_half_spacing=.0005,
                       vertical_height=.001, period=.001, fixture_inertia=.000005, cg_offset=.0005, damping_ratio=.0001)
    result = bifilar(**values, uncertainties=uncertainty)
    rng = random.Random(1729)
    samples = [bifilar(**{k: v+rng.gauss(0, uncertainty.get(k, 0)) for k, v in values.items()})['inertia_kgm2']
               for _ in range(20000)]
    result.update(disclaimer='DEMONSTRATION – SYNTHETIC PENDULUM; NOT SKYSTARS MEASUREMENT', inputs=values,
                  standard_uncertainties=uncertainty, period_reduction=pendulum_period(times, angles),
                  monte_carlo=dict(samples=len(samples), seed=1729, mean=statistics.mean(samples),
                                   std=statistics.stdev(samples),
                                   std_ratio_to_first_order=statistics.stdev(samples)/result['standard_uncertainty_kgm2']))
    write_json(output/'DEMO_inertia_reduction.json', result)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Run.
#
# @param[in]  output Explicit destination for new experiment results.
# @param[in]  archive Archive.
# @return     None.
#
# @details    Global module function. Calls: any, output.mkdir, prepare_plans, pendulum_demo.
# --------------------------------------------------------------------------------------------------------------------
def run(output, archive):
    expected = ['full-download.tlog', 'short-download.tlog', 'final-coldboot-readback.tlog']
    if any(not (archive/name).is_file() for name in expected):
        raise ValueError('Three existing tlog archives required; no device names accepted')
    before = {name: hashlib.sha256((archive/name).read_bytes()).hexdigest() for name in expected}
    output.mkdir(parents=True, exist_ok=False)
    plans = prepare_plans(output/'plans')
    results = {}
    for name, scenario in [('idle', 'idle'), ('list', 'list'), ('one', 'data'), ('paced', 'paced'),
                           ('timeout', 'data'), ('ekf-unhealthy', 'data'), ('loop-spike', 'paced')]:
        p = plan(scenario, chunks=3 if scenario == 'paced' else 1)
        clock = MockClock()
        peer = Peer(clock)
        peer.behaviour = {'timeout': 'timeout', 'ekf-unhealthy': 'ekf', 'loop-spike': 'loop-spike'}.get(name, 'normal')
        results[name] = Engine(peer, clock, p, Journal(clock, output/'mock'/name)).run()
    for name in expected:
        result = replay_archive(archive/name, plans['idle'], output/'archive-replay'/Path(name).stem)
        results[name] = {k: result[k] for k in ['messages', 'transmitted_messages', 'guard_observations', 'sha256']}
    pendulum_demo(output/'measurement-demo')
    after = {name: hashlib.sha256((archive/name).read_bytes()).hexdigest() for name in expected}
    if before != after:
        raise AssertionError('Archive changed during offline analysis')
    write_json(output/'offline-results.json', dict(results=results, archive_hashes=after, archive_unchanged=True,
               actual_hardware_access=False, disclaimer='Mock experiments and archived telemetry only'))
    print('Seven mock scenarios, three archived replays, eight plans and synthetic pendulum; no device access')


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
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--archive-root', type=Path, required=True)
    args = parser.parse_args()
    run(args.output, args.archive_root)


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
# ====================================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------------
# None
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Private functions
