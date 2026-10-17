#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Read archived H743 BIN offline; zero-gain infrastructure evidence, never hardware control."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_math_log_analysis.py
# @brief      Offline H743 PID/mixer recurrence a complete dataset analýza.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

from pymavlink import mavutil

from test_utb_math_sitl import packet_roundtrip
from utb_math_reference import Bound, allocate


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Distribution.
#
# @param[in]  values Values.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: dict, len, min, max.
# --------------------------------------------------------------------------------------------------------------------
def distribution(values):
    return dict(n=len(values), minimum=min(values), maximum=max(values), mean=statistics.mean(values)) if values else None


# --------------------------------------------------------------------------------------------------------------------
# @brief      Analyse.
#
# @param[in]  path Input or output path used by this operation.
# @param[in]  windows Windows.
# @return     Analysis result; assertions reject invalid or incomplete input evidence.
#
# @details    Global module function. Calls: path.resolve, mavutil.mavlink_connection, dict, ValueError.
# --------------------------------------------------------------------------------------------------------------------
def analyse(path, windows):
    path = path.resolve(strict=True)
    if not path.is_file() or path.suffix.lower() != '.bin':
        raise ValueError('Only an existing regular BIN file is accepted')
    reader = mavutil.mavlink_connection(str(path))
    packets, parameters = [], {}
    while True:
        message = reader.recv_match(type=['UTBC', 'PARM'])
        if message is None:
            break
        record = message.to_dict()
        if message.get_type() == 'UTBC':
            packets.append(record)
        else:
            parameters[record['Name']] = record['Value']
    gains = {name: value for name, value in parameters.items() if name.startswith('UTB_RAT_')}
    assert len(gains) == 15 and all(value == 0 for value in gains.values()), gains
    results = []
    for start, end in windows:
        selected = [p for p in packets if start <= p['TimeUS'] < end]
        assert selected
        decoded = [packet_roundtrip(p) for p in selected]
        assert all(r['Val'] for d in decoded for r in d[:4])
        assert all(math.isfinite(r[name]) for d in decoded for r in d[:3]
                   for name in ('Des', 'Meas', 'Err', 'P', 'I', 'D', 'Raw', 'Out', 'AP'))
        assert all(r[name] == 0 for d in decoded for r in d[:3] for name in ('P', 'I', 'D', 'Raw', 'Out'))
        motor_error = 0
        for d in decoded:
            expected = allocate([Bound(0)] * 3, d[3]['T'], 18)
            for i in range(4):
                error = abs(d[3][f'M{i + 1}'] - expected[f'm{i}'].value)
                assert error <= expected[f'm{i}'].error
                motor_error = max(motor_error, error)
        steps = {}
        offsets = []
        for i in range(1, len(selected)):
            difference = selected[i]['Seq'] - selected[i - 1]['Seq']
            steps[str(difference)] = steps.get(str(difference), 0) + 1
            if difference == 1:
                offsets.append(selected[i]['TimeUS'] - selected[i - 1]['TimeUS'] - decoded[i][5]['Dt'] * 1e6)
        results.append(dict(window=[start, end], complete_valid_sets=len(selected),
                            persisted_hz=len(selected) * 1e6 / (end - start), seq_steps=steps,
                            epochs=sorted({p['Epoch'] for p in selected}),
                            dt_seconds=distribution([d[5]['Dt'] for d in decoded]),
                            timestamp_delta_minus_dt_us=distribution(offsets),
                            capture_alignment_all=all(p['Cap'] and p['APS'] == p['Seq'] and p['APUS'] <= p['TimeUS']
                                                      and p['APM'] == p['Md'] for p in selected),
                            references_rads=[distribution([d[j]['Des'] for d in decoded]) for j in range(3)],
                            measured_rads=[distribution([d[j]['Meas'] for d in decoded]) for j in range(3)],
                            max_zero_gain_motor_error=motor_error))
    return dict(bin=str(path), size=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                gains=gains, windows=results, packed_payload_roundtrips=sum(r['complete_valid_sets'] for r in results),
                freshness='STATE_VALID accepted guards indirectly; raw sampled_us/INS timestamps/quaternion are not logged',
                rc_scaling='Rad/s fields finite and same units as source; raw RC-to-reference equivalence not reconstructed',
                nonzero_pid_hardware_verified=False)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, analyse.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bin', type=Path)
    parser.add_argument('--window', nargs=2, type=int, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = analyse(args.bin, args.window)
    with args.output.open('x') as stream:
        stream.write(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


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
