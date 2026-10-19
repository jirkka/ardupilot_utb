#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file utb_native_bench_analysis.py
# @author Jiri Kovar
# @version 2026-10-19
"""Decode separate AP/UTB3A diagnostic instances; never opens a serial endpoint."""
import argparse
import csv
import json
import math
from pathlib import Path

from pymavlink import mavutil

KINDS = {'U3BD', 'U3BF', 'U3BA', 'U3BQ', 'U3BT'}
PAIRS = [('AT', 'UT'), ('AP', 'UP'), ('AI', 'UI'), ('AD', 'UD'),
         ('AF', 'UF'), ('ADF', 'UDF'), ('AS', 'US'), ('AO', 'UO'),
         ('AMF', 'UMF'), ('ATot', 'UTot')]


def analyse(path, output, platform):
    """Preserve all rows; report incomplete/nonidentical/unhealthy samples separately."""
    if not path.is_file() or path.suffix.lower() != '.bin':
        raise ValueError('Only an existing local BIN file is accepted; no device or network endpoint')
    reader = mavutil.mavlink_connection(str(path.resolve()))
    sets, pm, dsf = {}, [], []
    while (message := reader.recv_match()) is not None:
        kind = message.get_type()
        if kind == 'PM':
            pm.append(message.to_dict())
        if kind == 'DSF':
            dsf.append(message.to_dict())
        if kind not in KINDS:
            continue
        row = message.to_dict()
        key = (row['TimeUS'], row['N'])
        subkey = kind + (str(row['Ax']) if kind in {'U3BD', 'U3BF'} else '')
        record = sets.setdefault(key, {})
        if subkey in record:
            raise ValueError(f'Duplicate diagnostic packet {key}/{subkey}')
        record[subkey] = row
    reader.close()
    required = {'U3BD0', 'U3BD1', 'U3BD2', 'U3BF0', 'U3BF1', 'U3BF2', 'U3BA', 'U3BQ', 'U3BT'}
    complete = [(key, record) for key, record in sorted(sets.items()) if set(record) == required]
    if len({key[1] for key, _ in complete}) != len(complete):
        raise ValueError('Multiple RUN sessions in one log; isolate one experiment before comparison')
    rows, errors, invalid, excluded, periods = [], {}, 0, 0, []
    tainted_phases = set()
    unequal_axes, unequal_sets, invalid_axes, invalid_sets = set(), set(), set(), set()
    for (time_us, seq), record in complete:
        timing = record['U3BT']
        assert timing['Ver'] == 2, 'Unknown fixture version'
        flags = timing['Flags']
        phase = record['U3BD0']['Ph']
        if flags & (8 | 16):
            tainted_phases.add(phase)
        if phase in tainted_phases:
            flags |= 8  # Preserve possible state divergence until the next profile reset.
        excluded += bool(flags & (8 | 16))
        if seq > 0:
            periods.append(timing['Wall'])
        for axis in range(3):
            row = dict(time_us=time_us, seq=seq, axis=axis, platform=platform,
                       ap_us=timing['APus'], utb_us=timing['UTus'], period_us=timing['Wall'],
                       previous_export_us=timing['LogUs'], flags=flags, raw_flags=timing['Flags'])
            row.update({k: v for k, v in record[f'U3BD{axis}'].items() if k != 'mavpackettype'})
            row.update({k: v for k, v in record[f'U3BF{axis}'].items() if k != 'mavpackettype'})
            row.update({'rate_'+k: v for k, v in record['U3BA'].items() if k != 'mavpackettype'})
            row.update({'quat_'+k: v for k, v in record['U3BQ'].items() if k != 'mavpackettype'})
            row['command'] = row['Ref'] * (.1 if row['Ph'] >= 2 and axis < 2 else 1)
            for ap, utb in PAIRS:
                a, u = row[ap], row[utb]
                if not math.isfinite(a) or not math.isfinite(u):
                    invalid += int(not math.isfinite(a)) + int(not math.isfinite(u))
                    invalid_axes.add((time_us, seq, axis))
                    invalid_sets.add((time_us, seq))
                    continue
                if not flags & (8 | 16):
                    errors.setdefault(f'phase{row["Ph"]}/{ap}-{utb}', []).append(a-u)
                    if a != u:
                        unequal_axes.add((time_us, seq, axis))
                        unequal_sets.add((time_us, seq))
            rows.append(row)
        for field_a, field_u in [('AR', 'UR'), ('AP', 'UP'), ('AY', 'UY')]:
            a, u = record['U3BA'][field_a], record['U3BA'][field_u]
            if math.isfinite(a) and math.isfinite(u) and not flags & (8 | 16):
                errors.setdefault(f'rate_target/{field_a}-{field_u}', []).append(a-u)
                if a != u:
                    unequal_sets.add((time_us, seq))
            elif not math.isfinite(a) or not math.isfinite(u):
                invalid += int(not math.isfinite(a)) + int(not math.isfinite(u))
                invalid_sets.add((time_us, seq))
        for suffix in 'WXYZ':
            a, u = record['U3BQ']['A'+suffix], record['U3BQ']['U'+suffix]
            if math.isfinite(a) and math.isfinite(u) and not flags & (8 | 16):
                errors.setdefault('quaternion/'+suffix, []).append(a-u)
                if a != u:
                    unequal_sets.add((time_us, seq))
            elif not math.isfinite(a) or not math.isfinite(u):
                invalid += int(not math.isfinite(a)) + int(not math.isfinite(u))
                invalid_sets.add((time_us, seq))
    if not rows:
        raise ValueError('No complete native benchmark sets; no comparison can be claimed')
    output.mkdir(parents=True, exist_ok=False)
    with (output/'comparison.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    stats = {field: dict(samples=len(values), max_abs=max(map(abs, values)),
                         rmse=math.sqrt(sum(v*v for v in values)/len(values)),
                         unequal=sum(v != 0 for v in values)) for field, values in errors.items()}
    # No invented PASS threshold: exact differences and sample completeness are reported.
    summary = dict(platform=platform, complete_sets=len(complete), partial_sets=len(sets)-len(complete),
                   excluded_input_or_health_sets=excluded, nonfinite_values=invalid,
                   observed_hz=(len(complete)-1)*1e6/(complete[-1][0][0]-complete[0][0][0])
                   if len(complete) > 1 else None,
                   missing_sequence_numbers=sorted(set(range(2000))-{key[1] for key, _ in complete}),
                   unequal_axis_samples=len(unequal_axes), unequal_sets=len(unequal_sets),
                   invalid_axis_samples=len(invalid_axes), invalid_sets=len(invalid_sets),
                   error=stats, ap_math_us_max=max(r['ap_us'] for r in rows),
                   utb_math_us_max=max(r['utb_us'] for r in rows),
                   previous_export_us_max=max(r['previous_export_us'] for r in rows),
                   period_us_min=min(periods) if periods else None, period_us_max=max(periods) if periods else None,
                   period_us_mean=sum(periods)/len(periods) if periods else None,
                   period_abs_jitter_us_mean=sum(abs(v-20000) for v in periods)/len(periods) if periods else None,
                   nominal_diagnostic_period_us=20000, pm=pm, dsf=dsf,
                   limitations=['Independent instances, no active-loop or motor equivalence claim',
                                'SMAX=0; clock-driven slew limiter excluded',
                                'observed_hz counts stored complete sets; Wall measures calculation update periods',
                                'First Wall=0 is a startup sentinel, excluded from period statistics',
                                'Input/health flags taint the profile until its next reset; raw_flags are preserved',
                                'DSF drops describe all logging, not only this diagnostic',
                                'Synthetic fixture uses model dt; phase3 uses wall dt',
                                'LogUs describes the preceding sample export; final export is not observed',
                                'Initial dynamic-format registration contributes to export cost'])
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--platform', choices=['SITL', 'H743', 'unknown'], default='unknown')
    args = parser.parse_args()
    print(json.dumps(analyse(args.bin, args.output, args.platform), indent=2))
