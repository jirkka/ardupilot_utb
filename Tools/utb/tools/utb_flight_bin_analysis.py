#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file utb_flight_bin_analysis.py
# @brief Reconstruct source-identified seven-packet sets; backend admission is not persistence.
# @author Jiri Kovar
# @version 2026-10-26
"""Analyze one native Flight BIN session. Reject ambiguous multiple sessions instead of merging them."""
import argparse
import csv
import json
import math
from pathlib import Path

from pymavlink import mavutil

NAMES = ('UFPI', 'UFRS', 'UFAT', 'UFLS')
PID_PAIRS = (('AP', 'UP'), ('AI', 'UI'), ('AD', 'UD'), ('AFF', 'UFF'), ('ADF', 'UDF'))
RATE_PAIRS = (('AT', 'UT'), ('AO', 'UO'), ('AF', 'UF'))


def analyze(records):
    """Group by captured MCU timestamp AND sequence, never by row order. Keep all faults visible."""
    groups, statuses, errors, rows, diffs = {}, [], [], [], {}
    duplicates = 0
    for r in records:
        kind = r['mavpackettype']
        if kind == 'UFLS':
            statuses.append(r)
            continue
        if kind not in NAMES:
            continue
        key = (r['TimeUS'], r['Seq'])
        group = groups.setdefault(key, {})
        slot = (kind, r.get('Ax', 0))
        if slot in group:
            duplicates += 1
        group[slot] = r
    starts = {r['Start'] for r in statuses}
    if len(starts) > 1:
        errors.append('Multiple sessions: split BIN by boot/session before analysis')
    if any(r['V'] != 1 for r in statuses):
        errors.append('Unsupported wire version')
    expected = {('UFPI', a) for a in range(3)} | {('UFRS', a) for a in range(3)} | {('UFAT', 0)}
    partial, invalid = 0, 0
    seqs = []
    for (timestamp, seq), group in sorted(groups.items()):
        seqs.append(seq)
        if set(group) != expected:
            partial += 1
            continue
        bad = False
        row = dict(TimeUS=timestamp, Seq=seq)
        rate0, att = group[('UFRS', 0)], group[('UFAT', 0)]
        for axis in range(3):
            pid, rate = group[('UFPI', axis)], group[('UFRS', axis)]
            bad |= pid['Md'] != 0 or rate['Md'] != 0 or pid['Flg'] & 3 != 3 or rate['Flg'] != pid['Flg']
            bad |= ((rate['Src'] + 1) & 0xffffffff) != rate['Lp']
            bad |= rate['SrcUS'] > timestamp or timestamp - rate['SrcUS'] > 100000
            bad |= any(rate[k] != rate0[k] for k in ('Lp', 'Src', 'SrcUS', 'Dt'))
            bad |= not math.isfinite(rate['Dt']) or rate['Dt'] <= 0
            for packet, pairs in ((pid, PID_PAIRS), (rate, RATE_PAIRS)):
                for ap, utb in pairs:
                    name = f'{axis}_{ap}'
                    av, uv = packet[ap], packet[utb]
                    bad |= not math.isfinite(av) or not math.isfinite(uv)
                    if math.isfinite(av) and math.isfinite(uv):
                        diffs.setdefault(name, []).append(uv - av)
                    row[f'{axis}_{ap}'] = av
                    row[f'{axis}_{utb}'] = uv
            bad |= not math.isfinite(rate['Gy'])
            row[f'{axis}_Gy'] = rate['Gy']
            row[f'{axis}_Flags'] = pid['Flg']
        bad |= att['SrcUS'] != rate0['SrcUS'] or att['Flg'] & 3 != 3
        aq = [att[f'A{k}'] for k in range(1, 5)]
        uq = [att[f'U{k}'] for k in range(1, 5)]
        for prefix in ('Q', 'A', 'U'):
            q = [att[f'{prefix}{k}'] for k in range(1, 5)]
            bad |= any(not math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1) > 1e-4
        sign = -1 if sum(a*u for a, u in zip(aq, uq)) < 0 else 1
        for k, (a, u) in enumerate(zip(aq, uq)):
            if math.isfinite(a) and math.isfinite(u):
                diffs.setdefault(f'quat{k}', []).append(sign*u-a)
        invalid += bool(bad)
        row.update(SrcUS=rate0['SrcUS'], Loop=rate0['Lp'], Source=rate0['Src'], dt=rate0['Dt'], Invalid=bool(bad))
        rows.append(row)
    times = [row['TimeUS'] for row in rows]
    periods = [b-a for a, b in zip(times, times[1:])]
    frequency = dict(period_min_us=min(periods), period_max_us=max(periods),
                     period_mean_us=sum(periods)/len(periods),
                     stored_hz=(len(times)-1)*1000000/(times[-1]-times[0])) if periods and min(periods) > 0 else None
    final = statuses[-1] if statuses else None
    if final is None or final['St'] != 2:
        errors.append('Missing terminal status: totals and persistence cannot be qualified')
    unique = set(seqs)
    if len(unique) != len(seqs) or any(b <= a for a, b in zip(seqs, seqs[1:])):
        errors.append('Nonmonotonic or reused sequence identity')
    gaps = max(unique, default=0) - len(unique)
    if duplicates:
        errors.append('Duplicate packets')
    if partial or invalid:
        errors.append('Partial or invalid sets')
    metrics = {k: dict(max_abs=max(map(abs, v)), rmse=math.sqrt(sum(x*x for x in v)/len(v)), count=len(v))
               for k, v in diffs.items() if v}
    mismatch = any(v['max_abs'] > (1e-6 if k.startswith('quat') else 0) for k, v in metrics.items())
    if mismatch:
        errors.append('Numerical mismatch')
    if final and final['St'] == 2:
        if final['Rsn'] in (4, 5, 6, 9, 11):
            errors.append('Diagnostic fault or rejected configuration')
        if final['Req'] != final['Que'] + final['Drp'] + final['Inv']:
            errors.append('Producer accounting mismatch')
        if final['Que'] != final['Acc'] + final['Rej'] + final['Dep']:
            errors.append('Consumer accounting mismatch')
        if final['Acc'] != len(rows):
            errors.append('Accepted sets missing from persistent BIN')
        if any(final[k] for k in ('Drp', 'Inv', 'Uns', 'Rej', 'Dep')):
            errors.append('Reported losses or unsupported data')
        if max(unique, default=0) > final['Req']:
            errors.append('Sequence beyond requested count')
    return dict(complete=len(rows), partial=partial, invalid=invalid, duplicates=duplicates, sequence_gaps=gaps,
                final=final, frequency=frequency, status_count=len(statuses), metrics=metrics, errors=errors,
                qualified=bool(rows) and not errors, rows=rows)


def read_bin(path):
    """DFReader decodes actual FMT; legacy UTBC is deliberately ignored."""
    records = []
    reader = mavutil.mavlink_connection(str(path))
    try:
        while (message := reader.recv_match(type=list(NAMES))) is not None:
            records.append(message.to_dict())
    finally:
        reader.close()
    return analyze(records)


def write_results(result, output):
    output.mkdir(parents=True, exist_ok=True)
    (output/'summary.json').write_text(json.dumps({k: v for k, v in result.items() if k != 'rows'}, indent=2))
    if result['rows']:
        with (output/'samples.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(result['rows'][0]))
            writer.writeheader()
            writer.writerows(result['rows'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bin', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = read_bin(args.bin)
    write_results(result, args.output)
    print(json.dumps({k: result[k] for k in ('complete', 'partial', 'invalid', 'qualified', 'errors')}))
    return 0 if result['qualified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
