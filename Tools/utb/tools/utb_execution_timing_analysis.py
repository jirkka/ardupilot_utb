#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @author Jiri Kovar
# @version 2026-10-25
"""Strict frozen UTT execution parser; no serial or vehicle access."""
import argparse
import csv
import json
from pathlib import Path

NAMES = ('main_to_motor_begin', 'motor_call', 'main_to_motor_return', 'main_to_scheduler_boundary',
         'rate_to_motor_begin', 'utb_rate_consume', 'main_to_mode_finish', 'main_to_fast_boundary')


def fields(line):
    """Decode named integer metadata without accepting duplicate keys."""
    result = {}
    for part in line.split(',')[1:]:
        key, value = part.split('=', 1)
        assert key not in result, 'Duplicated metadata key'
        result[key] = value if key == 'centre' else int(value)
    return result


def parse(text, expected_session=None):
    """Validate a complete single frozen export; boot identity is a separate host envelope."""
    lines = text.splitlines()
    assert lines and lines[0].startswith('UTBExecutionV1,frozen,'), 'Not a frozen execution snapshot'
    assert '\x00' not in text and lines[-1].startswith('end,'), 'Interrupted or malformed export'
    header = fields(lines[0].replace('UTBExecutionV1,frozen,', 'header,'))
    assert fields(lines[-1]) == {'session_us': header['session_us']}, 'Mixed sessions'
    if expected_session is not None:
        assert header['session_us'] == expected_session, 'Unexpected session identity'
    assert set(header) == {'session_us', 'reason', 'cycles', 'limit'}
    assert header['reason'] in (1, 2, 4, 6, 7, 8) and 0 <= header['session_us'] < 1 << 64
    if header['reason'] == 4 and header['cycles'] == 0:
        assert header['limit'] == 0  # Invalid request froze before a measurement window was started.
    else:
        assert 1 <= header['limit'] <= 20000 and 0 <= header['cycles'] <= header['limit']
    metrics, rows, metadata = [], [], {}
    keys = None
    for line in lines[1:-1]:
        values = line.split(',')
        if values[0] == 'metric':
            if values[1] == 'id':
                keys = values[1:]
                continue
            assert keys is not None and len(values[1:]) == len(keys), 'Malformed metric'
            metric = dict(zip(keys, map(int, values[1:])))
            assert 0 <= metric['id'] < 8
            n = metric['count']
            assert 0 <= n <= header['cycles'] and metric['over5000'] <= metric['over2500'] <= n
            if n:
                assert 0 <= metric['min'] <= metric['max'] <= 0x7fffffff
                assert metric['min'] * n <= metric['sum'] <= metric['max'] * n, 'Corrupt sums'
                assert 0 <= metric['min_us'] <= 0xffffffff and 0 <= metric['max_us'] <= 0xffffffff
                assert 1 <= metric['min_cycle'] <= header['cycles']
                assert 1 <= metric['max_cycle'] <= header['cycles']
            else:
                assert metric['sum'] == metric['max'] == metric['over2500'] == metric['over5000'] == 0
                assert metric['min'] == 0xffffffff
            metric['name'] = NAMES[metric['id']]
            metric['mean_us'] = metric['sum'] / n if n else None
            metric['mean_abs_from_zero_us'] = metric['mean_us']
            metrics.append(metric)
        elif values[0] == 'row':
            assert len(values) == 14
            numbers = list(map(int, values[1:]))
            row = dict(cycle=numbers[0], main=numbers[1], present=numbers[2], bad=numbers[3], times=numbers[4:])
            assert 1 <= row['cycle'] <= header['cycles'] and row['bad'] in (0, 1)
            assert 0 <= row['present'] < 1 << 9
            assert all(0 <= value <= 0xffffffff for value in [row['main']] + row['times'])
            if not row['bad']:
                assert bool(row['present'] & 2) == bool(row['present'] & 4), 'Half consume record'
                assert row['present'] & 505 == 505, 'Missing required event'
                sequence = [row['main']] + [v for k, v in enumerate(row['times']) if row['present'] & (1 << k)]
                assert all(((b - a) & 0xffffffff) <= 0x7fffffff for a, b in zip(sequence, sequence[1:]))
                assert all(((value - row['main']) & 0xffffffff) <= 0x7fffffff for value in sequence)
            rows.append(row)
        else:
            assert values[0] in ('health', 'hooks', 'context') and values[0] not in metadata
            metadata[values[0]] = fields(line)
    assert set(metadata) == {'health', 'hooks', 'context'}
    assert set(metadata['health']) == {'missing', 'duplicate', 'invalid', 'gaps', 'unsupported'}
    assert all(v >= 0 for v in metadata['health'].values())
    assert len(metrics) == 8 and {m['id'] for m in metrics} == set(range(8))
    by_id = {m['id']: m for m in metrics}
    n = by_id[0]['count']
    assert all(by_id[k]['count'] == n for k in (1, 2, 3, 4, 6, 7))
    assert by_id[5]['count'] <= n
    assert by_id[2]['sum'] == by_id[0]['sum'] + by_id[1]['sum']
    assert by_id[3]['sum'] >= by_id[7]['sum'] >= by_id[6]['sum'] >= by_id[2]['sum']
    assert metadata['context'] == dict(rows=len(rows), capacity=17, centre='main_to_motor_return')
    assert len(rows) <= 17
    assert all(((b['cycle'] - a['cycle']) & 0xffffffff) == 1 for a, b in zip(rows, rows[1:]))
    raw_maximum_seen = False
    for row in rows:
        if row['bad']:
            continue
        value = (row['times'][4] - row['main']) & 0xffffffff
        assert by_id[2]['min'] <= value <= by_id[2]['max'], 'Context outside metric range'
        raw_maximum_seen |= value == by_id[2]['max']
    assert n == 0 or raw_maximum_seen, 'Missing maximum context'
    if header['reason'] == 1:
        assert header['cycles'] == header['limit'] and n == header['cycles']
        assert all(row['bad'] == 0 for row in rows)
        assert metadata['hooks']['calls'] == 4 * n + 2 * by_id[5]['count']
        assert all(v == 0 for v in metadata['health'].values()), 'Fault marked as successful count stop'
    return dict(header=header, **metadata, metrics=metrics, rows=rows)


def write(result, output):
    """Export only the actual aggregated and bounded raw values."""
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(result, indent=2))
    with (output / 'metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result['metrics'][0]))
        writer.writeheader()
        writer.writerows(result['metrics'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    write(parse(args.snapshot.read_text()), args.output)
