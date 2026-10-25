#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file utb_flight_shadow_analysis.py
# @brief Strict offline analysis of frozen, bounded Flight RAM pages; never connects to a vehicle.
# @author Jiri Kovar
# @version 2026-10-25
import argparse
import csv
import io
import json
import math
from pathlib import Path
import struct


def fields(line):
    """Decode explicitly named immutable snapshot metadata."""
    return {key: int(value) for part in line.split(',')[1:] if '=' in part for key, value in [part.split('=', 1)]}


def parse(metadata, pages):
    """Reject pending/restarted/mixed/partial exports; do not silently discard bad rows."""
    lines = metadata.splitlines()
    assert '\x00' not in metadata and all('\x00' not in page for page in pages), 'Malformed text export'
    assert lines and lines[0].startswith('UTBFlightV1,frozen,'), 'Session is pending or armed, not a frozen result'
    header = fields(lines[0])
    groups = {line.split(',')[0]: fields(line) for line in lines if line.startswith(('storage,', 'records,', 'context,'))}
    assert set(groups) == {'storage', 'records', 'context'}
    storage, counts = groups['storage'], groups['records']
    assert storage['capacity'] == 128 and storage['pages'] == 8 and storage['persistent'] == 0
    assert 0 < storage['stride'] <= 400 and 2 <= storage['limit'] <= 48000
    assert counts['candidates'] == counts['selected'] + counts['decimated']
    assert storage['rows'] == min(storage['capacity'], counts['selected'])
    assert counts['overwritten'] == counts['selected'] - storage['rows']
    assert header['stop_us'] >= header['start_us']
    metrics = []
    scalars = {}
    last_context = {}
    for line in lines:
        parts = line.split(',')
        if parts[0] == 'flight_metric64':
            metrics.append(dict(section=parts[1], axis=int(parts[2]), metric=int(parts[3]),
                                max_abs=struct.unpack('>d', bytes.fromhex(parts[4]))[0],
                                rmse=struct.unpack('>d', bytes.fromhex(parts[5]))[0]))
        elif len(parts) == 2 and parts[1].isdigit():
            assert parts[0] not in scalars, 'Duplicated numerical accounting'
            scalars[parts[0]] = int(parts[1])
        elif parts[0] in ('last_values32', 'last_attitude_values32'):
            last_context[parts[0]] = [struct.unpack('>f', bytes.fromhex(value))[0] for value in parts[1:]]
    keys = {(r['section'], r['axis'], r['metric']) for r in metrics}
    expected = {('att', 0, k) for k in range(26)} | {('target', 0, k) for k in range(4)} | {
        ('pid', axis, k) for axis in range(3) for k in range(18)}
    assert keys == expected and len(metrics) == 84, 'Missing/duplicate numerical metric'
    assert all(math.isfinite(r[k]) and r[k] >= 0 for r in metrics for k in ('max_abs', 'rmse'))
    assert len(pages) == 8, 'Incomplete page export'
    rows = []
    for index, page in enumerate(pages):
        first, separator, body = page.partition('\n')
        assert separator and first.startswith('UTBFlightRowsV1,'), 'Pending/unreadable page'
        info = fields(first)
        assert info['start_us'] == header['start_us'], 'Mixed boot/session export'
        assert info['page'] == index and info['rows'] == storage['rows']
        assert info['selected'] == counts['selected'] and info['overwritten'] == counts['overwritten']
        assert info['float_bits'] == 32
        integers = {'Id', 'Loop', 'Source', 'TimeUS', 'SourceUS', 'Epoch', 'Resets', 'Mode',
                    'Spool', 'Flags', 'Limits', 'ResetFlags'}
        decoded = [{key: int(value) if key in integers else struct.unpack('>f', bytes.fromhex(value))[0]
                    for key, value in row.items()} for row in csv.DictReader(io.StringIO(body))]
        assert len(decoded) == max(0, min(16, storage['rows'] - 16 * index)), 'Missing/duplicate page row'
        assert all(math.isfinite(v) for row in decoded for v in row.values()), 'Nonfinite exported record'
        rows.extend(decoded)
    assert [r['Id'] for r in rows] == list(range(counts['overwritten'] + 1, counts['selected'] + 1))
    assert len({r['Loop'] for r in rows}) == len(rows), 'Duplicate main record'
    for row in rows:
        assert row['Mode'] == 0, 'Unsupported flight mode marked as supported'
        assert int(row['Flags']) & 1 == 0 or (int(row['Loop']) - int(row['Source'])) & 0xffffffff == 1
        assert 0 < row['Dt'] and row['Epoch'] >= 1
    return dict(header=header, storage=storage, records=counts, context=groups['context'],
                scalars=scalars, metrics=metrics, last_context=last_context, rows=rows)


def check_supported(result, reason=1):
    """Apply the existing chain's criteria to every full-rate numerical attempt."""
    s = result['scalars']
    assert result['header']['reason'] == reason
    assert s['warmup'] == 1 and s['paired'] + 1 == s['attempts']
    assert s['pid_seen'] == s['pid_valid'] == s['paired']
    assert s['att_seen'] == s['att_valid'] and s['att_synchronizations'] == s['pid_synchronizations'] == 1
    assert s['motor_checks'] == s['attempts'] + s['att_seen'], 'Missing native Rate/Attitude motor audits'
    assert result['records']['candidates'] == s['attempts']
    if reason == 1:
        assert s['attempts'] == result['storage']['limit']
    for key in ('missing', 'duplicate', 'invalid', 'unsupported', 'upstream_mismatch', 'downstream_mismatch',
                'motor_changes', 'att_invalid', 'att_desync', 'att_mismatch', 'att_unsupported',
                'pid_invalid', 'pid_desync', 'pid_mismatch', 'pid_unsupported'):
        assert s[key] == 0, (key, s[key])
    assert all(r['max_abs'] == r['rmse'] == 0 for r in result['metrics'])
    for row in result['rows']:
        if not int(row['Flags']) & 1:
            continue
        for axis in range(3):
            assert row[f'ATar{axis}'] == row[f'UTar{axis}']
            assert row[f'AOut{axis}'] == row[f'UOut{axis}']
            assert row[f'AFFOut{axis}'] == row[f'UFFOut{axis}']
            for term in ('P', 'I', 'D', 'FF', 'DFF'):
                assert row[f'AP{axis}{term}'] == row[f'UTB{axis}{term}']


def write_results(result, output):
    """Preserve the actual retained rows and all 84 metrics, including overwrite accounting."""
    output.mkdir(parents=True, exist_ok=True)
    for name in ('rows', 'metrics'):
        values = result[name]
        if values:
            with (output / (name + '.csv')).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(values[0]))
                writer.writeheader()
                writer.writerows(values)
    (output / 'summary.json').write_text(json.dumps({k: v for k, v in result.items() if k != 'rows'}, indent=2))


def plot(result, output):
    """Only recorded sample instants are plotted; no reconstruction of missing 400Hz samples."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for r in result['rows'] if int(r['Flags']) & 1]
    assert rows, 'No retained paired rows to plot'
    origin = rows[0]['TimeUS']
    times = [((int(r['TimeUS']) - int(origin)) & 0xffffffff) / 1e6 for r in rows]
    plots = [
        ('body-targets', [('ATar', 'UTar', 'Gyro')], 'rad/s'),
        ('pid-outputs', [('AOut', 'UOut', None)], 'AP control units'),
        ('integrator', [('AP', 'UTB', 'I')], 'AP I units'),
        ('feedforward', [('AP', 'UTB', 'FF'), ('AP', 'UTB', 'DFF')], 'AP FF units'),
    ]
    for filename, sets, unit in plots:
        fig, axes = plt.subplots(len(sets), 3, figsize=(14, 3 * len(sets)), squeeze=False)
        for line, (native, own, suffix) in enumerate(sets):
            for axis in range(3):
                ax = axes[line][axis]
                for prefix, label in ((native, 'AP'), (own, 'UTB')):
                    key = f'{prefix}{axis}{suffix}' if prefix in ('AP', 'UTB') else f'{prefix}{axis}'
                    line_label = label + ' target' if suffix == 'Gyro' else label
                    ax.plot(times, [r[key] for r in rows], '.-' if label == 'AP' else 'x--',
                            label=line_label, lw=.8, ms=3)
                if suffix == 'Gyro':
                    ax.plot(times, [r[f'Gyro{axis}'] for r in rows], '.-', label='actual gyro', lw=.7, ms=2)
                ax.set_ylabel(unit)
                title_suffix = ' body rates' if suffix == 'Gyro' else ' ' + suffix if suffix else ''
                ax.set_title(('Roll', 'Pitch', 'Yaw')[axis] + title_suffix)
                ax.grid()
                ax.legend()
                ax.set_xlabel('Retained sample time [s]')
        fig.suptitle('Actual decimated RAM records; not a reconstructed 400Hz trace')
        fig.tight_layout()
        fig.savefig(output / (filename + '.png'), dpi=140)
        plt.close(fig)
    fig, ax = plt.subplots(figsize=(13, 4))
    for name in ('max_abs', 'rmse'):
        ax.plot(range(84), [r[name] for r in result['metrics']], '.-', label=name)
    ax.set_xlabel('84 metric IDs: 26 Attitude + 4 target + 54 PID')
    ax.set_ylabel('Original metric units')
    ax.grid()
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / 'numerical-errors.png', dpi=140)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(times, [r['Dt'] * 1e6 for r in rows], '.-')
    axes[0].set_ylabel('Captured native dt [us]')
    axes[1].plot(times, [r['Id'] for r in rows], '.-', label='stored ID')
    axes[1].plot(times, [r['Loop'] - rows[0]['Loop'] for r in rows], '.-', label='main ID relative to first stored')
    axes[1].set_ylabel('IDs (decimation explicit)')
    axes[1].set_xlabel('Retained sample time [s]')
    axes[1].legend()
    for ax in axes:
        ax.grid()
    fig.suptitle(f"Actual stored rows; overwritten={result['records']['overwritten']}; dt is not CPU execution time")
    fig.tight_layout()
    fig.savefig(output / 'timing-records.png', dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metadata', required=True, type=Path)
    parser.add_argument('--pages', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--reason', type=int, default=1)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    result = parse(args.metadata.read_text(), [(args.pages / f'utb_flight{k}.csv').read_text() for k in range(8)])
    write_results(result, args.output)
    check_supported(result, args.reason)
    if args.plot:
        plot(result, args.output)


if __name__ == '__main__':
    main()
