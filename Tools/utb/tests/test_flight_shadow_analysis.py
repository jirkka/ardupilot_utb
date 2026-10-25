# AP_FLAKE8_CLEAN
"""Synthetic protocol fixtures, never presented as measured flight data."""
import copy
import csv
import io
from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from utb_flight_shadow_analysis import parse, check_supported  # noqa: E402


def fixture(attempts=4, stride=2):
    selected = (attempts - 1) // stride + 1
    used = min(selected, 128)
    overwritten = selected - used
    metadata = [
        'UTBFlightV1,frozen,reason=1,start_us=1000,stop_us=1000000',
        f'storage,record_bytes=304,capacity=128,rows={used},pages=8,stride={stride},limit={attempts},persistent=0',
        f'records,candidates={attempts},selected={selected},decimated={attempts-selected},overwritten={overwritten}',
        'context,epochs=1,arm_changes=0,mode_exits=0,unsupported=0',
    ]
    scalars = dict(attempts=attempts, paired=attempts-1, warmup=1, att_seen=attempts, att_valid=attempts,
                   pid_seen=attempts-1, pid_valid=attempts-1, att_synchronizations=1, pid_synchronizations=1,
                   motor_checks=2*attempts)
    for name in ('missing', 'duplicate', 'invalid', 'unsupported', 'upstream_mismatch', 'downstream_mismatch',
                 'motor_changes', 'att_invalid', 'att_desync', 'att_mismatch', 'att_unsupported',
                 'pid_invalid', 'pid_desync', 'pid_mismatch', 'pid_unsupported'):
        scalars[name] = 0
    metadata.extend(f'{key},{value}' for key, value in scalars.items())
    for section, axes, length in (('att', 1, 26), ('target', 1, 4), ('pid', 3, 18)):
        for axis in range(axes):
            for metric in range(length):
                metadata.append(f'flight_metric64,{section},{axis},{metric},0000000000000000,0000000000000000')
    rows = []
    for number in range(overwritten + 1, selected + 1):
        loop = 10 + (number-1) * stride
        row = dict(Id=number, Loop=loop, Source=loop-1 if number > 1 else loop, TimeUS=loop*2500,
                   SourceUS=(loop-1)*2500, Epoch=1, Resets=0, Mode=0, Spool=3, Flags=1 if number > 1 else 0,
                   Limits=0, ResetFlags=0, Dt=.0025)
        for prefix in ('Q', 'AQT', 'UQT'):
            row.update({f'{prefix}{axis}': float(axis == 0) for axis in range(4)})
        for axis in range(3):
            for prefix in ('Cmd', 'Gyro', 'ATar', 'UTar', 'AOut', 'UOut', 'AFFOut', 'UFFOut'):
                row[f'{prefix}{axis}'] = .1
            for prefix in ('AP', 'UTB'):
                for term in ('P', 'I', 'D', 'FF', 'DFF'):
                    row[f'{prefix}{axis}{term}'] = .1
        rows.append(row)
    return '\n'.join(metadata), rows, selected, overwritten


def pages(rows, selected, overwritten):
    result = []
    for index in range(8):
        stream = io.StringIO()
        stream.write(f'UTBFlightRowsV1,start_us=1000,page={index},rows={len(rows)},'
                     f'selected={selected},overwritten={overwritten},float_bits=32\n')
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        integers = {'Id', 'Loop', 'Source', 'TimeUS', 'SourceUS', 'Epoch', 'Resets', 'Mode',
                    'Spool', 'Flags', 'Limits', 'ResetFlags'}
        writer.writerows([{key: value if key in integers else struct.pack('>f', value).hex() for key, value in row.items()}
                          for row in rows[16*index:16*(index+1)]])
        result.append(stream.getvalue())
    return result


def test_complete_protocol_and_zero_metrics():
    metadata, rows, selected, overwritten = fixture()
    result = parse(metadata, pages(rows, selected, overwritten))
    check_supported(result)
    assert len(result['metrics']) == 84 and len(result['rows']) == 2


def test_full_buffer_loss_is_explicit_not_false_zero():
    metadata, rows, selected, overwritten = fixture(130, 1)
    result = parse(metadata, pages(rows, selected, overwritten))
    check_supported(result)
    assert result['records']['overwritten'] == 2
    assert result['rows'][0]['Id'] == 3


@pytest.mark.parametrize('fault', [
    'missing-page', 'mixed-session', 'duplicate-id', 'missing-row',
    'wrong-source', 'duplicate-metric', 'pending', 'restart',
])
def test_reject_protocol_fault(fault):
    metadata, rows, selected, overwritten = fixture()
    modified = copy.deepcopy(rows)
    if fault == 'duplicate-id':
        modified[1]['Id'] = modified[0]['Id']
    elif fault == 'missing-row':
        modified.pop()
    elif fault == 'wrong-source':
        modified[1]['Source'] -= 1
    exported = pages(modified, selected, overwritten)
    if fault == 'missing-page':
        exported.pop()
    elif fault == 'mixed-session':
        exported[1] = exported[1].replace('start_us=1000', 'start_us=2000')
    elif fault == 'duplicate-metric':
        metadata += '\n' + metadata.splitlines()[-1]
    elif fault in ('pending', 'restart'):
        metadata = 'UTBFlightV1,pending\n'
    with pytest.raises(AssertionError):
        parse(metadata, exported)


@pytest.mark.parametrize('field', ('UTar0', 'UOut0', 'UTB0I', 'UFFOut0'))
def test_own_diagnostic_fault_is_rejected_without_changing_native(field):
    metadata, rows, selected, overwritten = fixture()
    native = rows[1]['AOut0']
    rows[1][field] += .25
    result = parse(metadata, pages(rows, selected, overwritten))
    with pytest.raises(AssertionError):
        check_supported(result)
    assert rows[1]['AOut0'] == native


def test_binary64_small_error_is_never_rounded_to_false_zero():
    metadata, rows, selected, overwritten = fixture()
    bits = struct.pack('>d', 1e-100).hex()
    metadata = metadata.replace('flight_metric64,pid,0,0,0000000000000000', f'flight_metric64,pid,0,0,{bits}')
    result = parse(metadata, pages(rows, selected, overwritten))
    assert next(r for r in result['metrics'] if (r['section'], r['axis'], r['metric']) == ('pid', 0, 0))['max_abs'] == 1e-100
    with pytest.raises(AssertionError):
        check_supported(result)


def test_error_accounting_is_not_hidden_by_zero_exported_metrics():
    metadata, rows, selected, overwritten = fixture()
    metadata = metadata.replace('reason=1', 'reason=3').replace('invalid,0', 'invalid,1')
    result = parse(metadata, pages(rows, selected, overwritten))
    with pytest.raises(AssertionError):
        check_supported(result, 3)


def test_missing_motor_audit_is_not_false_pass():
    metadata, rows, selected, overwritten = fixture()
    metadata = metadata.replace('motor_checks,8', 'motor_checks,0')
    result = parse(metadata, pages(rows, selected, overwritten))
    with pytest.raises(AssertionError):
        check_supported(result)
