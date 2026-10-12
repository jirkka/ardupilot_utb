#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline audit of internal-reference BIN recordings. Accepts files only, never serial ports."""
import argparse
import hashlib
import json
from pathlib import Path

from pymavlink import mavutil

from test_utb_math_sitl import packet_roundtrip, verify_recurrence
from test_utb_synthetic_reference import identity, reference


def analyse(path):
    path = path.resolve(strict=True)
    if not path.is_file() or path.suffix.lower() != '.bin':
        raise ValueError('An existing regular BIN file is required')
    reader = mavutil.mavlink_connection(str(path))
    metadata, packets = [], []
    while True:
        message = reader.recv_match(type=['UTBC', 'UTBX'])
        if message is None:
            break
        (metadata if message.get_type() == 'UTBX' else packets).append(message.to_dict())
    if not metadata:
        raise ValueError('No UTBX source identity; cannot attribute internal reference')
    groups, previous, comparisons, maxima = {}, None, 0, {}
    for packet in packets:
        decoded = packet_roundtrip(packet)
        source = identity(packet, metadata)
        if source is None:
            previous = None
            continue
        profile, cycle = source
        _, target, bounds = reference(cycle, profile)
        for axis in range(3):
            assert abs(decoded[axis]['Des'] - target[axis]) <= bounds[axis], (packet['Seq'], 'reference', axis)
        group = groups.setdefault(
            packet['Epoch'], dict(profile=profile, sets=0, valid=0, first_us=packet['TimeUS'],
                                  last_us=0, requested_hz=decoded[5]['Req'], steps={}, last_seq=None))
        group['sets'] += 1
        group['valid'] += int(bool(decoded[3]['Val']))
        group['last_us'] = packet['TimeUS']
        if group['last_seq'] is not None:
            step = packet['Seq'] - group['last_seq']
            group['steps'][str(step)] = group['steps'].get(str(step), 0) + 1
        group['last_seq'] = packet['Seq']
        if (previous and previous[0]['Epoch'] == packet['Epoch'] and previous[0]['Md'] == packet['Md']
                and previous[0]['Seq'] + 1 == packet['Seq']
                and all(r['Val'] for r in previous[1][:3] + decoded[:3])):
            for name, error in verify_recurrence(previous[1], decoded).items():
                maxima[name] = max(maxima.get(name, 0), error)
            comparisons += 1
        previous = (packet, decoded)
    for group in groups.values():
        span = group['last_us'] - group['first_us']
        group['interior_hz'] = (group['sets'] - 1) * 1e6 / span if span else None
        group['full_400_sequence'] = (group['requested_hz'] == 400 and group['sets'] == group['valid'] == 8000
                                      and group['steps'] == {'1': 7999})
        del group['last_seq']
    return dict(file=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(), groups=groups,
                decoded_packed_sets=len(packets), pid_adjacent_pairs=comparisons, max_absolute_error=maxima,
                assumed_gains='All axes: P .31, I .23, D .004, IMAX .19, D_HZ 20; confirm parameter archive separately',
                hardware_origin_verified=False,
                limitation='This check does not certify PM, drops, stack, mutex timing or physical stability')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bin', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = analyse(args.bin)
    with args.output.open('x') as stream:
        stream.write(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
