#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Offline evidence extraction and bounded transfer planning; no device API."""
import argparse
import collections
import datetime
import hashlib
import json
from pathlib import Path


def utc(value):
    return datetime.datetime.fromtimestamp(value, datetime.timezone.utc).isoformat()


def changes(rows, key):
    result = []
    for row in rows:
        if not result or row[key] != result[-1][key]:
            result.append(row)
    return result


class TransferPlan:
    """Pure request/ack state machine, operated against a fake peer only here.

    One outstanding request, small chunks, bounded retries, immutable log identity.
    A partial/short response is NOT silently treated as valid EOF. No serial code.
    """

    def __init__(self, identity, size, chunk=1440, retries=3):
        if size <= 0 or chunk <= 0 or chunk % 90 or retries < 0:
            raise ValueError("Invalid transfer dimensions")
        self.identity, self.size, self.chunk, self.retries = identity, size, chunk, retries
        self.data = bytearray(size)
        self.seen = bytearray(size)
        self.pending = None
        self.attempts = 0

    def request(self, identity):
        if identity != self.identity:
            raise ValueError("Log identity changed")
        if self.pending is None:
            try:
                offset = self.seen.index(0)
            except ValueError:
                return None
            self.pending = (offset, min(self.chunk, self.size - offset))
            self.attempts = 0
        self.attempts += 1
        if self.attempts > self.retries + 1:
            raise TimeoutError("Bounded retry budget exhausted")
        return self.pending

    def acknowledge(self, identity, offset, payload):
        if identity != self.identity or self.pending is None:
            raise ValueError("Unexpected transfer identity/state")
        lo, count = self.pending
        hi = offset + len(payload)
        if not payload or offset < lo or hi > lo + count or len(payload) > 90:
            raise ValueError("Invalid packet/offset; zero count is not proof of EOF")
        for index, value in enumerate(payload, offset):
            if self.seen[index] and self.data[index] != value:
                raise ValueError("Conflicting duplicate")
            self.data[index], self.seen[index] = value, 1
        if all(self.seen[lo:lo + count]):
            self.pending = None


def parse_tlog(path):
    from pymavlink import mavutil
    if not path.is_file() or path.suffix != '.tlog':
        raise ValueError("Only existing tlog files accepted")
    reader = mavutil.mavlink_connection(str(path.resolve()))
    counts = collections.Counter()
    ekf, anchors, status, metrics, entries = [], [], [], [], []
    bounds, data_range = [], []
    while True:
        message = reader.recv_match()
        if message is None:
            break
        kind, d = message.get_type(), message.to_dict()
        host = message._timestamp
        counts[kind] += 1
        bounds.append(host)
        base = {'host_s': host, 'utc': utc(host)}
        if 'time_boot_ms' in d:
            anchors.append(dict(base, fc_boot_ms=d['time_boot_ms']))
        if kind == 'EKF_STATUS_REPORT':
            ekf.append(dict(base, flags=d['flags'], velocity_variance=d['velocity_variance'],
                            pos_vert_variance=d['pos_vert_variance'], compass_variance=d['compass_variance']))
        if kind == 'STATUSTEXT':
            status.append(dict(base, text=d['text']))
        if kind == 'NAMED_VALUE_INT' and d['name'] in ['UB22', 'UB23', 'UB29', 'UB27', 'UB28', 'UTBS_CALC']:
            metrics.append(dict(base, fc_boot_ms=d['time_boot_ms'], name=d['name'], value=d['value']))
        if kind == 'LOG_ENTRY':
            entries.append(dict(base, **d))
        if kind == 'LOG_DATA':
            data_range.append((host, d['ofs'], d['count']))
    reader.close()
    offsets = [a['host_s'] - a['fc_boot_ms'] / 1000 for a in anchors]
    return {'counts': dict(counts), 'host_range': [min(bounds), max(bounds)] if bounds else None,
            'boot_anchor_first_last': [anchors[0], anchors[-1]] if anchors else [],
            'host_minus_boot_range_s': [min(offsets), max(offsets)] if offsets else None,
            'clock_alignment': 'Receive-time offsets include buffering/transport latency; not exact boot time',
            'ekf_changes': changes(ekf, 'flags'), 'ekf_last': ekf[-1] if ekf else None,
            'statustext': status, 'metrics': metrics, 'log_entries': entries,
            'data': {'first': data_range[0], 'last': data_range[-1],
                     'received_payload_bytes': sum(x[2] for x in data_range)} if data_range else None}


def parse_bin(path):
    from pymavlink import mavutil
    if not path.is_file() or path.suffix != '.bin':
        raise ValueError("Only existing BIN files accepted")
    reader = mavutil.mavlink_connection(str(path.resolve()))
    records = collections.defaultdict(list)
    first, last = None, None
    while True:
        m = reader.recv_match()
        if m is None:
            break
        d, kind = m.to_dict(), m.get_type()
        if 'TimeUS' in d:
            t = d['TimeUS']
            first, last = t if first is None else min(first, t), t if last is None else max(last, t)
        if kind in ['UTBS', 'UTBX', 'UTBQ', 'PM', 'DSF', 'MSG', 'UBST']:
            records[kind].append(d)
    reader.close()
    keys = {}
    for key in [22, 23, 27, 28, 29]:
        keys[str(key)] = changes([r for r in records['UBST'] if r['Key'] == key], 'Val')
    return {'dataflash_timeus_range': [first, last], 'UTBS_changes': changes(records['UTBS'], 'Flg'),
            'UTBX_status_changes': changes(records['UTBX'], 'Test'),
            'UTBQ_endpoints': records['UTBQ'][::max(1, len(records['UTBQ'])-1)],
            'PM': records['PM'], 'DSF_changes': changes(records['DSF'], 'Dp'), 'MSG': records['MSG'], 'UBST_changes': keys}


def audit(root, output):
    output.mkdir(parents=True, exist_ok=False)
    evidence = {'tlogs': {}, 'bins': {}, 'host_events': []}
    for path in sorted(root.rglob('*.tlog')):
        evidence['tlogs'][str(path.relative_to(root))] = parse_tlog(path)
    for name in ['SYNTHETIC_SHORT.bin', 'SYNTHETIC_FULL_400.bin', 'before-upload-log-1.bin']:
        evidence['bins'][name] = parse_bin(root / name)
    for path in sorted(root.rglob('*.json')):
        d = json.loads(path.read_text())
        if not isinstance(d, dict):
            continue
        for key in ['start', 'end', 'active_start_wall']:
            if isinstance(d.get(key), (int, float)):
                evidence['host_events'].append({'host_s': d[key], 'utc': utc(d[key]),
                                                'source': str(path.relative_to(root)), 'marker': key})
        for key in ['events', 'before_events', 'final_events']:
            for event in d.get(key, []):
                if isinstance(event, dict) and 'wall' in event:
                    evidence['host_events'].append(dict(event, utc=utc(event['wall']), host_s=event['wall'],
                                                        source=str(path.relative_to(root))))
        if 'error' in d:
            evidence['host_events'].append({'source': str(path.relative_to(root)), 'error': d['error']})
    evidence['inputs'] = {str(p.relative_to(root)): {'bytes': p.stat().st_size,
                          'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                          for p in sorted(root.rglob('*')) if p.is_file()}
    (output / 'evidence.json').write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + '\n')
    print('Offline evidence written:', output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    audit(args.archive, args.output)


if __name__ == '__main__':
    main()
