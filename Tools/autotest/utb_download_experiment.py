#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Bounded DataFlash experiment engine. Default operation is offline replay.

Live serial creation requires a separately supplied approval matching the entire
plan. No parameter writes, motor commands, reboot, erase or experiment retry API.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import struct
import time

from utb_download_audit import TransferPlan

PARAMETERS = {'UTB_SHADOW': 0, 'UTB_B_SYNTH': 0, 'LOG_DISARMED': 0,
              'FRAME_CLASS': 1, 'FRAME_TYPE': 18, 'FSTRATE_ENABLE': 0, 'LOG_BACKEND_TYPE': 4}
READ_MESSAGES = (148, 193, 1)


class Stop(RuntimeError):
    pass


def validate_plan(plan):
    required = {'scenario', 'identity', 'baseline_s', 'recovery_s', 'duration_limit_s',
                'chunk_bytes', 'chunks', 'pacing_s', 'request_timeout_s', 'retries', 'log'}
    if set(plan) != required:
        raise ValueError('Incomplete/unknown plan fields')
    if plan['scenario'] not in ['idle', 'list', 'data', 'paced']:
        raise ValueError('Unknown scenario')
    if not 0 < plan['duration_limit_s'] <= 180 or not 0 <= plan['retries'] <= 2:
        raise ValueError('Duration/retry limit')
    if not 1 <= plan['chunks'] <= 8 or not 90 <= plan['chunk_bytes'] <= 8190 or plan['chunk_bytes'] % 90:
        raise ValueError('Data request limits')
    if plan['scenario'] == 'data' and plan['chunks'] != 1:
        raise ValueError('Single request scenario')
    if not 0 <= plan['pacing_s'] <= 5 or not 0 < plan['request_timeout_s'] <= 10:
        raise ValueError('Pacing/timeout limits')
    if not 0 <= plan['baseline_s'] <= 90 or not 0 <= plan['recovery_s'] <= 30:
        raise ValueError('Observation interval limit')
    if plan['baseline_s']+plan['recovery_s'] >= plan['duration_limit_s']:
        raise ValueError('No remaining experiment time')
    if not isinstance(plan['chunks'], int) or not isinstance(plan['chunk_bytes'], int):
        raise ValueError('Integer byte/request counts required')
    identity = plan['identity']
    if set(identity) != {'system', 'component', 'board_id', 'vendor_id', 'product_id',
                         'uid2', 'flight_custom_version', 'flight_sw_version'}:
        raise ValueError('Incomplete FC identity')
    if len(identity['uid2']) != 18 or len(identity['flight_custom_version']) != 8:
        raise ValueError('Invalid FC uid/source prefix')
    if plan['scenario'] in ['data', 'paced']:
        log = plan['log']
        if not isinstance(log, dict) or set(log) != {'id', 'size', 'time_utc', 'archive_sha256'}:
            raise ValueError('An archived immutable log identity is required')
        if log['size'] < plan['chunks']*plan['chunk_bytes'] or log['id'] < 1:
            raise ValueError('Log shorter than bounded test segment')
        if (len(log['archive_sha256']) != 64
                or any(c not in '0123456789abcdef' for c in log['archive_sha256'])):
            raise ValueError('Missing archive hash')
    elif plan['log'] is not None:
        raise ValueError('No selected log for idle/list scenario')
    return plan


class Journal:
    def __init__(self, clock, directory=None):
        self.clock, self.events, self.directory = clock, [], directory
        self.stream = None
        if directory is not None:
            directory.mkdir(parents=True, exist_ok=False)
            self.stream = (directory/'timeline.jsonl').open('x', encoding='utf-8')

    def record(self, event, **fields):
        wall = self.clock.wall()
        row = dict(event=event, host_monotonic_s=self.clock.now(), host_wall_s=wall,
                   host_utc=datetime.datetime.fromtimestamp(wall, datetime.timezone.utc).isoformat(), **fields)
        if getattr(self.clock, 'reconstructed', False):
            row['archive_elapsed_s'] = row.pop('host_monotonic_s')
            row['clock_source'] = 'reconstructed from archived receive wall timestamps'
        # BAD_DATA and some MAVLink array fields can contain bytes, even in a tlog.
        row = json.loads(json.dumps(row, default=wire_json))
        self.events.append(row)
        if self.stream:
            self.stream.write(json.dumps(row, ensure_ascii=False)+'\n')
            self.stream.flush()
        return row

    def close(self, summary):
        if self.directory:
            (self.directory/'summary.json').write_text(json.dumps(summary, indent=2, default=wire_json)+'\n')
        if self.stream:
            self.stream.close()


class Clock:
    @staticmethod
    def now():
        return time.monotonic()

    @staticmethod
    def wall():
        return time.time()


def wire_json(value):
    if isinstance(value, (bytes, bytearray)):
        return {'wire_bytes_hex': value.hex()}
    raise TypeError('Unsupported telemetry value: '+type(value).__name__)


class Engine:
    def __init__(self, transport, clock, plan, journal, expected_segment=None):
        self.transport, self.clock, self.plan, self.journal = transport, clock, validate_plan(plan), journal
        self.start = clock.now()
        self.deadline = self.start+plan['duration_limit_s']
        self.latest, self.seen_at, self.params, self.entries, self.counters = {}, {}, {}, {}, {}
        self.initial_mode, self.last_boot_ms, self.stopping = None, None, False
        self.request_counts = {'LOG_REQUEST_LIST': 0, 'LOG_REQUEST_DATA': 0, 'LOG_REQUEST_END': 0}
        self.rx_bytes, self.last_heartbeat, self.last_poll, self.transfer = 0, -10., -10., None
        self.initial_identity = None
        self.expected_segment = expected_segment
        self.boot_count, self.used = None, False
        self.baseline_loop_max, self.data_started = None, False

    def send(self, kind, **fields):
        if kind not in ['HEARTBEAT', 'REQUEST_MESSAGE', 'PARAM_REQUEST_READ',
                        'LOG_REQUEST_LIST', 'LOG_REQUEST_DATA', 'LOG_REQUEST_END']:
            raise Stop('Forbidden outbound message')
        if self.clock.now() >= self.deadline and kind != 'LOG_REQUEST_END':
            raise Stop('Total experiment deadline before send')
        if kind == 'REQUEST_MESSAGE' and fields.get('message_id') not in READ_MESSAGES:
            raise Stop('Forbidden message request')
        if kind == 'PARAM_REQUEST_READ' and fields.get('name') not in [*PARAMETERS, 'STAT_BOOTCNT']:
            raise Stop('Forbidden parameter read')
        if self.stopping and kind in ['LOG_REQUEST_DATA', 'LOG_REQUEST_LIST']:
            raise Stop('Experiment is already stopped')
        if kind in ['LOG_REQUEST_DATA', 'LOG_REQUEST_LIST'] and not self.ready():
            raise Stop('Health/identity guard before download request')
        if kind == 'LOG_REQUEST_DATA':
            if not self.ready():
                raise Stop('Health/identity guard before data request')
            maximum = self.plan['chunks']*(self.plan['retries']+1)
            if self.request_counts[kind] >= maximum:
                raise Stop('Data request budget exhausted')
            self.data_started = True
        if kind == 'LOG_REQUEST_LIST' and self.request_counts[kind] >= 1:
            raise Stop('Listing cannot be repeated automatically')
        if kind in self.request_counts:
            self.request_counts[kind] += 1
        attempt = self.journal.record('tx_attempt', kind=kind, fields=fields)
        begin = self.clock.now()
        try:
            self.transport.send(kind, fields)
        except OSError as error:
            self.journal.record('transport_error', direction='tx', error=repr(error))
            raise Stop('Transport write failure') from error
        self.journal.record('tx_sent', kind=kind, fields=fields,
                            host_transport_send_duration_s=self.clock.now()-begin,
                            host_journal_before_send_s=begin-attempt['host_monotonic_s'])

    def feed(self, message, check_health=True):
        kind = message['type']
        source = message.get('source', [0, 0])
        d = message['fields']
        self.journal.record('rx', kind=kind, source=source, fields=d,
                            fc_boot_ms=d.get('time_boot_ms'), fc_time_usec=d.get('time_usec'))
        if kind == 'BAD_DATA':
            raise Stop('Malformed MAVLink wire data; raw bytes preserved')
        expected = [self.plan['identity']['system'], self.plan['identity']['component']]
        if source != expected:
            if (kind == 'AUTOPILOT_VERSION' or
                    kind == 'HEARTBEAT' and (source[1] == expected[1] or d.get('autopilot') == 3 or d.get('type') == 2)):
                raise Stop('Unexpected autopilot heartbeat/identity')
            return
        self.latest[kind], self.seen_at[kind] = d, self.clock.now()
        if kind == 'HEARTBEAT':
            if d['base_mode'] & 128 or d['custom_mode'] == 40:
                raise Stop('ARMED or UTB_ACRO state')
            if d['type'] != 2 or d['autopilot'] != 3 or d['system_status'] in [5, 6, 7]:
                raise Stop('Unexpected/critical autopilot heartbeat')
            if self.initial_mode is not None and d['custom_mode'] != self.initial_mode:
                raise Stop('Flight mode changed')
            self.initial_mode = d['custom_mode']
        if 'time_boot_ms' in d:
            boot = d['time_boot_ms']
            # UB00 is one consistent timestamp stream; other messages can carry older samples.
            if kind == 'NAMED_VALUE_INT' and d['name'] == 'UB00':
                if self.last_boot_ms is not None and boot < self.last_boot_ms and self.last_boot_ms-boot < 2**31:
                    raise Stop('FC timestamp regression: restart or out-of-order telemetry')
                self.last_boot_ms = boot
        if kind == 'AUTOPILOT_VERSION':
            required = {'vendor_id', 'product_id', 'uid2', 'flight_sw_version', 'board_version', 'flight_custom_version'}
            if not required.issubset(d):
                raise Stop('Missing AUTOPILOT_VERSION identity fields; MAVLink2 UID2 required')
            actual = {key: d[key] for key in ['vendor_id', 'product_id', 'uid2', 'flight_sw_version']}
            actual['board_id'] = d['board_version'] >> 16
            actual['flight_custom_version'] = bytes(d['flight_custom_version']).decode('ascii', errors='replace')
            expected_values = {k: v for k, v in self.plan['identity'].items() if k not in ['system', 'component']}
            if actual != expected_values or (self.initial_identity is not None and actual != self.initial_identity):
                raise Stop('FC/firmware identity mismatch')
            self.initial_identity = actual
        if kind == 'EKF_STATUS_REPORT' and check_health and (not d['flags'] & 1 or d['flags'] & 1024):
            raise Stop('EKF unhealthy/uninitialized')
        if kind == 'PARAM_VALUE':
            name = d['param_id']
            if name == 'STAT_BOOTCNT':
                if self.boot_count is not None and d['param_value'] != self.boot_count:
                    raise Stop('FC boot counter changed')
                self.boot_count = d['param_value']
            if name in PARAMETERS:
                self.params[name] = d['param_value']
                if d['param_value'] != PARAMETERS[name]:
                    raise Stop('Required frozen configuration mismatch: '+name)
        if kind == 'NAMED_VALUE_INT':
            name = d['name']
            if name.startswith('UB') and name[2:].isdigit():
                self.counters[name] = d['value']
                if (check_health and name == 'UB29' and self.data_started and self.baseline_loop_max is not None
                        and d['value'] > self.baseline_loop_max):
                    raise Stop('New cumulative main-loop period maximum after data start; attribution unproven')
        if kind == 'STATUSTEXT' and check_health:
            text = d['text'].lower()
            if any(t in text for t in ['unhealthy', 'ekf attitude is bad', 'flash_died', 'spi fail', 'watchdog', 'failsafe']):
                raise Stop('Autopilot status warning: '+d['text'])
        if kind == 'LOG_ENTRY':
            if len(self.entries) >= 16 and d['id'] not in self.entries:
                raise Stop('Log listing limit exceeded')
            previous = self.entries.get(d['id'])
            if previous is not None and any(previous[k] != d[k] for k in ['size', 'time_utc']):
                raise Stop('Listed log changed')
            self.entries[d['id']] = d
        if kind == 'LOG_DATA':
            self.rx_bytes += d['count']
            if self.rx_bytes > self.plan['chunks']*self.plan['chunk_bytes']*(self.plan['retries']+2):
                raise Stop('Received byte budget exceeded')
            if self.transfer is None:
                raise Stop('Unsolicited LOG_DATA')
            if d['id'] != self.plan['log']['id'] or d['count'] > len(d['data']):
                raise Stop('Wrong log or truncated wire payload')
            offset, payload = d['ofs'], bytes(d['data'][:d['count']])
            if offset < 0 or offset+len(payload) > self.transfer.size or not payload:
                raise Stop('Invalid log range/zero count')
            if self.expected_segment is not None and payload != self.expected_segment[offset:offset+len(payload)]:
                raise Stop('Archived log prefix mismatch')
            if all(self.transfer.seen[offset:offset+len(payload)]):
                if bytes(self.transfer.data[offset:offset+len(payload)]) != payload:
                    raise Stop('Conflicting duplicate LOG_DATA')
                self.journal.record('duplicate_data', offset=offset, count=len(payload))
                return
            self.transfer.acknowledge(self.transfer.identity, offset, payload)

    def ready(self):
        now = self.clock.now()
        return (self.initial_identity is not None and 'UB29' in self.counters and self.params == PARAMETERS and
                self.boot_count is not None and self.latest.get('EKF_STATUS_REPORT', {}).get('flags', 1024) & 1 and
                not self.latest.get('EKF_STATUS_REPORT', {}).get('flags', 1024) & 1024 and
                all(k in self.seen_at and now-self.seen_at[k] <= 3 for k in ['HEARTBEAT', 'EKF_STATUS_REPORT']))

    def service(self, ignore_unhealthy=False):
        now = self.clock.now()
        if now >= self.deadline:
            raise Stop('Total experiment deadline')
        if now-self.last_heartbeat >= 1:
            self.send('HEARTBEAT')
            self.last_heartbeat = now
        if now-self.last_poll >= 1:
            for mid in READ_MESSAGES:
                self.send('REQUEST_MESSAGE', message_id=mid)
            self.send('PARAM_REQUEST_READ', name='STAT_BOOTCNT')
            self.last_poll = now
        try:
            message = self.transport.receive(.05)
        except OSError as error:
            self.journal.record('transport_error', direction='rx', error=repr(error))
            raise Stop('Transport read failure') from error
        if message:
            self.feed(message, not ignore_unhealthy)
        if self.initial_mode is not None and self.clock.now()-self.seen_at.get('HEARTBEAT', self.start) > 3:
            raise Stop('Stale heartbeat/link')
        if self.initial_identity is not None and self.clock.now()-self.seen_at.get('EKF_STATUS_REPORT', self.start) > 3:
            raise Stop('Stale EKF telemetry')

    def wait(self, predicate, timeout):
        end = min(self.deadline, self.clock.now()+timeout)
        while not predicate():
            if self.clock.now() >= end:
                raise Stop('Request/readiness timeout')
            self.service()

    def observe(self, seconds, label, ignore_unhealthy=False):
        self.journal.record('phase', name=label, counters=dict(self.counters))
        end = min(self.deadline, self.clock.now()+seconds)
        while self.clock.now() < end:
            self.service(ignore_unhealthy)

    def run(self):
        if self.used:
            raise Stop('An engine instance cannot repeat an experiment')
        self.used = True
        result = {'success': False, 'plan': self.plan, 'pm_nlon': None, 'flash_operation_us': None,
                  'clock_note': 'Host round trips include queues/transport; not flash or main mutex duration'}
        try:
            for name in PARAMETERS:
                self.send('PARAM_REQUEST_READ', name=name)
            self.wait(self.ready, 10)
            self.observe(self.plan['baseline_s'], 'BASELINE')
            self.baseline_loop_max = self.counters.get('UB29')
            result['baseline_cumulative_loop_max_us'] = self.baseline_loop_max
            if self.plan['scenario'] != 'idle':
                self.journal.record('phase', name='LIST')
                self.send('LOG_REQUEST_LIST', start=1, end=65535)
                self.wait(lambda: bool(self.entries) and len(self.entries) == next(iter(self.entries.values()))['num_logs'],
                          self.plan['request_timeout_s'])
            if self.plan['scenario'] in ['data', 'paced']:
                log = self.plan['log']
                entry = self.entries.get(log['id'])
                if entry is None or any(entry[k] != log[k] for k in ['size', 'time_utc']):
                    raise Stop('Selected log no longer matches archived identity')
                total = self.plan['chunk_bytes']*self.plan['chunks']
                identity = (log['id'], log['size'], log['time_utc'])
                self.transfer = TransferPlan(identity, total, self.plan['chunk_bytes'], self.plan['retries'])
                self.journal.record('phase', name='DATA')
                while (request := self.transfer.request(identity)) is not None:
                    lo, count = request
                    self.send('LOG_REQUEST_DATA', id=log['id'], ofs=lo, count=count)
                    end = min(self.deadline, self.clock.now()+self.plan['request_timeout_s'])
                    while self.transfer.pending is not None and self.clock.now() < end:
                        self.service()
                    if self.transfer.pending is not None:
                        self.journal.record('request_timeout', offset=lo, received=sum(self.transfer.seen),
                                            attempt=self.transfer.attempts)
                        # END the current request before a bounded retry; do not repeat the experiment.
                        self.send('LOG_REQUEST_END')
                    else:
                        self.observe(self.plan['pacing_s'], 'PACING')
            if self.plan['scenario'] != 'idle':
                self.send('LOG_REQUEST_END')
            self.observe(self.plan['recovery_s'], 'RECOVERY')
            result['success'] = True
        except (Stop, TimeoutError, ValueError, OSError, KeyboardInterrupt) as error:
            result['stop_reason'] = type(error).__name__+': '+str(error)
            self.journal.record('stop', reason=result['stop_reason'])
        finally:
            self.stopping = True
            try:
                if self.initial_identity is not None and not result['success'] and self.request_counts['LOG_REQUEST_LIST']:
                    self.send('LOG_REQUEST_END')
                    # Only observe recovery; no new DATA/LIST and no health bypass for further operations.
                    self.observe(min(self.plan['recovery_s'], max(0., self.deadline-self.clock.now()-.1)),
                                 'RECOVERY_AFTER_STOP', True)
            except (Stop, OSError, ValueError, KeyboardInterrupt):
                pass
            if self.transfer is not None:
                received = sum(self.transfer.seen)
                result.update(unique_received_bytes=received, planned_bytes=self.transfer.size,
                              partial_sha256=hashlib.sha256(self.transfer.data).hexdigest())
                if self.journal.directory:
                    (self.journal.directory/'partial.bin').write_bytes(self.transfer.data)
                    (self.journal.directory/'partial-validity.bin').write_bytes(self.transfer.seen)
            result.update(requests=self.request_counts, received_payload_bytes=self.rx_bytes,
                          last_messages=self.latest, counters=self.counters,
                          observed_source_identity=self.initial_identity, elapsed_s=self.clock.now()-self.start)
            result['boot_count'] = self.boot_count
            try:
                self.transport.close()
            except OSError as error:
                result['cleanup_error'] = repr(error)
                result['success'] = False
            finally:
                self.journal.close(result)
        return result


class RecordingWriter:
    def __init__(self, writer, stream):
        self.writer, self.stream = writer, stream

    def write(self, payload):
        result = self.writer.write(payload)
        if result != len(payload):
            raise OSError('Short/failed MAVLink serial write: '+str(result))
        self.stream.write(struct.pack('>Q', int(time.time()*1e6))+payload)
        self.stream.flush()
        return result


class LiveTransport:
    def __init__(self, plan, approval, directory):
        required = {'approved', 'props_removed', 'exclusive_port', 'port', 'expires_utc', 'plan_sha256'}
        digest = hashlib.sha256(json.dumps(plan, sort_keys=True).encode()).hexdigest()
        if set(approval) != required or not all(approval[k] is True for k in ['approved', 'props_removed', 'exclusive_port']):
            raise PermissionError('Separate physical experiment approval required')
        expiry = datetime.datetime.fromisoformat(approval['expires_utc'])
        if expiry.tzinfo is None or expiry <= datetime.datetime.now(datetime.timezone.utc):
            raise PermissionError('Approval expired/not timezone qualified')
        if approval['plan_sha256'] != digest or approval['port'] != 'COM5':
            raise PermissionError('Approval does not match exact plan/COM5')
        validate_plan(plan)
        from pymavlink import mavutil
        self.connection = mavutil.mavlink_connection(
            'COM5', baud=115200, source_system=254, source_component=190, autoreconnect=False)
        self.txlog = None
        try:
            self.connection.port.write_timeout = .5
            self.connection.logfile = (directory/'receive.tlog').open('xb')
            self.connection.logfile_raw = (directory/'receive-wire.bin').open('xb')
            self.txlog = (directory/'transmit.tlog').open('xb')
            self.connection.mav.file = RecordingWriter(self.connection.mav.file, self.txlog)
        except (OSError, AttributeError, KeyboardInterrupt):
            for stream in [self.connection.logfile, self.connection.logfile_raw, self.txlog]:
                if stream:
                    stream.close()
            self.connection.close()
            raise
        self.target = (plan['identity']['system'], plan['identity']['component'])
        self.mavutil = mavutil

    def send(self, kind, fields):
        mav, target = self.connection.mav, self.target
        if kind == 'HEARTBEAT':
            mav.heartbeat_send(6, 8, 0, 0, 0)
        elif kind == 'REQUEST_MESSAGE':
            mav.command_long_send(*target, 512, 0, fields['message_id'], 0, 0, 0, 0, 0, 0)
        elif kind == 'PARAM_REQUEST_READ':
            mav.param_request_read_send(*target, fields['name'].encode(), -1)
        elif kind == 'LOG_REQUEST_LIST':
            mav.log_request_list_send(*target, fields['start'], fields['end'])
        elif kind == 'LOG_REQUEST_DATA':
            mav.log_request_data_send(*target, fields['id'], fields['ofs'], fields['count'])
        elif kind == 'LOG_REQUEST_END':
            mav.log_request_end_send(*target)
        else:
            raise Stop('Forbidden live command')

    def receive(self, timeout):
        msg = self.connection.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        d = msg.to_dict()
        kind = d.pop('mavpackettype')
        return {'type': kind, 'source': [msg.get_srcSystem(), msg.get_srcComponent()], 'fields': d}

    def close(self):
        self.connection.logfile.close()
        self.connection.logfile_raw.close()
        self.txlog.close()
        self.connection.close()


def replay_archive(path, plan, directory):
    """Exercise real health messages with no sending and no simulated download."""
    if not path.is_file() or path.suffix.lower() != '.tlog':
        raise ValueError('Existing .tlog archive required; no device name accepted')
    from pymavlink import mavutil
    from utb_download_audit import parse_tlog

    class ArchiveClock:
        timestamp, elapsed = 0., 0.
        reconstructed = True

        def now(self):
            return self.elapsed

        def wall(self):
            return self.timestamp

    class NoSending:
        def send(self, *args):
            raise AssertionError('Replay must not send anything')

    clock = ArchiveClock()
    journal = Journal(clock, directory)
    engine = Engine(NoSending(), clock, plan, journal)
    reader = mavutil.mavlink_connection(str(path.resolve()))
    count, violations, last = 0, {}, None
    selected = {'HEARTBEAT', 'AUTOPILOT_VERSION', 'PARAM_VALUE', 'EKF_STATUS_REPORT',
                'STATUSTEXT', 'NAMED_VALUE_INT', 'SYS_STATUS', 'SYSTEM_TIME', 'BAD_DATA'}
    try:
        while (msg := reader.recv_match()) is not None:
            stamp = msg._timestamp
            if last is not None:
                clock.elapsed += max(0, stamp-last)
                if stamp < last:
                    journal.record('archive_wall_regression', previous=last, current=stamp)
            clock.timestamp, last = stamp, stamp
            d = msg.to_dict()
            kind = d.pop('mavpackettype')
            count += 1
            if kind in selected:
                try:
                    engine.feed(dict(type=kind, source=[msg.get_srcSystem(), msg.get_srcComponent()], fields=d))
                except Stop as error:
                    violations[str(error)] = violations.get(str(error), 0)+1
                    journal.record('replayed_guard_stop', reason=str(error))
    finally:
        reader.close()
    result = dict(archive=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(), messages=count,
                  guard_observations=violations, counters=engine.counters, transmitted_messages=0,
                  disclaimer='Archived receive timestamps; reconstructed elapsed time is not recorded host monotonic time. '
                             'Historical configuration may differ from the proposed frozen experiment.',
                  evidence=parse_tlog(path))
    journal.close(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--replay', type=Path, help='Existing archive tlog, no device access')
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--approval', type=Path)
    parser.add_argument('--archive', type=Path, help='Immutable BIN matching data scenario, checked before any port opens')
    args = parser.parse_args()
    plan = validate_plan(json.loads(args.plan.read_text()))
    if args.live:
        if args.replay or not args.approval:
            parser.error('Live operation requires a separate approval; cannot combine with replay')
        expected_segment = None
        if plan['scenario'] in ['data', 'paced']:
            if not args.archive or not args.archive.is_file():
                parser.error('Live data requires the previously archived BIN')
            payload = args.archive.read_bytes()
            if len(payload) != plan['log']['size'] or hashlib.sha256(payload).hexdigest() != plan['log']['archive_sha256']:
                parser.error('Archive size/hash mismatch')
            expected_segment = payload[:plan['chunk_bytes']*plan['chunks']]
        journal = Journal(Clock(), args.output)
        try:
            transport = LiveTransport(plan, json.loads(args.approval.read_text()), args.output)
        except (PermissionError, OSError, ValueError) as error:
            journal.record('preflight_abort', reason=repr(error))
            journal.close(dict(success=False, error=repr(error), hardware_experiment_started=False))
            raise
        result = Engine(transport, Clock(), plan, journal, expected_segment).run()
        if not result['success']:
            raise SystemExit(1)
    elif args.replay:
        replay_archive(args.replay, plan, args.output)
        print('Offline archived telemetry replay, not an executed experiment')
    else:
        parser.error('Select --replay; --live is gated by separate approval')


if __name__ == '__main__':
    main()
