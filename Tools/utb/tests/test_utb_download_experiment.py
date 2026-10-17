#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""1H-E experiment tests exclusively with a mock peer/clock or archived files."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       test_utb_download_experiment.py
# @brief      Unit regrese: 1H-E experiment tests exclusively with a mock peer/clock or archived files.
# @author     Jiri Kovar
# @version    2026-10-17
# @date       2026-10-17
# --------------------------------------------------------------------------------------------------------------------

# ====================================================================================================================
# Include Files
# --------------------------------------------------------------------------------------------------------------------
import copy
import datetime
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import struct
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from utb_download_experiment import (Engine, Journal, LiveTransport, PARAMETERS,  # noqa: E402
                                     RecordingWriter, Stop, replay_archive)

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
IDENTITY = dict(system=1, component=1, board_id=1075, vendor_id=4617, product_id=22337,
                uid2=[32, 0, 36, 0, 18, 81, 48, 52, 51, 51, 51, 54, 0, 0, 0, 0, 0, 0],
                flight_custom_version='f9cc3bcc', flight_sw_version=67568127)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Plan.
#
# @param[in]  scenario Scenario.
# @param[in]  chunks Chunks.
# @param[in]  retries Retries.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: dict, copy.deepcopy.
# --------------------------------------------------------------------------------------------------------------------
def plan(scenario='data', chunks=1, retries=0):
    return dict(scenario=scenario, identity=copy.deepcopy(IDENTITY), baseline_s=.1, recovery_s=.1,
                duration_limit_s=12, chunk_bytes=180, chunks=chunks, pacing_s=.05,
                request_timeout_s=.4, retries=retries,
                log=dict(id=2, size=9000, time_utc=123, archive_sha256='a'*64) if scenario in ['data', 'paced'] else None)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class MockClock:
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self):
        self.value, self.wall_jump = 0., 0.

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Now.
    #
    # @param[in]  None
    # @return     Returned values: self.value.
    #
    # @details    Public class method.
    # --------------------------------------------------------------------------------------------------------------------
    def now(self):
        return self.value

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Wall.
    #
    # @param[in]  None
    # @return     Returned values: 1791900000 + self.value + self.wall_jump.
    #
    # @details    Public class method.
    # --------------------------------------------------------------------------------------------------------------------
    def wall(self):
        return 1791900000+self.value+self.wall_jump


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class Peer:
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  clock Clock.
    # @return     None.
    #
    # @details    Public class method. Calls: bytes, range.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self, clock):
        self.clock, self.queue, self.sent = clock, [], []
        self.payload = bytes(i % 256 for i in range(9000))
        self.closed, self.behaviour, self.attempts = False, 'normal', 0

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Message.
    #
    # @param[in]  kind Kind.
    # @param[in]  fields Fields.
    # @return     None.
    #
    # @details    Public class method. Calls: self.queue.append, dict.
    # --------------------------------------------------------------------------------------------------------------------
    def message(self, kind, **fields):
        self.queue.append(dict(type=kind, source=[1, 1], fields=fields))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Send.
    #
    # @param[in]  kind Kind.
    # @param[in]  fields Fields.
    # @return     None.
    #
    # @details    Public class method. Calls: self.sent.append, self.message, reversed, dict.
    # --------------------------------------------------------------------------------------------------------------------
    def send(self, kind, fields):
        self.sent.append((kind, dict(fields)))
        if kind == 'HEARTBEAT':
            self.message('HEARTBEAT', type=2, autopilot=3, base_mode=81, custom_mode=0, system_status=3)
        if kind == 'PARAM_REQUEST_READ':
            self.message('PARAM_VALUE', param_id=fields['name'], param_value=PARAMETERS.get(fields['name'], 74))
        if kind == 'REQUEST_MESSAGE':
            if fields['message_id'] == 148:
                self.message('AUTOPILOT_VERSION', vendor_id=4617, product_id=22337, uid2=IDENTITY['uid2'],
                             flight_sw_version=67568127, board_version=1075 << 16,
                             flight_custom_version=list(b'f9cc3bcc'))
            elif fields['message_id'] == 193:
                self.message('EKF_STATUS_REPORT', flags=167)
            else:
                self.message('SYS_STATUS', load=200)
            self.message('NAMED_VALUE_INT', name='UB29', value=2600, time_boot_ms=int(self.clock.now()*1000))
        if kind == 'LOG_REQUEST_LIST':
            self.message('LOG_ENTRY', id=1, size=9000, time_utc=122, num_logs=2, last_log_num=2)
            self.message('LOG_ENTRY', id=2, size=9000, time_utc=123, num_logs=2, last_log_num=2)
        if kind == 'LOG_REQUEST_DATA':
            self.attempts += 1
            if self.behaviour in ['timeout', 'disconnect']:
                return
            if self.behaviour == 'ekf':
                self.message('EKF_STATUS_REPORT', flags=1024)
                return
            if self.behaviour == 'armed':
                self.message('HEARTBEAT', type=2, autopilot=3, base_mode=209, custom_mode=0, system_status=3)
                return
            if self.behaviour == 'interrupt':
                raise KeyboardInterrupt('MOCK manual cancellation')
            if self.behaviour == 'truncated':
                self.message('LOG_DATA', id=2, ofs=0, count=90, data=[1]*20)
                return
            if self.behaviour == 'loop-spike':
                self.message('NAMED_VALUE_INT', name='UB29', value=40000, time_boot_ms=int(self.clock.now()*1000))
                return
            start, count = fields['ofs'], fields['count']
            for offset in reversed(range(start, start+count, 90)):
                if self.behaviour == 'missing-first' and self.attempts == 1 and offset == start:
                    continue
                payload = self.payload[offset:offset+min(90, start+count-offset)]
                self.message('LOG_DATA', id=2, ofs=offset, count=len(payload), data=list(payload))
                if self.behaviour == 'duplicate':
                    self.message('LOG_DATA', id=2, ofs=offset, count=len(payload), data=list(payload))
                if self.behaviour == 'conflict':
                    self.message('LOG_DATA', id=2, ofs=offset, count=len(payload), data=[255]*len(payload))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Receive.
    #
    # @param[in]  timeout Timeout.
    # @return     Returned values: self.queue.pop(0) if self.queue else None.
    #
    # @details    Public class method. Calls: OSError, self.queue.pop.
    # --------------------------------------------------------------------------------------------------------------------
    def receive(self, timeout):
        self.clock.value += .01 if self.queue else timeout
        if self.behaviour == 'disconnect' and self.attempts:
            raise OSError('MOCK disconnected')
        return self.queue.pop(0) if self.queue else None

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Release the resources owned by this instance.
    #
    # @param[in]  None
    # @return     None.
    #
    # @details    Public class method.
    # --------------------------------------------------------------------------------------------------------------------
    def close(self):
        self.closed = True


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class DownloadExperimentTests(unittest.TestCase):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Engine.
    #
    # @param[in]  scenario Scenario.
    # @param[in]  chunks Chunks.
    # @param[in]  retries Retries.
    # @param[in]  behaviour Behaviour.
    # @param[in]  directory Experiment input/output directory.
    # @return     Returned values: (engine, peer, clock, journal).
    #
    # @details    Public class method. Calls: MockClock, Peer, Journal, Engine.
    # --------------------------------------------------------------------------------------------------------------------
    def engine(self, scenario='data', chunks=1, retries=0, behaviour='normal', directory=None):
        clock = MockClock()
        peer = Peer(clock)
        peer.behaviour = behaviour
        journal = Journal(clock, directory)
        engine = Engine(peer, clock, plan(scenario, chunks, retries), journal)
        return engine, peer, clock, journal

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify idle has no log requests or mutation.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertTrue, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_idle_has_no_log_requests_or_mutation(self):
        engine, peer, _, _ = self.engine('idle')
        result = engine.run()
        self.assertTrue(result['success'])
        self.assertEqual(sum(result['requests'].values()), 0)
        self.assertTrue(peer.closed)
        self.assertTrue(all(k in ['HEARTBEAT', 'REQUEST_MESSAGE', 'PARAM_REQUEST_READ'] for k, _ in peer.sent))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify listing only no data.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertTrue, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_listing_only_no_data(self):
        engine, _, _, _ = self.engine('list')
        result = engine.run()
        self.assertTrue(result['success'])
        self.assertEqual(result['requests']['LOG_REQUEST_LIST'], 1)
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 0)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify bounded data out of order.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertTrue, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_bounded_data_out_of_order(self):
        engine, peer, _, _ = self.engine()
        result = engine.run()
        self.assertTrue(result['success'])
        self.assertEqual(result['unique_received_bytes'], 180)
        self.assertEqual(engine.transfer.data, peer.payload[:180])
        self.assertIsNone(result['flash_operation_us'])
        self.assertIsNone(result['pm_nlon'])

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify paced requests.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertTrue, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_paced_requests(self):
        engine, _, _, journal = self.engine('paced', chunks=3)
        result = engine.run()
        self.assertTrue(result['success'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 3)
        sends = [r for r in journal.events if r['event'] == 'tx_sent' and r['kind'] == 'LOG_REQUEST_DATA']
        self.assertTrue(all(b['host_monotonic_s']-a['host_monotonic_s'] >= .05 for a, b in zip(sends, sends[1:])))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify duplicates including completed chunk.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertTrue, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_duplicates_including_completed_chunk(self):
        engine, _, _, journal = self.engine('paced', chunks=2, behaviour='duplicate')
        result = engine.run()
        self.assertTrue(result['success'])
        self.assertTrue(any(e['event'] == 'duplicate_data' for e in journal.events))
        self.assertEqual(result['unique_received_bytes'], 360)
        self.assertGreater(result['received_payload_bytes'], 360)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify missing data bounded retry.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertTrue, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_missing_data_bounded_retry(self):
        engine, _, _, journal = self.engine(retries=1, behaviour='missing-first')
        result = engine.run()
        self.assertTrue(result['success'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 2)
        self.assertTrue(any(e['event'] == 'request_timeout' for e in journal.events))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify timeout does not repeat experiment.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertFalse, self.assertEqual.
    # --------------------------------------------------------------------------------------------------------------------
    def test_timeout_does_not_repeat_experiment(self):
        engine, peer, _, _ = self.engine(behaviour='timeout')
        result = engine.run()
        self.assertFalse(result['success'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 1)
        self.assertTrue(peer.closed)
        with self.assertRaises(Stop):
            engine.run()

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify transport loss saved separately.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertFalse, self.assertIn.
    # --------------------------------------------------------------------------------------------------------------------
    def test_transport_loss_saved_separately(self):
        engine, peer, _, journal = self.engine(behaviour='disconnect')
        result = engine.run()
        self.assertFalse(result['success'])
        self.assertIn('Transport read failure', result['stop_reason'])
        self.assertTrue(any(e['event'] == 'transport_error' for e in journal.events))
        self.assertTrue(peer.closed)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unhealthy stops data even with retries.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertFalse, self.assertIn.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unhealthy_stops_data_even_with_retries(self):
        engine, _, _, _ = self.engine(retries=2, behaviour='ekf')
        result = engine.run()
        self.assertFalse(result['success'])
        self.assertIn('EKF unhealthy', result['stop_reason'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 1)
        self.assertFalse(engine.ready())

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify armed stops.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertFalse, self.assertIn.
    # --------------------------------------------------------------------------------------------------------------------
    def test_armed_stops(self):
        engine, _, _, _ = self.engine(behaviour='armed')
        result = engine.run()
        self.assertFalse(result['success'])
        self.assertIn('ARMED', result['stop_reason'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 1)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unexpected heartbeat.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, self.assertRaises, engine.feed, dict.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unexpected_heartbeat(self):
        engine, _, _, _ = self.engine()
        with self.assertRaises(Stop):
            engine.feed(dict(type='HEARTBEAT', source=[9, 1], fields={}))
        with self.assertRaises(Stop):
            engine.feed(dict(type='HEARTBEAT', source=[1, 2], fields={'type': 2, 'autopilot': 3}))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify missing identity extension fails closed.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, peer.send, peer.queue.pop, self.assertRaisesRegex.
    # --------------------------------------------------------------------------------------------------------------------
    def test_missing_identity_extension_fails_closed(self):
        engine, peer, _, _ = self.engine('idle')
        peer.send('REQUEST_MESSAGE', {'message_id': 148})
        msg = peer.queue.pop(0)
        del msg['fields']['uid2']
        with self.assertRaisesRegex(Stop, 'MAVLink2 UID2 required'):
            engine.feed(msg)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify identity change.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, peer.send, peer.queue.pop, engine.feed.
    # --------------------------------------------------------------------------------------------------------------------
    def test_identity_change(self):
        engine, peer, _, _ = self.engine('idle')
        peer.send('REQUEST_MESSAGE', {'message_id': 148})
        msg = peer.queue.pop(0)
        engine.feed(msg)
        msg['fields']['uid2'] = [0]*18
        with self.assertRaises(Stop):
            engine.feed(msg)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify boot count change.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, dict, engine.feed, self.assertRaises.
    # --------------------------------------------------------------------------------------------------------------------
    def test_boot_count_change(self):
        engine, _, _, _ = self.engine()
        msg = dict(type='PARAM_VALUE', source=[1, 1], fields=dict(param_id='STAT_BOOTCNT', param_value=74))
        engine.feed(msg)
        msg['fields']['param_value'] = 75
        with self.assertRaises(Stop):
            engine.feed(msg)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify wall clock jump does not define deadline.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, self.assertTrue, self.assertLess, clock.now.
    # --------------------------------------------------------------------------------------------------------------------
    def test_wall_clock_jump_does_not_define_deadline(self):
        engine, _, clock, _ = self.engine('idle')
        clock.wall_jump = -3600
        self.assertTrue(engine.run()['success'])
        self.assertLess(clock.now(), engine.plan['duration_limit_s'])

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify forbidden mutation.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, self.assertRaises, engine.send.
    # --------------------------------------------------------------------------------------------------------------------
    def test_forbidden_mutation(self):
        engine, _, _, _ = self.engine()
        for kind in ['PARAM_SET', 'ARM', 'RC_OVERRIDE', 'LOG_ERASE', 'REBOOT', 'MOTOR_TEST']:
            with self.assertRaises(Stop):
                engine.send(kind)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify unapproved live never opens.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: patch, connect.assert_not_called, self.assertRaises, LiveTransport.
    # --------------------------------------------------------------------------------------------------------------------
    def test_unapproved_live_never_opens(self):
        with patch('pymavlink.mavutil.mavlink_connection') as connect:
            with self.assertRaises(PermissionError):
                LiveTransport(plan(), {}, Path('unused'))
            connect.assert_not_called()

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify transmit capture uses actual mavlink api.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: mavutil.mavlink.MAVLink, mav.heartbeat_send, self.assertEqual, io.BytesIO.
    # --------------------------------------------------------------------------------------------------------------------
    def test_transmit_capture_uses_actual_mavlink_api(self):
        from pymavlink import mavutil
        wire, log = io.BytesIO(), io.BytesIO()
        mav = mavutil.mavlink.MAVLink(RecordingWriter(wire, log), srcSystem=254, srcComponent=190)
        mav.heartbeat_send(6, 8, 0, 0, 0)
        self.assertEqual(log.getvalue()[8:], wire.getvalue())

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify partial capture saved and existing archive untouched.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: tempfile.TemporaryDirectory, Path, raw.write_bytes,
    #             hashlib.sha256(raw.read_bytes()).hexdigest.
    # --------------------------------------------------------------------------------------------------------------------
    def test_partial_capture_saved_and_existing_archive_untouched(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            raw = root/'raw.bin'
            raw.write_bytes(b'UNTOUCHED RAW')
            before = hashlib.sha256(raw.read_bytes()).hexdigest()
            engine, _, _, _ = self.engine(behaviour='timeout', directory=root/'session')
            self.assertFalse(engine.run()['success'])
            self.assertEqual(hashlib.sha256(raw.read_bytes()).hexdigest(), before)
            summary = json.loads((root/'session/summary.json').read_text())
            self.assertEqual(summary['unique_received_bytes'], 0)
            self.assertTrue((root/'session/partial-validity.bin').exists())
            self.assertTrue((root/'session/timeline.jsonl').exists())

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify truncated packet and conflicting duplicate stop.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.subTest, self.engine, engine.run, self.assertFalse.
    # --------------------------------------------------------------------------------------------------------------------
    def test_truncated_packet_and_conflicting_duplicate_stop(self):
        for behaviour, reason in [('truncated', 'truncated wire'), ('conflict', 'Conflicting duplicate')]:
            with self.subTest(behaviour=behaviour):
                engine, peer, _, _ = self.engine(behaviour=behaviour)
                result = engine.run()
                self.assertFalse(result['success'])
                self.assertIn(reason, result['stop_reason'])
                self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 1)
                self.assertTrue(peer.closed)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify manual cancellation ends request without restart.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertFalse, self.assertIn.
    # --------------------------------------------------------------------------------------------------------------------
    def test_manual_cancellation_ends_request_without_restart(self):
        engine, peer, _, journal = self.engine(behaviour='interrupt')
        result = engine.run()
        self.assertFalse(result['success'])
        self.assertIn('KeyboardInterrupt', result['stop_reason'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 1)
        self.assertGreaterEqual(result['requests']['LOG_REQUEST_END'], 1)
        self.assertTrue(peer.closed)
        self.assertTrue(any(r['event'] == 'stop' for r in journal.events))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify stale heartbeat stops.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.feed, dict, self.assertRaisesRegex.
    # --------------------------------------------------------------------------------------------------------------------
    def test_stale_heartbeat_stops(self):
        engine, _, clock, _ = self.engine('idle')
        engine.feed(dict(type='HEARTBEAT', source=[1, 1], fields=dict(type=2, autopilot=3,
                         base_mode=81, custom_mode=0, system_status=3)))
        clock.value = 4
        engine.last_heartbeat = engine.last_poll = 4
        with self.assertRaisesRegex(Stop, 'Stale heartbeat'):
            engine.service()

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify bad wire bytes preserved as hex.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: tempfile.TemporaryDirectory, self.engine, journal.close, json.loads.
    # --------------------------------------------------------------------------------------------------------------------
    def test_bad_wire_bytes_preserved_as_hex(self):
        with tempfile.TemporaryDirectory() as name:
            engine, _, _, journal = self.engine(directory=Path(name)/'session')
            with self.assertRaises(Stop):
                engine.feed(dict(type='BAD_DATA', source=[0, 0], fields={'data': b'\xff\x00'}))
            journal.close({})
            row = json.loads((Path(name)/'session/timeline.jsonl').read_text())
            self.assertEqual(row['fields']['data']['wire_bytes_hex'], 'ff00')

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify expired or mismatched approval never opens port.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: dict, patch, (datetime.datetime.now(datetime.timezone.utc) +
    #             datetime.timedelta(minutes=1)).isoformat, connect.assert_not_called.
    # --------------------------------------------------------------------------------------------------------------------
    def test_expired_or_mismatched_approval_never_opens_port(self):
        approval = dict(approved=True, props_removed=True, exclusive_port=True, port='COM5',
                        expires_utc='2000-01-01T00:00:00+00:00', plan_sha256='0'*64)
        with patch('pymavlink.mavutil.mavlink_connection') as connect:
            with self.assertRaises(PermissionError):
                LiveTransport(plan(), approval, Path('unused'))
            approval['expires_utc'] = (datetime.datetime.now(datetime.timezone.utc)
                                       + datetime.timedelta(minutes=1)).isoformat()
            with self.assertRaises(PermissionError):
                LiveTransport(plan(), approval, Path('unused'))
            connect.assert_not_called()

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify failed mavserial write does not record sent packet.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: type('FailedWriter', (), {'write': lambda self, payload: -1}), io.BytesIO,
    #             self.assertEqual, type.
    # --------------------------------------------------------------------------------------------------------------------
    def test_failed_mavserial_write_does_not_record_sent_packet(self):
        writer = type('FailedWriter', (), {'write': lambda self, payload: -1})()
        log = io.BytesIO()
        with self.assertRaises(OSError):
            RecordingWriter(writer, log).write(b'packet')
        self.assertEqual(log.getvalue(), b'')

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify configuration guard and archive prefix.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, bytes, self.assertIn, self.assertRaises.
    # --------------------------------------------------------------------------------------------------------------------
    def test_configuration_guard_and_archive_prefix(self):
        engine, _, _, _ = self.engine()
        with self.assertRaises(Stop):
            engine.feed(dict(type='PARAM_VALUE', source=[1, 1], fields=dict(param_id='UTB_SHADOW', param_value=1)))
        engine, _, _, _ = self.engine()
        engine.expected_segment = bytes(180)
        self.assertIn('Archived log prefix mismatch', engine.run()['stop_reason'])

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify new cumulative loop max prevents next request.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: self.engine, engine.run, self.assertFalse, self.assertIn.
    # --------------------------------------------------------------------------------------------------------------------
    def test_new_cumulative_loop_max_prevents_next_request(self):
        engine, _, _, _ = self.engine('paced', chunks=3, behaviour='loop-spike')
        result = engine.run()
        self.assertFalse(result['success'])
        self.assertIn('New cumulative main-loop', result['stop_reason'])
        self.assertEqual(result['requests']['LOG_REQUEST_DATA'], 1)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Verify archived wire health replay has no transmissions.
    #
    # @param[in]  None
    # @return     None; assertion failure reports a violated test condition.
    #
    # @details    Public class method. Calls: tempfile.TemporaryDirectory, Path, mavutil.mavlink.MAVLink, b''.join.
    # --------------------------------------------------------------------------------------------------------------------
    def test_archived_wire_health_replay_has_no_transmissions(self):
        from pymavlink import mavutil
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            archive = root/'mock-archive.tlog'
            mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            messages = [mavutil.mavlink.MAVLink_heartbeat_message(2, 3, 81, 0, 3, 3),
                        mavutil.mavlink.MAVLink_ekf_status_report_message(167, 0, 0, 0, 0, 0),
                        mavutil.mavlink.MAVLink_ekf_status_report_message(1024, 0, 0, 0, 0, 0)]
            wire = b''.join(struct.pack('>Q', 1791900000000000+i*1000000)+msg.pack(mav)
                            for i, msg in enumerate(messages))
            archive.write_bytes(wire)
            result = replay_archive(archive, plan('idle'), root/'output')
            self.assertEqual(result['messages'], 3)
            self.assertEqual(result['transmitted_messages'], 0)
            self.assertIn('EKF unhealthy/uninitialized', result['guard_observations'])
            self.assertEqual(archive.read_bytes(), wire)


if __name__ == '__main__':
    unittest.main()

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
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
