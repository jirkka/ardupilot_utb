#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Approved DISARMED no-RC BENCH capture only; no upload/reboot/log transfer/erase or motor commands."""

# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses and copyright notices are retained.
# --------------------------------------------------------------------------------------------------------------------
# @file       utb_synthetic_hardware_capture.py
# @brief      Sériový no-RC DISARMED experiment guard; žádný upload/download/erase/motor API.
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
from pathlib import Path
import time

from serial.tools import list_ports

from utb_hardware_capture import Connection

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Include Files
# ====================================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------------
SHA = 'a45f3c9f08e21e8b713cfe59e3fe404fe4e866164c6c21f1a00edd0876622878'
GAINS = {f'UTB_RAT_{axis}_{key}': value for axis in ('RLL', 'PIT', 'YAW')
         for key, value in zip(('P', 'I', 'D', 'IMAX', 'D_HZ'), (.31, .23, .004, .19, 20))}
WRITABLE = set(GAINS) | {'UTB_SHADOW', 'UTB_B_SYNTH', 'UTB_LOG_FMT', 'UTB_LOG_RATE', 'LOG_DISARMED', 'FS_GCS_ENABLE'}


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Static data declarations
# ====================================================================================================================
# Data type, constant, macro definition
# --------------------------------------------------------------------------------------------------------------------
class BenchConnection(Connection):
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # End Data type, constant, macro definition
    # ====================================================================================================================
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Initialize the instance from the supplied configuration and collaborators.
    #
    # @param[in]  args Args.
    # @return     None.
    #
    # @details    Public class method. Calls: super().__init__, self.link.port.set_buffer_size, super.
    # --------------------------------------------------------------------------------------------------------------------
    def __init__(self, *args):
        self.original_mode = None
        self.stopping = False
        super().__init__(*args)
        self.link.port.set_buffer_size(rx_size=1048576, tx_size=65536)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Receive.
    #
    # @param[in]  None
    # @return     Returned values: msg; None.
    #
    # @details    Public class method. Calls: super().receive, msg.get_type, RuntimeError, msg.text.lower.
    # --------------------------------------------------------------------------------------------------------------------
    def receive(self):
        msg = super().receive()
        if msg is None:
            return None
        if self.stopping:
            return msg
        kind = msg.get_type()
        if kind == 'HEARTBEAT':
            if self.original_mode is None:
                self.original_mode = msg.custom_mode
            if msg.custom_mode != self.original_mode or msg.system_status in (5, 6, 7):
                raise RuntimeError('STOP: original mode changed or critical/emergency state')
        if kind == 'RC_CHANNELS' and msg.chancount != 0:
            raise RuntimeError('STOP: RC input appeared in no-RC experiment')
        if kind == 'EKF_STATUS_REPORT' and (not msg.flags & 1 or msg.flags & 1024):
            raise RuntimeError('STOP: EKF attitude unavailable/uninitialised')
        if kind == 'STATUSTEXT':
            text = msg.text.lower()
            if any(word in text for word in ('failsafe', 'unhealthy', 'ekf attitude is bad', 'unable to initialise')):
                raise RuntimeError('STOP: ' + msg.text)
        return msg

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Write.
    #
    # @param[in]  name Name.
    # @param[in]  value Value supplied to this operation.
    # @return     Returned values: actual.
    #
    # @details    Public class method. Calls: self.parameter, ValueError, abs, RuntimeError.
    # --------------------------------------------------------------------------------------------------------------------
    def write(self, name, value):
        if name not in WRITABLE:
            raise ValueError('Non-diagnostic write prohibited: ' + name)
        self.parameter(name, value)
        actual = self.parameter(name)
        if abs(actual - value) > 1e-6:
            raise RuntimeError('Explicit parameter readback mismatch: ' + name)
        return actual

    # --------------------------------------------------------------------------------------------------------------------
    # @brief      Settle.
    #
    # @param[in]  seconds Seconds.
    # @return     None.
    #
    # @details    Public class method. Calls: time.monotonic, self.receive.
    # --------------------------------------------------------------------------------------------------------------------
    def settle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.receive()


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# End Public functions
# ====================================================================================================================
# Public functions
# --------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------
# @brief      Check the observed state and freshness required for a DISARMED diagnostic experiment.
#
# @param[in]  connection Connection.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: bool, time.monotonic.
# --------------------------------------------------------------------------------------------------------------------
def state_ready(connection):
    health = connection.monitor.health
    return (health is not None and time.monotonic() - connection.monitor.health_at < 7
            and bool(health['flags'] & 4) and health['calc'] != 2 and health['policy'] in (0, 1))


# --------------------------------------------------------------------------------------------------------------------
# @brief      Recognize terminal experiment metadata without declaring unverified data complete.
#
# @param[in]  connection Connection.
# @param[in]  elapsed Elapsed.
# @return     Computed result of this operation.
#
# @details    Global module function. Calls: bool.
# --------------------------------------------------------------------------------------------------------------------
def completion_candidate(connection, elapsed):
    # Generator status exists in BIN UTBX only. Completion must be confirmed offline.
    health = connection.monitor.health
    return (elapsed >= 20.8 and health is not None and health['calc'] == 4 and health['policy'] == 0
            and bool(health['flags'] & 4) and not health['flags'] & 64)


# --------------------------------------------------------------------------------------------------------------------
# @brief      Parse command-line arguments and execute this module's documented operation.
#
# @param[in]  None
# @return     None.
#
# @details    Global module function. Calls: argparse.ArgumentParser, parser.add_argument, parser.parse_args, any.
# --------------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--apj', type=Path, required=True)
    parser.add_argument('--backup', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--kind', choices=('short', 'full'), required=True)
    parser.add_argument('--props-removed', action='store_true', required=True)
    parser.add_argument('--approved-gcs-enable', action='store_true', required=True)
    args = parser.parse_args()
    assert hashlib.sha256(args.apj.read_bytes()).hexdigest() == SHA
    assert json.loads(args.apj.read_text())['board_id'] == 1075
    assert any(p.device == args.port and p.serial_number == args.serial and p.vid == 0x1209
               for p in list_ports.comports()), 'Physical identity mismatch'
    before = json.loads(args.backup.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(start=time.time(), kind=args.kind, firmware_sha256=SHA, success=False)
    connection, changed, verified = None, [], False
    try:
        connection = BenchConnection(args.port, args.output)
        connection.wait(lambda: 'HEARTBEAT' in connection.latest, 8)
        connection.request(148)
        connection.wait(lambda: 'AUTOPILOT_VERSION' in connection.latest, 5)
        version = connection.latest['AUTOPILOT_VERSION']
        assert version['board_version'] >> 16 == 1075
        assert bytes(version['flight_custom_version']).decode('ascii') == 'f9cc3bcc'
        required = dict(UTB_ENABLE=1, UTB_SHADOW=0, UTB_B_SYNTH=0, LOG_DISARMED=0,
                        FRAME_CLASS=1, FRAME_TYPE=18, SCHED_LOOP_RATE=400, FSTRATE_ENABLE=0,
                        LOG_BACKEND_TYPE=4, FS_THR_ENABLE=1, FS_EKF_ACTION=1, FS_GCS_ENABLE=0,
                        LOG_BITMASK=180222, LOG_BLK_RATEMAX=0, LOG_DARM_RATEMAX=0, LOG_FILE_BUFSIZE=172)
        result['before'] = {k: connection.parameter(k) for k in set(required) | WRITABLE}
        assert all(result['before'][k] == v for k, v in required.items()), result['before']
        assert all(result['before'][k] == before[k] for k in GAINS), 'Gains differ from original backup'
        connection.request(65)
        connection.wait(lambda: 'RC_CHANNELS' in connection.latest, 5)
        assert connection.latest['RC_CHANNELS']['chancount'] == 0
        for msgid in (1, 193):
            connection.request(msgid)
        connection.wait(lambda: 'EKF_STATUS_REPORT' in connection.latest, 5)
        verified = True
        settings = dict(GAINS, FS_GCS_ENABLE=1, UTB_LOG_FMT=1, UTB_LOG_RATE=400)
        for name, value in settings.items():
            changed.append(name)
            connection.write(name, value)
        changed.append('UTB_SHADOW')
        connection.write('UTB_SHADOW', 1)
        connection.monitor.health = None
        connection.wait(lambda: state_ready(connection), 12)
        result['actual_state_gate'] = dict(connection.monitor.health)
        changed.append('LOG_DISARMED')
        connection.write('LOG_DISARMED', 1)
        logger_start = time.monotonic()
        connection.settle(15)  # Include startup and a complete PM interval before source activation.
        if not state_ready(connection):
            raise RuntimeError('STOP: actual state invalid after logger warmup')
        metrics = connection.monitor.metrics
        assert metrics is not None and all(metrics[k] == v for k, v in {10: 3, 18: 3, 33: 4, 35: 0, 36: 1}.items())
        assert metrics[16] not in (0, 0xffffffff), 'Unknown/zero worker watermark'
        result['warmup_baseline'] = dict(metrics)
        changed.append('UTB_B_SYNTH')
        active_start = time.monotonic()
        connection.write('UTB_B_SYNTH', 1)
        connection.monitor.health = None
        connection.wait(lambda: connection.monitor.ready(time.monotonic()), 5)
        baseline = dict(connection.monitor.metrics)
        result['active_baseline'] = baseline
        result['active_start_wall'] = time.time() - (time.monotonic() - active_start)
        while True:
            connection.receive()
            elapsed = time.monotonic() - active_start
            if args.kind == 'short' and elapsed >= 6:
                break
            if args.kind == 'full' and completion_candidate(connection, elapsed):
                result['completion_candidate'] = True
                break
            connection.monitor.check_window(baseline, time.monotonic())
            if elapsed > 25 or time.monotonic() - logger_start > 42:
                raise RuntimeError('STOP: finite experiment deadline exceeded')
        result['active_elapsed_wall_s'] = time.monotonic() - active_start
        result['active_final'] = dict(connection.monitor.metrics)
        result['end_health'] = connection.monitor.health
        result['success'] = True
    except Exception as exc:
        result['error'] = str(exc)
        raise
    finally:
        if connection and verified:
            connection.stopping = True
            result['freeze'], result['restore'], result['cleanup_errors'] = {}, {}, {}
            for name in ('UTB_B_SYNTH', 'UTB_SHADOW', 'LOG_DISARMED'):
                try:
                    result['freeze'][name] = connection.write(name, 0)
                except Exception as exc:
                    result['cleanup_errors'][name] = str(exc)
            for name in reversed(changed):
                if name in result['freeze'] or name in result['restore']:
                    continue
                try:
                    result['restore'][name] = connection.write(name, result['before'][name])
                except Exception as exc:
                    result['cleanup_errors'][name] = str(exc)
        if connection:
            result.update(events=connection.events, telemetry=connection.records, latest=connection.latest)
            if connection.link.logfile:
                connection.link.logfile.close()
                connection.link.logfile = None
            connection.link.close()
        result['end'] = time.time()
        (args.output / 'capture.json').write_text(json.dumps(result, indent=2, default=list) + '\n')
        if result.get('cleanup_errors'):
            raise RuntimeError('Cleanup incomplete; inspect capture.json')


if __name__ == '__main__':
    main()

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
