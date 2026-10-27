#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @author Jiri Kovar
# @version 2026-10-25
"""Bounded USB metadata discovery after an operator-confirmed reconnect; never opens a port."""
import argparse
import datetime
import json
import math
import time
from pathlib import Path

USB_SERIAL = '200024001251303433333336'
USB_VID = 0x1209
USB_PID = 0x5741


def classify(ports, serial_number=USB_SERIAL):
    """Select by USB serial first; a COM number alone never establishes device identity."""
    matches = [dict(row) for row in ports if row.get('serial') == serial_number]
    if not matches:
        return dict(status='absent', port=None, candidates=[])
    if len(matches) != 1:
        return dict(status='ambiguous', port=None, candidates=matches)
    row = matches[0]
    if (row.get('vid'), row.get('pid')) != (USB_VID, USB_PID):
        return dict(status='unexpected_mode', port=None, candidates=matches)
    if not isinstance(row.get('port'), str) or not row['port']:
        return dict(status='ambiguous', port=None, candidates=matches)
    return dict(status='present', port=row['port'], candidates=matches)


def enumerate_ports():
    """Read OS port metadata only; no serial.Serial or MAVLink connection is created."""
    from serial.tools import list_ports
    return [dict(port=row.device, vid=row.vid, pid=row.pid, serial=row.serial_number)
            for row in list_ports.comports()]


def discover(evidence, confirmed, wait_seconds=0, scan=enumerate_ports, sleep=time.sleep):
    """Save one exclusive discovery attempt; optional absent-device polling is capped at ten seconds."""
    if not confirmed:
        raise ValueError('Explicit operator reconnect confirmation is required')
    if not math.isfinite(wait_seconds) or not 0 <= wait_seconds <= 10:
        raise ValueError('Passive enumeration wait must be between zero and ten seconds')
    result = dict(schema='UTBUSBDiscoveryV1', operator_confirmation_provided=True,
                  samples=[], status='error', port=None, serial_port_opened=False)
    # Exclusive creation prevents accidentally overwriting an earlier attempt or its error.
    with Path(evidence).open('x', encoding='utf-8') as stream:
        try:
            for attempt in range(int(wait_seconds) + 1):
                ports = scan()
                selection = classify(ports)
                result['samples'].append(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                                              host_counter_ns=time.perf_counter_ns(), ports=ports, selection=selection))
                result.update(status=selection['status'], port=selection['port'])
                if selection['status'] != 'absent' or attempt == int(wait_seconds):
                    break
                sleep(1)
        except Exception as error:
            result.update(status='error', port=None, error=repr(error))
        finally:
            json.dump(result, stream, indent=2)
            stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirmed-reconnect', action='store_true', required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--wait-seconds', type=float, default=0)
    args = parser.parse_args()
    result = discover(args.evidence, args.confirmed_reconnect, args.wait_seconds)
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'present' else 2


if __name__ == '__main__':
    raise SystemExit(main())
