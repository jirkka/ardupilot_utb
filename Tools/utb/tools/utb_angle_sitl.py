#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Finite pilot ANGLE experiments; owns a local SITL, accepts no external endpoint."""
# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses remain unchanged.
# @file utb_angle_sitl.py
# @brief Skutečné RC override vstupy do vlastního lokálního SITL procesu a analýza ANGLE.
# @author Jiri Kovar
# @version 2026-10-17
# @date 2026-10-17
# --------------------------------------------------------------------------------------------------------------------
# Include Files
import argparse
import bisect
from functools import partial
import hashlib
import json
import math
import os
from pathlib import Path
import select
import sys
import socket
import struct
import time

from pymavlink import mavutil
from test_utb_active_sitl import ActiveSITL, BASE, json_safe
from test_utb_active_sitl import assemble
from utb_attitude_sitl import angle_metrics

# End Include Files
# Static data declarations
PARAMS = BASE + ('UTB_A_ENABLE 1\nUTB_A_ANGLE 5\nUTB_A_YMAX 0.1\nUTB_A_AW 0\n'
                 'FLTMODE2 42\nRC_OVERRIDE_TIME 0.5\nSCR_ENABLE 0\nFS_OPTIONS 0\n')
for _channel in range(1, 9):
    PARAMS += (f'RC{_channel}_MIN 1000\nRC{_channel}_MAX 2000\nRC{_channel}_TRIM 1500\n'
               f'RC{_channel}_DZ {0 if _channel == 3 else 30}\nRC{_channel}_REVERSED 0\n')
KINDS = ('UARC', 'UARX', 'UAMT', 'UAFB', 'UALT', 'UAAT', 'UAST', 'RCIN', 'RCOU', 'ATT', 'RATE',
         'PARM', 'MODE', 'PM', 'EV', 'ERR', 'MSG', 'DSF', 'PIDR', 'PIDP', 'PIDY')
# End Static data declarations
# Data type, constant, macro definition


class AngleSITL(ActiveSITL):
    # Public functions
    # --------------------------------------------------------------------------------------------------------------
    # @brief Připravit mapování kanálů a samostatné řízení RC a GCS spojení.
    #
    # @param[in] rcmap Pořadí skutečných RC kanálů roll, pitch, yaw, throttle.
    # @param[in] args Argumenty původního SITL runneru.
    # @param[in] kwargs Pojmenované argumenty původního runneru.
    # @return None.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def __init__(self, *args, rcmap=(1, 2, 4, 3), **kwargs):
        self.rcmap = rcmap
        self.send_rc = self.send_heartbeat = True
        self.expected_mode = None
        self.bench_altitude = None
        self.telemetry = {}
        self.native_rc = False
        self.rc_socket = None
        self.marks = []
        super().__init__(*args, **kwargs)

    # --------------------------------------------------------------------------------------------------------------
    # @brief Poslat kalibrované kanály přes MAVLink nebo vlastní lokální UDP přijímač.
    #
    # @param[in] None
    # @return None.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def rc(self):
        if not self.send_rc:
            return
        channels = [1500] * 8
        channels[4] = self.rc5
        for channel, value in zip(self.rcmap, (self.roll, self.pitch, self.yaw, self.throttle)):
            channels[channel - 1] = value
        if self.native_rc:
            self.rc_socket.sendto(struct.pack('<8H', *channels), ('127.0.0.1', 5501))
        else:
            self.link.mav.rc_channels_override_send(1, 1, *channels)

    # --------------------------------------------------------------------------------------------------------------
    # @brief Ověřit vlastnictví UDP portu a oddělit RC přijímač od GCS provozu.
    #
    # @param[in] None
    # @return None.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def enable_native_receiver(self):
        # RCIN port comes from SITL_cmdline.cpp; reject shared/unowned bindings before any UDP input.
        descriptors = {os.readlink(fd) for fd in Path(f'/proc/{self.process.pid}/fd').iterdir()}
        bindings = [row.split()[9] for row in Path('/proc/net/udp').read_text().splitlines()[1:]
                    if row.split()[1].split(':')[1] == f'{5501:04X}']
        assert bindings and all(f'socket:[{inode}]' in descriptors for inode in bindings), 'RC UDP port not exclusively owned'
        self.rc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rc_socket.setblocking(False)
        self.native_rc = True
        self.rc()
        # Clear the GCS overrides once; subsequent input comes through AP_RCProtocol_UDP at native 50 Hz.
        self.link.mav.rc_channels_override_send(1, 1, *([0] * 8))

    # --------------------------------------------------------------------------------------------------------------
    # @brief Uzavřít vlastní UDP socket a ukončit pouze vlastněný SITL proces.
    #
    # @param[in] None
    # @return None.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def close(self):
        if self.rc_socket is not None:
            self.rc_socket.close()
            self.rc_socket = None
        super().close()

    # --------------------------------------------------------------------------------------------------------------
    # @brief Přijímat telemetrii, udržovat povolené vstupy a kontrolovat obálku.
    #
    # @param[in] kind Požadovaný typ MAVLink zprávy.
    # @param[in] predicate Podmínka nad skutečnou přijatou zprávou.
    # @param[in] timeout Maximální host čekání v sekundách.
    # @return Data z lokálního běhu.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def wait(self, kind, predicate=lambda msg: True, timeout=30):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.send_heartbeat and time.monotonic() - self.heartbeat_wall >= .1:
                self.link.mav.heartbeat_send(6, 8, 0, 0, 0)
                self.heartbeat_wall = time.monotonic()
            self.rc()
            msg = self.link.recv_match(blocking=True, timeout=.02)
            if msg is None:
                if self.process.poll() is not None:
                    raise RuntimeError('Owned SITL exited')
                continue
            typ = msg.get_type()
            if typ in ('ATTITUDE', 'GLOBAL_POSITION_INT', 'HEARTBEAT', 'RC_CHANNELS'):
                self.telemetry[typ] = msg.to_dict()
            if typ == 'SYSTEM_TIME':
                self.last_boot_ms = msg.time_boot_ms
            if typ == 'STATUSTEXT':
                self.events.append(dict(host=time.time(), boot_ms=self.last_boot_ms, text=msg.text))
                if self.expected_mode in (42, 43) and msg.text.startswith(('UTB ANGLE stop=', 'UTB ACRO stop=')):
                    raise AssertionError(msg.text)
            if self.expected_mode is not None and typ == 'HEARTBEAT':
                assert msg.custom_mode == self.expected_mode, ('Unexpected mode', msg.custom_mode)
            if self.expected_mode in (0, 1, 42, 43) and typ == 'ATTITUDE':
                assert max(abs(msg.roll), abs(msg.pitch)) <= math.radians(35), 'Tilt envelope'
            if self.bench_altitude is not None and typ == 'GLOBAL_POSITION_INT':
                assert abs(msg.relative_alt * .001 - self.bench_altitude) <= 2, 'Harness altitude envelope'
                assert abs(msg.vz) <= 50, 'Harness vertical speed envelope'
            if typ == kind and predicate(msg):
                return msg
        raise AssertionError('Timeout waiting for ' + kind)

    # --------------------------------------------------------------------------------------------------------------
    # @brief Vybrat standardní AP režim a ověřit jeho skutečnou aktivaci.
    #
    # @param[in] number Nativní číslo letového režimu.
    # @param[in] armed Očekávaný skutečný armed stav.
    # @param[in] settle Zachovat původní dodatečné pozorování; false pouze ověří skutečný mode/armed heartbeat.
    # @return None.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def select(self, number, armed=False, settle=True):
        self.expected_mode = None
        if settle:
            self.mode(number, number, armed)
        else:
            # Nonzero-stick transfer must not acquire the legacy observer's extra three-second dwell.
            self.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number)
            self.wait('HEARTBEAT', lambda msg: msg.custom_mode == number and bool(msg.base_mode & 128) == armed)
        self.expected_mode = number

    # --------------------------------------------------------------------------------------------------------------
    # @brief Změnit normalizované pilotní vstupy a zaznamenat čas povelu.
    #
    # @param[in] roll Normalizovaná roll páčka.
    # @param[in] pitch Normalizovaná pitch páčka.
    # @param[in] yaw Normalizovaná yaw páčka.
    # @param[in] seconds Doba držení vstupu v sekundách simulace.
    # @param[in] label Popis změny pro časové značky.
    # @return None.
    #
    # @details Veřejná metoda třídy. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
    # --------------------------------------------------------------------------------------------------------------
    def sticks(self, roll=0, pitch=0, yaw=0, seconds=3, label=''):
        self.roll, self.pitch, self.yaw = (pwm(value) for value in (roll, pitch, yaw))
        self.marks.append(dict(boot_ms=self.last_boot_ms, label=label, roll=self.roll, pitch=self.pitch,
                               yaw=self.yaw, throttle=self.throttle))
        self.duration(seconds)

    # End Public functions
# End Data type, constant, macro definition
# Public functions
# --------------------------------------------------------------------------------------------------------------
# @brief Převést omezený normalizovaný testovací povel na kalibrované RC PWM.
#
# @param[in] normalized Konečná páčka v rozsahu -1 až +1.
# @return Kalibrovaná RC PWM hodnota.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------


def pwm(normalized):
    if not math.isfinite(normalized) or abs(normalized) > 1:
        raise ValueError('Normalized RC outside finite range')
    return 1500 if normalized == 0 else round(1500 + math.copysign(30 + abs(normalized) * 470, normalized))


# --------------------------------------------------------------------------------------------------------------
# @brief Načíst skutečné DataFlash záznamy vlastního experimentu.
#
# @param[in] directory Adresář se skutečnými BIN logy.
# @return Data z lokálního běhu.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def read_packets(directory):
    records = {kind: [] for kind in KINDS}
    for path in sorted((directory / 'logs').glob('*.BIN')):
        reader = mavutil.mavlink_connection(str(path))
        while (msg := reader.recv_match()) is not None:
            if msg.get_type() in records:
                records[msg.get_type()].append(msg.to_dict())
        reader.close()
    return records


# --------------------------------------------------------------------------------------------------------------
# @brief Spustit izolovaný SITL experiment a uchovat data i při neúspěchu.
#
# @param[in] binary Cesta k lokálnímu SITL executable; žádné zařízení ani vzdálený endpoint.
# @param[in] output Nový adresář pro důkazy.
# @param[in] name Jednoznačný název experimentu.
# @param[in] action Testovací callback nad vlastněným simulátorem.
# @param[in] extra Dodatečné SITL parametry.
# @param[in] rcmap Pořadí skutečných RC kanálů roll, pitch, yaw, throttle.
# @return Data z lokálního běhu.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def launch(binary, output, name, action, extra='', rcmap=(1, 2, 4, 3)):
    directory = output / name
    directory.mkdir(parents=True, exist_ok=False)
    (directory / 'run-config.json').write_text(json.dumps(dict(
        binary=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        size=binary.stat().st_size, speedup=1, model='bfxrev', rcmap=rcmap), indent=2) + '\n')
    root = Path(__file__).resolve().parents[3]
    sim, error = None, None
    with (directory / 'console.log').open('x') as console:
        try:
            mapping = ''.join(f'RCMAP_{key} {value}\n' for key, value in
                              zip(('ROLL', 'PITCH', 'YAW', 'THROTTLE'), rcmap))
            sim = AngleSITL(binary, root / 'Tools/autotest/default_params/copter.parm', directory,
                            console, True, 1, extra_params=PARAMS + mapping + extra, model='bfxrev',
                            speedup=1, rcmap=rcmap)
            sim.link.logfile = (directory / 'receive.tlog').open('xb')
            sim.ready()
            action(sim)
            sim.duration(3)
        except Exception as failure:
            error = repr(failure)
        finally:
            if sim:
                (directory / 'events.json').write_text(json.dumps(sim.events, indent=2) + '\n')
                (directory / 'marks.json').write_text(json.dumps(sim.marks, indent=2) + '\n')
                if sim.link.logfile:
                    sim.link.logfile.close()
                sim.close()
    records = read_packets(directory)
    (directory / 'packets.json').write_text(json.dumps(json_safe(records), allow_nan=False) + '\n')
    (directory / 'runtime.json').write_text(json.dumps(dict(error=error, rcmap=rcmap), indent=2) + '\n')
    if error:
        raise AssertionError(f'{name}: {error}; preserved in {directory}')
    return records


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit mapování, GCS/RC volbu režimu a nulovou disarmovanou autoritu.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def disarmed(sim):
    sim.param('UTB_A_ENABLE', 0)
    sim.mode(42, 0)
    sim.param('UTB_A_ENABLE', 1)
    sim.select(42)
    sim.sticks(seconds=1, label='center')
    for axis in ('roll', 'pitch', 'yaw'):
        for sign in (-1, 1):
            sim.sticks(**{axis: sign}, seconds=1, label=f'{axis}{sign}')
    sim.sticks(1, 1, 1, seconds=1, label='diagonal')
    sim.sticks(seconds=1)
    for throttle in (1000, 1250, 1500, 1750, 2000, 1000):
        sim.throttle = throttle
        sim.sticks(seconds=1, label=f'throttle{throttle}')
    sim.select(0)
    sim.expected_mode = None
    sim.rc5 = 1300
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 42)
    sim.expected_mode = 42
    sim.duration(1)
    sim.expected_mode = None
    sim.rc5 = 1000
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 0)
    sim.select(40)
    assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
    assert sim.arm(force=True) != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.select(0)
    sim.select(42)
    sim.duration(1)
    sim.expected_mode = None
    start = len(sim.events)
    sim.roll = 2100
    sim.duration(1)
    assert any('UTB ANGLE stop=2 rc=5' in e['text'] for e in sim.events[start:])
    sim.roll = 1500
    sim.duration(1)
    assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
    sim.select(0)
    sim.select(42)
    sim.duration(1)
    sim.select(0)


# --------------------------------------------------------------------------------------------------------------
# @brief Vyhodnotit skutečné RC reference a nulové disarmované motorové povely.
#
# @param[in] records Skutečné dekódované DataFlash záznamy.
# @return True/analýza/data podle kontraktu; nesoulad vyvolá výjimku.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def analyse_disarmed(records):
    assert records['UARC'] and records['UAMT'] and records['UAAT'], 'Missing ANGLE evidence'
    assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
    assert all(r['APN'] == 0 and r['MN'] == r['Seq'] for r in records['UAMT'])
    assert all(max(r[f'C{i}'] for i in range(1, 5)) <= 1000 for r in records['UAMT'])
    assert all(r['AN'] == 0 for r in records['UAST'])
    assert all(r['Out'] == 0 for r in records['UARC'])
    healthy = [r for r in records['UARC'] if r['V'] and r['Stop'] == 0]
    angles = {(r['TimeUS'], r['Seq']): r for r in records['UAAT']}
    for axis in ('NR', 'NP', 'NY'):
        assert min(r[axis] for r in healthy) < -.99 and max(r[axis] for r in healthy) > .99, axis
    for r in healthy:
        a = angles.get((r['TimeUS'], r['Seq']))
        if a:
            length = max(1, math.hypot(r['NR'], r['NP']))
            assert abs(a['RRef'] - math.radians(5) * r['NR'] / length) < 1e-6
            assert abs(a['PRef'] - math.radians(5) * r['NP'] / length) < 1e-6
            assert abs(a['YR'] - .1 * r['NY']) < 1e-6
    assert any(r['Thr'] > .99 for r in healthy) and any(r['Thr'] == 0 for r in healthy)
    assert any(r['Stop'] == 2 and r['RCF'] == 5 and not r['V'] for r in records['UARC'])
    return dict(disarmed=True, samples=len(records['UARC']), sole_authority=True, rc_signs=True,
                circular_limit=True, manual_throttle=True, range_stop=True)


# --------------------------------------------------------------------------------------------------------------
# @brief Provést původní GUIDED přípravu a vyčkat na ustálený zdravý let.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def prepare_flight(sim):
    sim.bench_altitude = None
    sim.wait('EKF_STATUS_REPORT', lambda m: m.flags & (16 | 512) and not m.flags & 1024, timeout=45)
    sim.mode(4, 4)
    sim.standard_arm()
    sim.throttle = 1500
    assert sim.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10) == 0
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt >= 9700, timeout=45)
    sim.duration(4)
    sim.wait('ATTITUDE', lambda m: max(abs(m.rollspeed), abs(m.pitchspeed), abs(m.yawspeed)) < .03
             and max(abs(m.roll), abs(m.pitch)) < math.radians(3), timeout=15)
    # Measured working point for this bfxrev model/default AP hover map; actual pilot input, not firmware collective.
    sim.throttle = 1472
    position = sim.wait('GLOBAL_POSITION_INT', lambda m: abs(m.vz) < 10)
    sim.bench_altitude = position.relative_alt * .001


# --------------------------------------------------------------------------------------------------------------
# @brief Opustit experiment přes původní LAND a vyčkat na standardní disarm.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def finish_flight(sim):
    sim.bench_altitude = None
    sim.expected_mode = None
    sim.select(9, armed=True)
    sim.wait('GLOBAL_POSITION_INT', lambda m: m.relative_alt < 200, timeout=45)
    sim.duration(3)
    assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128)
    sim.expected_mode = None


# --------------------------------------------------------------------------------------------------------------
# @brief Provést oba směry jedné osy a návraty páčky do středu.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @param[in] axis Testovaná osa roll nebo pitch.
# @param[in] amplitude Požadovaný úhel v °, nejvýše 5.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def flight(sim, axis='roll', amplitude=3):
    prepare_flight(sim)
    sim.select(42, armed=True)
    sim.sticks(seconds=2, label='initial_center')
    for sign in (1, -1):
        sim.sticks(**{axis: sign * amplitude / 5}, label=f'{axis}{sign * amplitude}')
        sim.sticks(label='level')
    finish_flight(sim)


# --------------------------------------------------------------------------------------------------------------
# @brief Vyvolat konfigurace a RC chyby bez armování.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def disarmed_guards(sim):
    for parameter, wrong, correct, stop, fault in (
            ('UTB_A_ANGLE', 4, 5, 1, None), ('FSTRATE_ENABLE', 1, 0, 1, None),
            ('RC1_DZ', 600, 30, 2, 4), ('UTB_SHADOW', 1, 0, 1, None)):
        sim.select(42)
        sim.duration(1)
        sim.expected_mode = None
        start = len(sim.events)
        sim.param(parameter, wrong)
        sim.duration(1)
        text = f'UTB ANGLE stop={stop}' + ('' if fault is None else f' rc={fault}')
        assert any(text in e['text'] for e in sim.events[start:]), (parameter, sim.events[start:])
        sim.param(parameter, correct)
        sim.duration(1)
        assert sim.arm() != mavutil.mavlink.MAV_RESULT_ACCEPTED
        sim.select(0)
    sim.select(42)
    sim.duration(2)
    sim.select(0)


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit současné osy, yaw a malé změny ručního throttle.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @param[in] heading_degrees Přípravný heading v ° před ANGLE vstupem.
# @param[in] throttle_pwm Ruční pracovní bod konkrétního modelu, nikoli výškový regulátor.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def combined_flight(sim, heading_degrees=0, throttle_pwm=1472):
    prepare_flight(sim)
    if heading_degrees:
        assert sim.command(mavutil.mavlink.MAV_CMD_CONDITION_YAW, heading_degrees, 10, 1, 0) == 0
        sim.wait('ATTITUDE', lambda m: abs(math.atan2(math.sin(m.yaw - math.radians(heading_degrees)),
                                                      math.cos(m.yaw - math.radians(heading_degrees)))) < .02
                 and abs(m.yawspeed) < .03, timeout=30)
        sim.duration(1)
    sim.throttle = throttle_pwm
    sim.select(42, armed=True)
    sim.sticks(seconds=2, label='center')
    sim.sticks(.6, -.6, .5, seconds=3, label='combined_yaw_positive')
    sim.sticks(-.6, .6, -.5, seconds=3, label='combined_yaw_negative')
    sim.sticks(seconds=3, label='level_at_changed_heading')
    # Small real manual throttle changes, bounded by the unchanged vz/altitude guards.
    sim.throttle = throttle_pwm + 3
    sim.sticks(.4, 0, 0, seconds=1, label='throttle_up_with_roll')
    sim.throttle = throttle_pwm - 3
    sim.sticks(0, -.4, 0, seconds=1, label='throttle_down_with_pitch')
    sim.throttle = throttle_pwm
    sim.sticks(seconds=2, label='final_level')
    finish_flight(sim)


# --------------------------------------------------------------------------------------------------------------
# @brief Změřit skutečný původní AP STABILIZE při srovnatelných úhlových krocích.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @param[in] axis Testovaná osa roll nebo pitch.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def baseline_flight(sim, axis='roll'):
    prepare_flight(sim)
    # Native STABILIZE keeps its original 30 degree map, shaping and angle boost.
    sim.select(0, armed=True)
    sim.sticks(seconds=2, label='AP_center')
    for sign in (1, -1):
        sim.sticks(**{axis: sign * 3 / 30}, label=f'AP_{axis}{sign * 3}')
        sim.sticks(label='AP_level')
    finish_flight(sim)


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit výslovné přechody AP/UTB a RC mode switch v simulovaném letu.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def transitions_flight(sim):
    prepare_flight(sim)
    # Original AP STABILIZE is first exercised independently at center sticks.
    sim.select(0, armed=True)
    sim.sticks(seconds=2, label='AP_before')
    sim.select(42, armed=True)
    sim.sticks(.6, seconds=3, label='UTB_roll')
    sim.sticks(seconds=3, label='UTB_level')
    sim.select(0, armed=True)
    sim.sticks(seconds=2, label='AP_after')
    sim.select(42, armed=True)
    sim.sticks(seconds=3, label='UTB_reentry')
    sim.expected_mode = None
    sim.rc5 = 1300
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 42)
    sim.duration(1)
    sim.rc5 = 1000
    sim.wait('HEARTBEAT', lambda m: m.custom_mode == 0)
    sim.duration(1)
    finish_flight(sim)


# --------------------------------------------------------------------------------------------------------------
# @brief Odděleně vyvolat skutečnou ztrátu RC nebo GCS a obnovit spojení.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @param[in] source Ztracený zdroj rc nebo gcs.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def failsafe_flight(sim, source):
    prepare_flight(sim)
    sim.select(42, armed=True)
    sim.sticks(.4, seconds=2, label='before_loss')
    sim.sticks(seconds=2)
    sim.expected_mode = None
    # Native AP RTL/LAND may leave the ANGLE vertical envelope after the explicit failsafe mode transition.
    sim.bench_altitude = None
    start = len(sim.events)
    if source == 'rc':
        sim.param('SIM_RC_FAIL', 1)
        sim.send_rc = False
    else:
        sim.enable_native_receiver()
        sim.send_heartbeat = False
    sim.duration(8)
    events = sim.events[start:]
    assert any(('Radio Failsafe' in e['text'] or 'UTB ANGLE stop=2' in e['text'])
               if source == 'rc' else 'GCS Failsafe' in e['text'] for e in events), events
    sim.send_rc = sim.send_heartbeat = True
    sim.native_rc = False
    if source == 'rc':
        sim.param('SIM_RC_FAIL', 0)
    sim.duration(2)
    finish_flight(sim)


# --------------------------------------------------------------------------------------------------------------
# @brief Odmítnout neúplnou evidenci a vyhodnotit skutečnou UTB kaskádu a output.
#
# @param[in] records Skutečné dekódované DataFlash záznamy.
# @param[in] axis Testovaná osa roll nebo pitch.
# @param[in] amplitude Požadovaný úhel v °, nejvýše 5.
# @param[in] pilot_mode 42 pro vlastní attitude, 43 pro přímou RC rate referenci.
# @param[in] minimum_samples Počet požadovaných úplných vzorků; přechody mají kratší okna.
# @return True/analýza/data podle kontraktu; nesoulad vyvolá výjimku.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def analyse_flight(records, axis=None, amplitude=None, pilot_mode=42, minimum_samples=800):
    sets = assemble(records)
    assert sets and records['UARC'], 'No complete controller/output evidence'
    pilot = {(r['TimeUS'], r['Seq']): r for r in records['UARC']}
    angles = {(r['TimeUS'], r['Seq']): r for r in records['UAAT']}
    status = {(r['TimeUS'], r['Seq']): r for r in records['UAST']}
    feedback = {}
    for r in records['UAFB']:
        feedback.setdefault((r['TimeUS'], r['Seq']), []).append(r)
    thrust = {(r['TimeUS'], r['Seq']): r for r in records['UALT']}
    assert all(r['Stop'] == 0 and r['V'] for r in records['UARC']), 'Latched or invalid pilot'
    assert all(r['Flt'] == 0 for r in records['UAMT']), 'PID fault'
    rows = []
    first = sets[0][0]['TimeUS']
    for m, axes in sets:
        key = (m['TimeUS'], m['Seq'])
        p, a, s = pilot[key], angles[key], status[key]
        assert len(feedback[key]) == 3 and key in thrust, 'Incomplete allocation capture'
        assert {row['Ax'] for row in feedback[key]} == {0, 1, 2}, 'Duplicated or missing feedback axis'
        if m['Seq'] > 1:
            assert all(row['Av'] and row['PV'] and row['CSeq'] == m['Seq'] for row in feedback[key]), 'Invalid capture'
        assert m['APN'] == 0 and s['AN'] == 0 and m['MN'] == m['Seq'], 'Double ownership/output'
        assert a['Flt'] == 0 and a['Src'] == (0 if pilot_mode == 43 else 1)
        assert m['UN'] == m['Seq'] and s['UN'] == (0 if pilot_mode == 43 else m['Seq']), 'Missing UTB cycle'
        assert math.hypot(a['RRef'], a['PRef']) <= math.radians(5) + 1e-6
        assert math.hypot(a['RR'], a['PR']) <= .3 + 1e-6
        row = dict(t=(m['TimeUS'] - first) * 1e-6, dt=axes[0]['Dt'], seq=m['Seq'],
                   angle0=a['Roll'], angle1=a['Pitch'], angle_ref0=a['RRef'], angle_ref1=a['PRef'],
                   yaw=a['Yaw'], yaw_target=a['YR'], altitude=s['Alt'], vz=s['VZ'], throttle=s['Thr'],
                   manual=p['Thr'], execution_us=m['Ex'], log_us=p['Log'], run_us=p['Run'])
        row.update(time_us=m['TimeUS'], rc_time_ms=p['RCms'], rc_age_ms=p['Age'],
                   normalized_roll=p['NR'], normalized_pitch=p['NP'], normalized_yaw=p['NY'],
                   mode=p['Md'], spool=axes[0]['Spl'])
        for motor in range(4):
            row[f'pwm{motor}'] = m[f'C{motor + 1}']
        row['attitude_calls'] = s['UN']
        row['collective'] = sum(thrust[key][f'T{motor + 1}'] for motor in range(4)) / (4 * thrust[key]['G'])
        row['allocation_limits'] = thrust[key]['Lim']
        row['integration_blocked'] = feedback[key][0]['Blk']
        for i in range(3):
            assert axes[i]['Arm'] and axes[i]['Spl'] == 3 and axes[i]['Own'] == 1
            for label in ('Tar', 'Gyr', 'P', 'I', 'D', 'Out'):
                row[f'{label}{i}'] = axes[i][label]
            if m['Seq'] > 1:
                gain = .3 if i == 2 else .19
                assert abs(row[f'P{i}'] - gain * (row[f'Tar{i}'] - row[f'Gyr{i}'])) < 1e-6, 'Wrong pilot PID preset'
        assert all(math.isfinite(v) for v in row.values()), 'Nonfinite evidence'
        rows.append(row)
    assert len(rows) >= minimum_samples, 'Insufficient complete active evidence'
    assert all(b['seq'] == a['seq'] + 1 for a, b in zip(rows, rows[1:])), 'Missing complete set'
    result = dict(complete_sets=len(rows), sole_authority=True,
                  angle=[angle_metrics(rows, i) for i in range(2)],
                  maximum_vertical_speed=max(abs(r['vz']) for r in rows),
                  altitude_span=max(r['altitude'] for r in rows) - min(r['altitude'] for r in rows),
                  producer_wall_us=dict(mean=sum(r['execution_us'] for r in rows) / len(rows),
                                        maximum=max(r['execution_us'] for r in rows)),
                  logger_wall_us=dict(mean=sum(r['log_us'] for r in rows) / len(rows),
                                      maximum=max(r['log_us'] for r in rows)))
    result['logger_drops_max'] = max((r['Dp'] for r in records['DSF']), default=None)
    result['complete_set_rate_hz'] = (len(rows) - 1) / (rows[-1]['t'] - rows[0]['t'])
    result['allocation_limit_masks'] = sorted({r['allocation_limits'] for r in rows})
    result['integration_blocked_masks'] = sorted({r['integration_blocked'] for r in rows})
    if axis is not None:
        index = ('roll', 'pitch').index(axis)
        steps = result['angle'][index]['steps']
        assert len(steps) == 4, ('Expected signed steps and center returns', steps)
        assert max(r[f'angle_ref{index}'] for r in rows) >= math.radians(amplitude) - 1e-5
        assert min(r[f'angle_ref{index}'] for r in rows) <= -math.radians(amplitude) + 1e-5
        for step in steps:
            assert step['rise_10_90_s'] is not None and step['settling_2pct_s'] is not None, step
            assert abs(step['steady_error_rad']) < math.radians(.15), step
        centers = [i for i in range(1, len(rows)) if rows[i][f'angle_ref{index}'] == 0
                   and abs(rows[i - 1][f'angle_ref{index}']) > .01]
        assert len(centers) == 2
        for i in centers:
            assert rows[i][f'Tar{index}'] * rows[i][f'angle{index}'] < 0, 'No active leveling'
            assert any(r[f'Out{index}'] * r[f'angle{index}'] < 0 for r in rows[i:i + 80]), 'No PID leveling output'
    return result, rows


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit standardní rudder arming, spool a rudder disarming v SITL na zemi.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def ground_spool(sim):
    sim.select(42)
    sim.yaw = 2000
    sim.wait('HEARTBEAT', lambda m: bool(m.base_mode & 128), timeout=10)
    sim.yaw = 1500
    sim.duration(1)
    sim.throttle = 1100
    sim.duration(2)
    sim.throttle = 1000
    sim.duration(2)
    sim.yaw = 1000
    sim.wait('HEARTBEAT', lambda m: not m.base_mode & 128, timeout=10)
    sim.yaw = 1500
    sim.duration(1)
    sim.select(0)


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit fail-stop při změně primárního EKF/gyro zdroje za DISARMED.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def gyro_switch(sim):
    sim.select(42)
    sim.duration(2)
    sim.expected_mode = None
    start = len(sim.events)
    sim.param('EK3_PRIMARY', 1)
    sim.wait('STATUSTEXT', lambda m: m.text.startswith('UTB ANGLE stop='), timeout=15)
    assert any('UTB ANGLE stop=' in e['text'] for e in sim.events[start:])
    sim.duration(1)
    sim.param('EK3_PRIMARY', 0)
    sim.duration(1)
    sim.select(0)


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit nulovou starou PID historii v jednotlivých skutečných mode oknech.
#
# @param[in] records Skutečné dekódované DataFlash záznamy.
# @return True/analýza/data podle kontraktu; nesoulad vyvolá výjimku.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def analyse_transitions(records):
    entries = [r['TimeUS'] for r in records['MODE'] if r['ModeNum'] == 42]
    assert len(entries) >= 2, 'No repeated actual mode entry'
    result = []
    for start in entries:
        end = next((r['TimeUS'] for r in records['MODE'] if r['TimeUS'] > start and r['ModeNum'] != 42), math.inf)
        part = {kind: [r for r in records[kind] if start < r.get('TimeUS', 0) < end] for kind in KINDS}
        analysis, rows = analyse_flight(part)
        assert rows[0]['seq'] == 1, 'Sequence not reset on reentry'
        assert all(rows[0][f'{field}{axis}'] == 0 for field in ('I', 'D', 'Out') for axis in range(3)), 'Old PID state'
        result.append(analysis)
    assert any(r['ModeNum'] == 0 and r['TimeUS'] > entries[0] for r in records['MODE'])
    return dict(reentry_reset=True, windows=result)


# --------------------------------------------------------------------------------------------------------------
# @brief Ověřit explicitní AP přechod nebo latched stop bez návratu staré reference.
#
# @param[in] records Skutečné dekódované DataFlash záznamy.
# @param[in] source Ztracený zdroj rc nebo gcs.
# @return True/analýza/data podle kontraktu; nesoulad vyvolá výjimku.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def analyse_failsafe(records, source):
    modes = records['MODE']
    entries = [r['TimeUS'] for r in modes if r['ModeNum'] == 42]
    assert len(entries) == 1, 'Unexpected automatic ANGLE reentry'
    exit_time = next((r['TimeUS'] for r in modes if r['TimeUS'] > entries[0] and r['ModeNum'] != 42), None)
    stopped = [r for r in records['UARC'] if r['Stop']]
    assert stopped or exit_time is not None, 'No stop or native failsafe exit'
    assert all(r['Out'] == 0 and not r['V'] for r in stopped), 'Old pilot throttle retained after stop'
    keys = {(r['TimeUS'], r['Seq']) for r in stopped}
    assert all(r['Out'] == 0 for r in records['UARX'] if (r['TimeUS'], r['Seq']) in keys)
    if exit_time is not None:
        assert all(r['TimeUS'] <= exit_time for r in records['UARC']), 'ANGLE continued after explicit exit'
    return dict(source=source, first_stop=stopped[0] if stopped else None, explicit_exit_us=exit_time,
                no_automatic_reentry=True, no_old_output=True)


# --------------------------------------------------------------------------------------------------------------
# @brief Vyhodnotit původní STABILIZE s uvedenou časovou nejistotou RCIN.
#
# @param[in] records Skutečné dekódované DataFlash záznamy.
# @param[in] axis Testovaná osa roll nebo pitch.
# @return True/analýza/data podle kontraktu; nesoulad vyvolá výjimku.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def analyse_baseline(records, axis):
    assert not records['UARC'] and not records['UAMT'], 'UTB active in AP baseline'
    start = next(r['TimeUS'] for r in records['MODE'] if r['ModeNum'] == 0 and r['TimeUS'] > 10000000)
    end = next(r['TimeUS'] for r in records['MODE'] if r['ModeNum'] == 9 and r['TimeUS'] > start)
    rc = records['RCIN']
    times = [r['TimeUS'] for r in rc]
    rows = []
    for a in records['ATT']:
        if not start <= a['TimeUS'] < end:
            continue
        channels = rc[bisect.bisect_right(times, a['TimeUS']) - 1]
        row = dict(t=(a['TimeUS'] - start) * 1e-6, dt=.1)
        for i, label in enumerate(('Roll', 'Pitch')):
            value = channels[f'C{i + 1}'] - 1500
            normalized = 0 if abs(value) <= 30 else math.copysign((abs(value) - 30) / 470, value)
            row[f'angle_ref{i}'] = math.radians(normalized * 30)
            row[f'angle{i}'] = math.radians(a[label])
        row['dt'] = row['t'] - rows[-1]['t'] if rows else .1
        rows.append(row)
    assert len(rows) >= 100 and rows[-1]['t'] - rows[0]['t'] >= 10, 'Insufficient native ATT coverage'
    metrics = angle_metrics(rows, ('roll', 'pitch').index(axis))
    assert len(metrics['steps']) == 4 and all(s['rise_10_90_s'] is not None for s in metrics['steps'])
    return dict(axis=axis, samples=len(rows), metrics=metrics, raw_reference_timestamp_uncertainty_s=.1,
                original_ap_shaping=True, original_ap_angle_boost=True), rows


# --------------------------------------------------------------------------------------------------------------
# @brief Zprostředkovat ruční příkazy skutečnému RC vstupu vlastního simulátoru.
#
# @param[in] sim Vlastněný lokální simulátor s AP RC cestou.
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def manual(sim):
    """Actual interactive pilot input; keep RC and heartbeat flowing between commands."""
    print('Owned SITL ready. Commands: sticks R P Y (-1..1), throttle PWM (1000..2000),', flush=True)
    print('mode N, rate-limit PARAM VALUE, arm, disarm, takeoff, land, status, external, quit.', flush=True)
    print('No hardware endpoint is accepted.', flush=True)
    print('Secondary GCS telemetry: tcp:127.0.0.1:5762. Use external before GCS RC control.', flush=True)
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            command = sys.stdin.readline().strip().split()
            if not command or command[0] == 'quit':
                heartbeat = sim.wait('HEARTBEAT')
                if heartbeat.base_mode & 128:
                    finish_flight(sim)
                return
            if command[0] == 'sticks' and len(command) == 4:
                sim.roll, sim.pitch, sim.yaw = (pwm(float(value)) for value in command[1:])
            elif command[0] == 'throttle' and len(command) == 2:
                value = int(command[1])
                if not 1000 <= value <= 2000:
                    print('Throttle range is 1000..2000', flush=True)
                    continue
                sim.throttle = value
            elif command[0] == 'mode' and len(command) == 2:
                armed = bool(sim.wait('HEARTBEAT').base_mode & 128)
                sim.select(int(command[1]), armed=armed, settle=False)
            elif command[0] == 'rate-limit' and len(command) == 3:
                allowed = ('UTB_C_RMAX', 'UTB_C_PMAX', 'UTB_C_YMAX', 'UTB_A_RMAX', 'UTB_A_YMAX')
                if command[1] not in allowed or sim.wait('HEARTBEAT').custom_mode in (42, 43):
                    print('Change supported rate-limit PARAM VALUE outside mode42/43.', flush=True)
                    continue
                sim.param(command[1], float(command[2]))
            elif command[0] == 'arm':
                sim.standard_arm()
            elif command[0] == 'disarm':
                assert sim.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0) == 0
            elif command[0] == 'takeoff':
                prepare_flight(sim)
                sim.select(42, armed=True)
            elif command[0] == 'land':
                finish_flight(sim)
            elif command[0] == 'status':
                print(json.dumps(sim.telemetry), flush=True)
            elif command[0] == 'external':
                sim.link.mav.rc_channels_override_send(1, 1, *([0] * 8))
                sim.expected_mode = None
                sim.send_rc = sim.send_heartbeat = False
                print('Overrides released; external GCS must provide RC and heartbeat.', flush=True)
            else:
                print('Unknown command', flush=True)
        sim.wait('SYSTEM_TIME')


# --------------------------------------------------------------------------------------------------------------
# @brief Vybrat reprodukovatelný scénář a uložit jeho skutečný výsledek.
#
# @param[in] None
# @return None.
#
# @details Globální funkce modulu. Pouze vlastní lokální SITL; žádná fyzická FC komunikace.
# --------------------------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--suite', choices=('disarmed', 'roll3', 'pitch3', 'roll5', 'pitch5', 'guards', 'combined',
                                            'transitions', 'rc-loss', 'gcs-loss', 'remap-roll3',
                                            'ap-roll3', 'ap-pitch3', 'manual'), required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.suite == 'manual':
        launch(args.binary.resolve(), args.output.resolve(), 'manual', manual)
        return
    if args.suite == 'disarmed':
        records = launch(args.binary.resolve(), args.output.resolve(), args.suite, disarmed)
        result = analyse_disarmed(records)
    else:
        gate = args.output / 'disarmed' / 'analysis.json'
        assert gate.exists() and json.loads(gate.read_text())['disarmed'], 'DISARMED evidence required first'
        if args.suite == 'guards':
            records = launch(args.binary.resolve(), args.output.resolve(), args.suite, disarmed_guards)
            assert all(r['Arm'] == 0 and r['Out'] == 0 for r in records['UARX'])
            result = dict(disarmed_guards=True, stops=sorted({r['Stop'] for r in records['UARC']}))
        elif args.suite in ('rc-loss', 'gcs-loss'):
            source = args.suite.split('-')[0]
            records = launch(args.binary.resolve(), args.output.resolve(), args.suite,
                             partial(failsafe_flight, source=source))
            result = analyse_failsafe(records, source)
        elif args.suite == 'transitions':
            records = launch(args.binary.resolve(), args.output.resolve(), args.suite, transitions_flight)
            result = analyse_transitions(records)
        elif args.suite.startswith('ap-'):
            axis = args.suite[3:-1]
            records = launch(args.binary.resolve(), args.output.resolve(), args.suite,
                             lambda sim: baseline_flight(sim, axis), extra='UTB_ENABLE 0\n')
            result, rows = analyse_baseline(records, axis)
            (args.output / args.suite / 'rows.json').write_text(json.dumps(rows) + '\n')
        else:
            extra, rcmap = '', (1, 2, 4, 3)
            if args.suite == 'combined':
                action, axis, amplitude = combined_flight, None, None
            elif args.suite == 'remap-roll3':
                action, axis, amplitude = partial(flight, axis='roll', amplitude=3), 'roll', 3
                extra, rcmap = 'RC6_REVERSED 1\n', (6, 7, 8, 3)
            else:
                axis, amplitude = args.suite[:-1], int(args.suite[-1])
                action = partial(flight, axis=axis, amplitude=amplitude)
            records = launch(args.binary.resolve(), args.output.resolve(), args.suite, action, extra=extra, rcmap=rcmap)
            result, rows = analyse_flight(records, axis, amplitude)
            (args.output / args.suite / 'rows.json').write_text(json.dumps(rows) + '\n')
    (args.output / args.suite / 'analysis.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


# End Public functions
# Private function prototypes
# None
# End Private function prototypes
# Private functions
# None
# End Private functions


if __name__ == '__main__':
    main()
