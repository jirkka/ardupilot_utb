#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Evidence gate and native mode-switch runner contract tests; no vehicle connection."""
# NOTICE: UTB project extension; existing licenses remain unchanged.
# @file test_utb_pilot_sitl.py
# @brief Ověřit odmítnutí neúplných/chybně přiřazených důkazů a přímý RC mode switch.
# @author Jiri Kovar
# @version 2026-10-17
# @date 2026-10-17
# Include Files
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import utb_pilot_sitl as pilot  # noqa: E402
# End Include Files
# Data type, constant, macro definition


class PilotEvidence(unittest.TestCase):
    # Public functions
    # @brief Odmítnout aktivní experiment bez předchozího skutečného gate.
    # @param[in] None
    # @return None; assertions kontrolují očekávanou výjimku.
    # @details Unit test runneru, žádná náhrada skutečných SITL dat.
    def test_missing_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(AssertionError):
                pilot.require_gate(Path(directory) / 'binary', Path(directory))

    # @brief Odmítnout gate při změně executable po DISARMED kontrole.
    # @param[in] None
    # @return None; assertions kontrolují skutečný SHA kontrakt.
    # @details Lokální dočasné unit soubory, bez vehicle procesu.
    def test_gate_bound_to_exact_executable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / 'binary'
            binary.write_bytes(b'validated executable')
            gate = root / 'disarmed'
            gate.mkdir()
            (gate / 'analysis.json').write_text(json.dumps(dict(
                disarmed=True, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())))
            pilot.require_gate(binary, root)
            binary.write_bytes(b'changed executable')
            with self.assertRaisesRegex(AssertionError, 'binary mismatch'):
                pilot.require_gate(binary, root)

    # @brief Neakceptovat prázdný BIN export jako DISARMED PASS.
    # @param[in] None
    # @return None; očekává odmítnutí.
    # @details Nepředkládá syntetická data jako letové výsledky.
    def test_empty_disarmed_rejected(self):
        with self.assertRaises(AssertionError):
            pilot.analyse_disarmed(dict(UARC=[], UAMT=[]))

    # @brief Neakceptovat absenci skutečných mode oken.
    # @param[in] None
    # @return None; očekává odmítnutí.
    # @details Hlídá, aby další analýza nemohla formálně projít bez letu.
    def test_empty_flight_rejected(self):
        with self.assertRaises(AssertionError):
            pilot.windows(dict(MODE=[]))
        with self.assertRaises(AssertionError):
            pilot.validate_acro([])

    # @brief Při RC přepnutí neměnit žádnou páčku ani throttle.
    # @param[in] None
    # @return None; assertions ověřují kontrakt pilotního runneru.
    # @details Mock transport není tvrzení o skutečné mode registraci; tu ověřuje SITL.
    def test_direct_nonzero_rc_switch_preserves_sticks(self):
        sim = SimpleNamespace(last_boot_ms=1000, marks=[], roll=1671, pitch=1329, yaw=1765,
                              throttle=1475, rc5=1300, expected_mode=42, wait=Mock())
        pilot.switch(sim, 43, rc=True)
        self.assertEqual((sim.roll, sim.pitch, sim.yaw, sim.throttle), (1671, 1329, 1765, 1475))
        self.assertEqual(sim.rc5, 1450)
        self.assertEqual(sim.expected_mode, 43)
        self.assertTrue(sim.wait.call_args.args[1](SimpleNamespace(custom_mode=43, base_mode=128)))
        self.assertFalse(sim.wait.call_args.args[1](SimpleNamespace(custom_mode=42, base_mode=128)))

    # @brief Při GCS přepnutí použít standardní select bez zásahu do páček.
    # @param[in] None
    # @return None; assertions ověřují standardní mode request.
    # @details Pouze unit test transportní politiky.
    def test_direct_gcs_switch(self):
        sim = SimpleNamespace(last_boot_ms=1000, marks=[], roll=1671, pitch=1500, yaw=1500,
                              throttle=1472, select=Mock())
        pilot.switch(sim, 42)
        sim.select.assert_called_once_with(42, armed=True, settle=False)
        self.assertEqual(sim.roll, 1671)

    # @brief Zachovat skutečný mode/armed ACK bez nuceného dlouhého držení nenulové páčky.
    # @param[in] None
    # @return None; assertions kontrolují obě varianty existujícího selektoru.
    # @details Rychlá větev nesmí vynechat heartbeat ověření; původní default zachovává observer.
    def test_fast_ack_and_legacy_observer(self):
        sim = pilot.angle.AngleSITL.__new__(pilot.angle.AngleSITL)
        sim.mode = Mock()
        sim.wait = Mock()
        sim.link = SimpleNamespace(mav=Mock())
        sim.select(43, armed=True, settle=False)
        sim.mode.assert_not_called()
        sim.link.mav.set_mode_send.assert_called_once_with(
            1, pilot.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 43)
        predicate = sim.wait.call_args.args[1]
        self.assertTrue(predicate(SimpleNamespace(custom_mode=43, base_mode=128)))
        self.assertFalse(predicate(SimpleNamespace(custom_mode=43, base_mode=0)))
        self.assertFalse(predicate(SimpleNamespace(custom_mode=42, base_mode=128)))
        self.assertEqual(sim.expected_mode, 43)
        sim.select(42, armed=True)
        sim.mode.assert_called_once_with(42, 42, True)
    # End Public functions
# End Data type, constant, macro definition
# ==============================================================================================================
# Static data declarations
# --------------------------------------------------------------------------------------------------------------
# None
# End Static data declarations
# ==============================================================================================================
# Private function prototypes
# --------------------------------------------------------------------------------------------------------------
# None
# End Private function prototypes
# ==============================================================================================================
# Private functions
# --------------------------------------------------------------------------------------------------------------
# None
# End Private functions


if __name__ == '__main__':
    unittest.main()
