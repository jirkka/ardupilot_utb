#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Unit checks of pilot conversion and strict rejection of incomplete SITL evidence."""
# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses remain unchanged.
# @file test_utb_angle_sitl.py
# @brief Nezávislé unit kontroly runneru, nikoli náhrada skutečných letových dat.
# @author Jiri Kovar
# @version 2026-10-17
# @date 2026-10-17
# --------------------------------------------------------------------------------------------------------------------
# Include Files
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from utb_angle_sitl import analyse_disarmed, analyse_flight, pwm  # noqa: E402
from test_utb_shadow import source_without_comments  # noqa: E402

# End Include Files
# Data type, constant, macro definition


class AngleEvidence(unittest.TestCase):
    # Public functions
    # --------------------------------------------------------------------------------------------------------------------
    # @brief Ověřit endpointy, znaménka a skutečnou testovací mrtvou zónu.
    # @param[in] None
    # @return Výsledek operace; nesoulad kontraktu vyvolá výjimku.
    # @details Veřejná metoda třídy. Skutečné SITL výsledky se nezaměňují za unit data.
    # --------------------------------------------------------------------------------------------------------------------
    def test_pwm_endpoints_and_real_deadband(self):
        self.assertEqual(pwm(0), 1500)
        self.assertEqual(pwm(1), 2000)
        self.assertEqual(pwm(-1), 1000)
        self.assertEqual(pwm(.6), 1812)
        self.assertEqual(pwm(-.6), 1188)
        self.assertGreater(pwm(.01), 1530)
        self.assertLess(pwm(-.01), 1470)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief Odmítnout NaN, Inf a povely mimo povolený rozsah.
    # @param[in] None
    # @return Výsledek operace; nesoulad kontraktu vyvolá výjimku.
    # @details Veřejná metoda třídy. Skutečné SITL výsledky se nezaměňují za unit data.
    # --------------------------------------------------------------------------------------------------------------------
    def test_nonfinite_and_unsupported_commands_rejected(self):
        for value in (math.nan, math.inf, -math.inf, 1.01, -1.01):
            with self.assertRaises(ValueError):
                pwm(value)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief Odmítnout DISARMED výsledek bez skutečných záznamů.
    # @param[in] None
    # @return Výsledek operace; nesoulad kontraktu vyvolá výjimku.
    # @details Veřejná metoda třídy. Skutečné SITL výsledky se nezaměňují za unit data.
    # --------------------------------------------------------------------------------------------------------------------
    def test_no_evidence_cannot_pass_disarmed(self):
        with self.assertRaises(AssertionError):
            analyse_disarmed(dict(UARC=[], UAMT=[], UAAT=[]))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief Odmítnout letový výsledek bez controller a motorové evidence.
    # @param[in] None
    # @return Výsledek operace; nesoulad kontraktu vyvolá výjimku.
    # @details Veřejná metoda třídy. Skutečné SITL výsledky se nezaměňují za unit data.
    # --------------------------------------------------------------------------------------------------------------------
    def test_no_evidence_cannot_pass_flight(self):
        with self.assertRaises(AssertionError):
            analyse_flight(dict(UARX=[], UAMT=[], UARC=[]))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief Odmítnout záznam s chybějícím odpovídajícím pilotním snapshotem.
    # @param[in] None
    # @return Výsledek operace; nesoulad kontraktu vyvolá výjimku.
    # @details Veřejná metoda třídy. Skutečné SITL výsledky se nezaměňují za unit data.
    # --------------------------------------------------------------------------------------------------------------------
    def test_missing_pilot_packet_cannot_pass(self):
        records = dict(UARC=[dict(TimeUS=1, Seq=1, Stop=0, V=1)], UAAT=[], UAST=[],
                       UAMT=[dict(TimeUS=2, Seq=2, Flt=0)],
                       UARX=[dict(TimeUS=2, Seq=2, Ax=i) for i in range(3)])
        with self.assertRaises((AssertionError, KeyError)):
            analyse_flight(records)

    # --------------------------------------------------------------------------------------------------------------------
    # @brief Nezapočítat Doxygen popis funkce jako další volání nebo definici.
    # @param[in] None
    # @return None.
    # @details Unit regrese textové kontroly; skutečná volání se zachovávají.
    # --------------------------------------------------------------------------------------------------------------------
    def test_source_audit_ignores_only_comment_occurrences(self):
        source = 'bool utb_log_update(); // utb_log_update()\n/* utb_log_update() */ utb_log_update();\n'
        code = source_without_comments(source)
        self.assertEqual(code.count('utb_log_update()'), 2)
        self.assertEqual(len(code), len(source))
        self.assertEqual(code.count('\n'), source.count('\n'))

    # --------------------------------------------------------------------------------------------------------------------
    # @brief Zachovat skutečnou zakázanou závislost a comment markery v literálech.
    # @param[in] None
    # @return None.
    # @details Oprava parseru nesmí vymazat skutečný motorový writer ze statického auditu.
    # --------------------------------------------------------------------------------------------------------------------
    def test_source_audit_retains_real_code_and_quoted_markers(self):
        source = '// AP_Motors\nconst char *url = "https://x/*keep*/"; AP_Motors *motors;\n'
        code = source_without_comments(source)
        self.assertEqual(code.count('AP_Motors'), 1)
        self.assertIn('"https://x/*keep*/"', code)
        self.assertIn('AP_Motors *motors;', code)

    # End Public functions
# End Data type, constant, macro definition
# Static data declarations
# None
# End Static data declarations
# Private function prototypes
# None
# End Private function prototypes
# Private functions
# None
# End Private functions


if __name__ == '__main__':
    unittest.main()
