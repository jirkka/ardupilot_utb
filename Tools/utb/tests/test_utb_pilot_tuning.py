#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''Offline validation of tuning analysis contracts; never opens a vehicle connection.'''
# --------------------------------------------------------------------------------------------------------------------
# NOTICE: UTB project extension; existing licenses remain unchanged.
# @file test_utb_pilot_tuning.py
# @brief Kontrola parametrické izolace a pravdivého vyhodnocení ustálení.
# @author Jiri Kovar
# @version 2026-10-18
# @date 2026-10-18
# --------------------------------------------------------------------------------------------------------------------
# Include Files
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import utb_pilot_tuning as tuning  # noqa: E402
# End Include Files
# Data type, constant, macro definition


class PilotTuningAnalysis(unittest.TestCase):
    # Public functions
    # @brief Ověřit samostatné AP názvy bez nechtěného přepsání původního RAT/ATC kontextu.
    # @param[in] None
    # @return None.
    # @details Unit parametrické serializace; není náhradou skutečného AP readbacku.
    def test_isolated_parameter_names(self):
        names = [line.split()[0] for line in tuning.parameter_text(tuning.BASELINE).splitlines()]
        self.assertEqual(len(names), 17)
        self.assertEqual(len(set(names)), 17)
        self.assertTrue(all(n.startswith('UTB_P_') and len(n) <= 16 for n in names))
        self.assertNotIn('UTB_RAT_RLL_P', names)
        self.assertNotIn('ATC_RAT_RLL_P', names)

    # @brief Správně vyhodnotit monotónní první řád ve stejném pevném okně.
    # @param[in] None
    # @return None.
    # @details Rise má analyticky tau*ln(9); toleruje jen diskretizaci 2,5ms.
    def test_known_monotone_response(self):
        result = tuning.plateau(self._rows(), 0, 'rate')
        self.assertAlmostEqual(result['rise_10_90_s'], .03 * math.log(9), delta=.003)
        self.assertEqual(result['overshoot_percent'], 0)
        self.assertIsNotNone(result['settle_2percent_s'])
        self.assertGreater(result['iae'], .004)
        self.assertLess(result['iae'], .005)

    # @brief Nevydávat neustálenou oscilující odezvu za dokončený settling.
    # @param[in] None
    # @return None.
    # @details Ochrana před výběrem kandidáta podle falešného PASS.
    def test_no_false_settling(self):
        result = tuning.plateau(self._rows(True), 0, 'rate')
        self.assertIsNone(result['settle_2percent_s'])
        self.assertGreater(result['overshoot_percent'], 10)

    # @brief Odmítnout příliš krátká data místo inventování časové odezvy.
    # @param[in] None
    # @return None.
    # @details Prázdný nebo krátký plateau není naměřená odezva.
    def test_missing_or_short_plateau(self):
        self.assertIsNone(tuning.plateau([], 0, 'rate'))
        self.assertIsNone(tuning.plateau(self._rows()[:20], 0, 'rate'))

    # @brief Zachovat identické referenční amplitudy při rozdílných controller gains.
    # @param[in] None
    # @return None.
    # @details Parametrický text nesmí měnit C_RMAX/A_ANGLE nebo aplikovat shaping.
    def test_tune_does_not_change_pilot_reference(self):
        text = tuning.parameter_text(tuning.BASELINE)
        for forbidden in ('C_RMAX', 'C_PMAX', 'C_YMAX', 'A_ANGLE', 'A_RMAX', 'EXPO', 'RATE_TC'):
            self.assertNotIn(forbidden, text)

    # @brief Ověřit skutečnou konfiguraci podle časově správných BIN parametrů.
    # @param[in] None
    # @return None.
    # @details Nesmí se vrátit k hardcoded P při parametrizovaném controlleru.
    def test_logged_gain_snapshot(self):
        records = {'PARM': [dict(Name=name, Value=value, TimeUS=10) for name, value in
                            zip(('UTB_P_RLL_P', 'UTB_P_PIT_P', 'UTB_P_YAW_P'), (.095, .1425, .3))]}
        gains, source = tuning.angle.pilot_proportional_gains(records, 100, 200)
        self.assertEqual(gains, (.095, .1425, .3))
        self.assertEqual(source, 'BIN-PARM-frozen')
        records['PARM'].append(dict(Name='UTB_P_RLL_P', Value=.19, TimeUS=300))
        self.assertEqual(tuning.angle.pilot_proportional_gains(records, 100, 200)[0], gains)
        with self.assertRaisesRegex(AssertionError, 'changed'):
            tuning.angle.pilot_proportional_gains(records, 100, 400)

    # @brief Odmítnout chybějící nové metadata, historický log rozlišit explicitně.
    # @param[in] None
    # @return None.
    # @details Legacy fallback se nepoužije pro částečně parametrizovaný log.
    def test_missing_new_gain_metadata(self):
        records = {'PARM': [dict(Name='UTB_P_RLL_P', Value=.095, TimeUS=10)]}
        with self.assertRaisesRegex(AssertionError, 'Missing'):
            tuning.angle.pilot_proportional_gains(records, 100, 200)
        self.assertEqual(tuning.angle.pilot_proportional_gains({'PARM': []}, 100, 200),
                         ((.19, .19, .30), 'legacy-2F-2G-preset'))

    # @brief Ověřit i I/D/filter readback a nezaměnit požadovanou konfiguraci za skutečnou.
    # @param[in] None
    # @return None.
    # @details Chybné I nebo chybějící outer P musí zneplatnit srovnání.
    def test_all_tune_readback(self):
        records = dict(UARC=[dict(TimeUS=100), dict(TimeUS=200)], PARM=[
            dict(Name=line.split()[0], Value=float(line.split()[1]), TimeUS=10)
            for line in tuning.parameter_text(tuning.BASELINE).splitlines()])
        tuning.verify_logged_tune(records, tuning.BASELINE)
        records['PARM'][1]['Value'] = .05
        with self.assertRaises(AssertionError):
            tuning.verify_logged_tune(records, tuning.BASELINE)
        records['PARM'][1]['Value'] = .16
        records['PARM'].pop()
        with self.assertRaises(AssertionError):
            tuning.verify_logged_tune(records, tuning.BASELINE)

    # @brief Vyhodnotit pomalejší yaw ve skutečně dostupném dvousekundovém okně.
    # @param[in] None
    # @return None.
    # @details Analytický první řád s tau0,23s; settling0,9s nelze doložit jen prvním1s.
    def test_slow_yaw_has_sufficient_window(self):
        rows = [dict(t=i * .0025, dt=.0025, Tar2=.05,
                     Gyr2=.05 * (1 - math.exp(-i * .0025 / .23)), P2=0, I2=0, D2=0, Out2=0)
                for i in range(801)]
        result = tuning.plateau(rows, 2, 'rate')
        self.assertAlmostEqual(result['rise_10_90_s'], .23 * math.log(9), delta=.003)
        self.assertIsNotNone(result['settle_2percent_s'])
        self.assertAlmostEqual(result['settle_2percent_s'], -.23 * math.log(.02), delta=.003)

    # End Public functions
    # Private functions
    # @brief Vytvořit pouze analytická unit data s explicitně známou odezvou.
    # @param[in] oscillating True pro neustálenou odezvu.
    # @return Syntetické unit vzorky, nikoli letová evidence.
    # @details Soukromá pomocná metoda testu.
    def _rows(self, oscillating=False):
        result = []
        for i in range(401):
            t = i * .0025
            value = .15 * (1 - math.exp(-t / .03))
            if oscillating and t > .15:
                value += .02 * math.sin(2 * math.pi * 15 * t)
            result.append(dict(t=t, dt=.0025, Tar0=.15, Gyr0=value, P0=.01, I0=.001, D0=0, Out0=.011))
        return result

    # End Private functions


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
