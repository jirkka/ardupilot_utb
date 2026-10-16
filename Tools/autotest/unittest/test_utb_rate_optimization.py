#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Reject misleading tuning scores and incomplete allocator evidence."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utb_rate_optimization import allocation_analysis, parameters, quality, SELECTED  # noqa: E402


class TuningEvidence(unittest.TestCase):
    def test_bounds_reject_extreme_or_nonfinite(self):
        for value in [5., float('nan'), float('inf')]:
            gains = [list(g) for g in SELECTED]
            gains[0][0] = value
            with self.assertRaises(ValueError):
                parameters(gains)

    def test_undershoot_is_not_rewarded_as_zero_overshoot(self):
        rows = [dict(ref0=.15, target0=.15, gyro0=0, out0=0, i0=0, dt=.0025)]*400
        metric = dict(axis=[dict(iae_rad=.15, steps=[dict(steady_error_rads=.15, overshoot_percent=0)])])
        score = quality(rows, 0, metric)
        self.assertAlmostEqual(score['score'], .8)
        self.assertEqual(sum(score['weights'].values()), 1.)

    def test_missing_capture_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'Missing/invalid'):
                allocation_analysis(dict(UAFB=[], UALT=[]), [dict(seq=1)], Path(directory), SELECTED, True, 0)

    def test_real_normalized_projection_and_pid_attribution(self):
        record = dict(UAFB=[], UALT=[], UAMT=[dict(Seq=20, TimeUS=50000)])
        row = dict(seq=20, dt=.0025, out0=.1, out1=0., out2=0., i0=0., i1=0., i2=0.)
        motor = dict(Seq=20, TimeUS=50000, Av=1, Lim=0, G=2., T1=.7, T2=.7, T3=.9, T4=.9)
        record['UALT'].append(motor)
        for axis in range(3):
            val = .1 if axis == 0 else 0.
            record['UAFB'].append(dict(Seq=20, TimeUS=50000, Ax=axis, Av=1, PV=1, CSeq=20, Cmd=val, Alc=val,
                                       Raw=val, Res=0., Blk=0))
        with tempfile.TemporaryDirectory() as directory:
            result = allocation_analysis(record, [row], Path(directory), SELECTED, True, 0)
            self.assertLess(result['max_projection_error'], 1e-7)
            self.assertFalse(result['physical_moment_claimed'])
            self.assertEqual(json.loads((Path(directory)/'allocation-metrics.json').read_text())['complete_capture_sets'], 1)
        record['UAFB'][0]['CSeq'] = 19
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'PID attribution'):
                allocation_analysis(record, [row], Path(directory), SELECTED, True, 0)


if __name__ == '__main__':
    unittest.main()
