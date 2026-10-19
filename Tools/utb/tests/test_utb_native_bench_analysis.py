# AP_FLAKE8_CLEAN
# GNU GPL v3 or later. Author: Jiri Kovar, revision 2026-10-19.
"""Numerical summaries must not hide missing, unequal or unhealthy diagnostics."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'native_bench_analysis', Path(__file__).resolve().parents[1]/'tools/utb_native_bench_analysis.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Message:
    def __init__(self, kind, row):
        self.kind, self.row = kind, row

    def get_type(self):
        return self.kind

    def to_dict(self):
        return dict(self.row, mavpackettype=self.kind)


class Reader:
    def __init__(self, rows):
        self.rows = iter(rows)

    def recv_match(self):
        return next(self.rows, None)

    def close(self):
        pass


def sample(seq, time_us, delta=0, flags=0):
    common = dict(TimeUS=time_us, N=seq)
    rows = []
    for axis in range(3):
        row = dict(common, Ph=0, Ax=axis, Dt=.0025, Ref=1., Gyr=.1)
        row.update({k: 1. for k in ['AT', 'UT', 'AP', 'UP', 'AI', 'UI', 'AD', 'UD']})
        row['AP'] += delta
        rows.append(Message('U3BD', row))
        rows.append(Message('U3BF', dict(common, Ax=axis, **{k: 0. for k in
                            ['AF', 'UF', 'ADF', 'UDF', 'AS', 'US', 'AO', 'UO', 'AMF', 'UMF', 'ATot', 'UTot']})))
    rows.append(Message('U3BA', dict(common, **{k: 0. for k in ['AR', 'AP', 'AY', 'UR', 'UP', 'UY']})))
    rows.append(Message('U3BQ', dict(common, **{k: 0. for k in ['AW', 'AX', 'AY', 'AZ', 'UW', 'UX', 'UY', 'UZ']})))
    rows.append(Message('U3BT', dict(common, APus=10, UTus=11, Wall=20000, LogUs=50, Flags=flags, Ver=2)))
    return rows


class AnalysisTest(unittest.TestCase):
    def analyse(self, rows):
        with tempfile.TemporaryDirectory() as root:
            with patch.object(MODULE.mavutil, 'mavlink_connection', return_value=Reader(rows)):
                source = Path(root)/'input.bin'
                source.write_bytes(b'')  # Reader is explicitly mocked for these unit cases.
                result = MODULE.analyse(source, Path(root)/'result', 'SITL')
            self.assertEqual(json.loads((Path(root)/'result/summary.json').read_text()), result)
            self.assertTrue((Path(root)/'result/comparison.csv').exists())
            return result

    def test_error_metrics_and_missing_samples(self):
        result = self.analyse(sample(0, 1000000)+sample(1, 1020000, delta=.25))
        error = result['error']['phase0/AP-UP']
        self.assertEqual(error['unequal'], 3)
        self.assertEqual(result['unequal_axis_samples'], 3)
        self.assertEqual(result['unequal_sets'], 1)
        self.assertEqual(error['max_abs'], .25)
        self.assertAlmostEqual(error['rmse'], math.sqrt(.25**2/2))
        self.assertEqual(result['observed_hz'], 50)
        self.assertIn(2, result['missing_sequence_numbers'])
        self.assertNotIn(1, result['missing_sequence_numbers'])

    def test_health_is_excluded_not_silently_passed(self):
        result = self.analyse(sample(0, 1000000, delta=.25, flags=16))
        self.assertEqual(result['excluded_input_or_health_sets'], 1)
        self.assertEqual(result['error'], {})

    def test_inconsistent_input_taints_profile(self):
        rows = sample(0, 1000000, flags=8)+sample(1, 1020000)
        result = self.analyse(rows)
        self.assertEqual(result['excluded_input_or_health_sets'], 2)
        self.assertEqual(result['error'], {})

    def test_partial_set_reported(self):
        result = self.analyse(sample(0, 1000000)+sample(1, 1020000)[:-1])
        self.assertEqual(result['partial_sets'], 1)
        self.assertEqual(result['complete_sets'], 1)

    def test_transport_endpoint_rejected_before_open(self):
        with patch.object(MODULE.mavutil, 'mavlink_connection') as connection:
            with self.assertRaisesRegex(ValueError, 'local BIN'):
                MODULE.analyse(Path('COM5'), Path('unused'), 'unknown')
            connection.assert_not_called()

    def test_nonfinite_output_is_not_hidden(self):
        rows = sample(0, 1000000)
        rows[0].row['AP'] = float('nan')
        result = self.analyse(rows)
        self.assertEqual(result['nonfinite_values'], 1)
        self.assertEqual(result['invalid_axis_samples'], 1)
        self.assertEqual(result['invalid_sets'], 1)

    def test_duplicate_rejected(self):
        rows = sample(0, 1000000)
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.analyse(rows+[rows[0]])

    def test_multiple_sessions_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Multiple RUN'):
            self.analyse(sample(0, 1000000)+sample(0, 2000000))


if __name__ == '__main__':
    unittest.main()
