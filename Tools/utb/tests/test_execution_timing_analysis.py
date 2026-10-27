# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from utb_execution_timing_analysis import parse  # noqa: E402


def fixture(empty=False):
    n = 0 if empty else 1
    lines = [f'UTBExecutionV1,frozen,session_us=123,reason={6 if empty else 1},cycles=1,limit=1',
             'health,missing=0,duplicate=0,invalid=0,gaps=0,unsupported=' + str(int(empty)),
             'hooks,sum_us=7,calls=6,max_us=2',
             'metric,id,count,min,max,sum,min_cycle,max_cycle,min_us,max_us,over2500,over5000']
    for k, value in enumerate((100, 30, 130, 250, 90, 20, 200, 220)):
        lines.append(f'metric,{k},{n},{value if n else 4294967295},{value if n else 0},'
                     f'{value if n else 0},{n},{n},0,0,0,0')
    lines.append(f'context,rows={n},capacity=17,centre=main_to_motor_return')
    if n:
        lines.append('row,1,0,511,0,10,20,40,100,130,180,200,220,250')
    lines.append('end,session_us=123')
    return '\n'.join(lines) + '\n'


class ExecutionParserTest(unittest.TestCase):
    def test_complete(self):
        result = parse(fixture(), expected_session=123)
        self.assertEqual(result['metrics'][2]['sum'], 130)
        self.assertEqual(result['header']['cycles'], 1)

    def test_empty_context_is_valid_partial(self):
        self.assertEqual(parse(fixture(True))['rows'], [])

    def test_invalid_request_without_started_window(self):
        text = fixture(True).replace('reason=6,cycles=1,limit=1', 'reason=4,cycles=0,limit=0')
        text = text.replace('unsupported=1', 'unsupported=0')
        self.assertEqual(parse(text)['header']['cycles'], 0)

    def test_pending_rejected(self):
        with self.assertRaises(AssertionError):
            parse('UTBExecutionV1,pending\n')

    def test_interrupted_export(self):
        with self.assertRaises(AssertionError):
            parse(fixture().rsplit('end,', 1)[0])

    def test_mixed_session(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('end,session_us=123', 'end,session_us=124'))

    def test_unexpected_session(self):
        with self.assertRaises(AssertionError):
            parse(fixture(), expected_session=124)

    def test_corrupt_counts(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('metric,0,1,100,100,100', 'metric,0,2,100,100,100'))

    def test_corrupt_sum(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('metric,2,1,130,130,130', 'metric,2,1,130,130,131'))

    def test_duplicate_metric(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('metric,7,', 'metric,6,'))

    def test_missing_motor_event(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('row,1,0,511,0', 'row,1,0,503,0'))

    def test_backward_raw_time(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace(',100,130,180,', ',100,99,180,'))

    def test_duplicate_context(self):
        row = 'row,1,0,511,0,10,20,40,100,130,180,200,220,250'
        with self.assertRaises(AssertionError):
            parse(fixture().replace('context,rows=1', 'context,rows=2').replace(row, row + '\n' + row))

    def test_healthy_stop_with_missing(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('health,missing=0', 'health,missing=1'))

    def test_half_consume_record(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('row,1,0,511,0', 'row,1,0,507,0'))

    def test_unknown_stop_reason(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace('reason=1', 'reason=99'))

    def test_context_outside_aggregates(self):
        with self.assertRaises(AssertionError):
            parse(fixture().replace(',100,130,180,', ',100,140,180,'))

    def test_corrupt_page(self):
        with self.assertRaises((AssertionError, ValueError)):
            parse(fixture().replace('session_us=123,reason', 'session_us=xyz,reason'))


if __name__ == '__main__':
    unittest.main()
