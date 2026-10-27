# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from utb_usb_identity import USB_SERIAL, classify, discover  # noqa: E402


def device(port='COM5', vid=0x1209, pid=0x5741, serial=USB_SERIAL):
    return dict(port=port, vid=vid, pid=pid, serial=serial)


class USBIdentityTests(unittest.TestCase):
    def test_absent(self):
        self.assertEqual(classify([])['status'], 'absent')

    def test_unrelated_com5_is_not_the_board(self):
        self.assertEqual(classify([device(serial='other')])['status'], 'absent')

    def test_present(self):
        self.assertEqual(classify([device()])['port'], 'COM5')

    def test_changed_com_is_accepted(self):
        self.assertEqual(classify([device('COM8')])['port'], 'COM8')

    def test_duplicate_serial_is_ambiguous(self):
        value = classify([device(), device('COM8')])
        self.assertEqual(value['status'], 'ambiguous')
        self.assertIsNone(value['port'])

    def test_wrong_vid_pid_stops_without_a_port(self):
        for vid, pid in ((0x0483, 0xDF11), (0x1209, 0x5740)):
            with self.subTest(vid=vid, pid=pid):
                value = classify([device(vid=vid, pid=pid)])
                self.assertEqual(value['status'], 'unexpected_mode')
                self.assertIsNone(value['port'])

    def test_missing_port_is_not_present(self):
        self.assertEqual(classify([device(port=None)])['status'], 'ambiguous')

    def test_metadata_is_not_mutated(self):
        value = device()
        result = classify([value])
        result['candidates'][0]['port'] = 'changed'
        self.assertEqual(value['port'], 'COM5')

    def test_confirmation_required_before_scan(self):
        scan = Mock()
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                discover(Path(folder) / 'evidence.json', False, scan=scan)
        scan.assert_not_called()

    def test_success_is_one_scan_and_saved(self):
        scan = Mock(return_value=[device('COM9')])
        sleep = Mock()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'evidence.json'
            result = discover(path, True, 10, scan=scan, sleep=sleep)
            self.assertEqual(json.loads(path.read_text()), result)
            self.assertEqual(result['port'], 'COM9')
            self.assertFalse(result['serial_port_opened'])
        scan.assert_called_once()
        sleep.assert_not_called()

    def test_absent_wait_is_bounded(self):
        scan = Mock(return_value=[])
        sleep = Mock()
        with tempfile.TemporaryDirectory() as folder:
            result = discover(Path(folder) / 'evidence.json', True, 10, scan=scan, sleep=sleep)
        self.assertEqual(result['status'], 'absent')
        self.assertEqual(scan.call_count, 11)
        self.assertEqual(sleep.call_count, 10)

    def test_discovery_can_observe_a_return_without_reopening(self):
        scan = Mock(side_effect=[[], [device('COM8')]])
        with tempfile.TemporaryDirectory() as folder:
            result = discover(Path(folder) / 'evidence.json', True, 2, scan=scan, sleep=Mock())
        self.assertEqual(result['status'], 'present')
        self.assertEqual(len(result['samples']), 2)

    def test_existing_evidence_is_preserved(self):
        scan = Mock()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'evidence.json'
            path.write_text('previous error')
            with self.assertRaises(FileExistsError):
                discover(path, True, scan=scan)
            self.assertEqual(path.read_text(), 'previous error')
        scan.assert_not_called()

    def test_enumeration_error_is_not_retried(self):
        scan = Mock(side_effect=OSError('USB inventory failed'))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'evidence.json'
            result = discover(path, True, 10, scan=scan, sleep=Mock())
            self.assertIn('USB inventory failed', json.loads(path.read_text())['error'])
        self.assertEqual(result['status'], 'error')
        scan.assert_called_once()

    def test_invalid_wait_does_not_scan(self):
        scan = Mock()
        with tempfile.TemporaryDirectory() as folder:
            for number in (-1, 11, float('nan'), float('inf')):
                with self.subTest(number=number), self.assertRaises(ValueError):
                    discover(Path(folder) / 'evidence.json', True, number, scan=scan)
        scan.assert_not_called()
