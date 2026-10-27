# AP_FLAKE8_CLEAN
# GNU GPL v3 or later.
# @file test_flight_bin_analysis.py
# @brief Persistence loss, source identity, real FMT decoding and version boundaries.
# @author Jiri Kovar
# @version 2026-10-26
import copy
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from utb_flight_bin_analysis import analyze, read_bin  # noqa: E402


def fixture():
    records = []
    for seq in (1, 2):
        for axis in range(3):
            common = dict(TimeUS=100000+seq*100000, Seq=seq, Ax=axis, Md=0, Flg=3)
            records.append(dict(common, mavpackettype='UFPI', AP=.1, AI=.2, AD=.3, AFF=.4, ADF=.5,
                                UP=.1, UI=.2, UD=.3, UFF=.4, UDF=.5))
            records.append(dict(common, mavpackettype='UFRS', SrcUS=common['TimeUS']-2400, Lp=seq*40,
                                Src=seq*40-1, Dt=.0025, Gy=.15, AT=.2, UT=.2, AO=.3, UO=.3, AF=.4, UF=.4))
        records.append(dict(TimeUS=common['TimeUS'], Seq=seq, Flg=3, SrcUS=common['TimeUS']-2400,
                            mavpackettype='UFAT', Q1=1., Q2=0., Q3=0., Q4=0., A1=1., A2=0., A3=0., A4=0.,
                            U1=1., U2=0., U3=0., U4=0.))
    records.append(dict(mavpackettype='UFLS', TimeUS=400000, Start=100000, V=1, St=2, Rsn=1, Hz=10,
                        Req=2, Que=2, Acc=2, Drp=0, Inv=0, Uns=0, Rej=0, HWM=1, Dep=0, Sec=1))
    return records


class FlightBinAnalysis(unittest.TestCase):
    def test_complete(self):
        r = analyze(fixture())
        self.assertTrue(r['qualified'], r['errors'])
        self.assertEqual(r['complete'], 2)
        self.assertTrue(all(v['max_abs'] == v['rmse'] == 0 for v in r['metrics'].values()))

    def test_order_independent(self):
        self.assertTrue(analyze(list(reversed(fixture())))['qualified'])

    def test_missing_packet(self):
        r = fixture()
        del r[2]
        self.assertEqual(analyze(r)['partial'], 1)
        self.assertFalse(analyze(r)['qualified'])

    def test_missing_whole_set(self):
        r = fixture()[7:]
        self.assertIn('Accepted sets missing from persistent BIN', analyze(r)['errors'])

    def test_backend_partial(self):
        r = fixture()
        del r[2]
        r[-1].update(Acc=1, Rej=1)
        self.assertFalse(analyze(r)['qualified'])

    def test_dropped(self):
        r = fixture()
        r[-1].update(Req=3, Drp=1)
        self.assertFalse(analyze(r)['qualified'])

    def test_duplicate(self):
        r = fixture()
        r.insert(0, copy.copy(r[0]))
        self.assertEqual(analyze(r)['duplicates'], 1)

    def test_source_and_nan(self):
        for key, value in [('Src', 0), ('Dt', float('nan')), ('Gy', float('inf')), ('Md', 1), ('Flg', 0)]:
            r = fixture()
            r[1][key] = value
            self.assertGreater(analyze(r)['invalid'], 0, key)

    def test_mismatch(self):
        r = fixture()
        r[0]['UP'] += .001
        self.assertIn('Numerical mismatch', analyze(r)['errors'])

    def test_quaternion_sign(self):
        r = fixture()
        r[6]['U1'] = -1
        self.assertTrue(analyze(r)['qualified'])

    def test_no_terminal_or_no_data(self):
        self.assertFalse(analyze(fixture()[:-1])['qualified'])
        self.assertFalse(analyze([])['qualified'])

    def test_multiple_sessions_or_version(self):
        r = fixture()
        r.append(dict(r[-1], Start=999))
        self.assertFalse(analyze(r)['qualified'])
        r = fixture()
        r[-1]['V'] = 2
        self.assertFalse(analyze(r)['qualified'])

    def test_actual_fmt_bin_packing(self):
        root = Path(__file__).resolve().parents[3]
        import re
        header = (root/'libraries/AP_UTB/FlightBinLogStructure.h').read_text()
        definitions = re.findall(r'"(UF..)", "([QIBHf]+)", "([^"]+)"', header)
        self.assertEqual(len(definitions), 4)
        codes = dict(Q='Q', I='I', B='B', H='H', f='f')
        payload = bytearray()
        formats = {}
        for index, (name, fmt, labels) in enumerate(definitions, 230):
            self.assertLessEqual(len(labels), 64)
            self.assertLessEqual(len(fmt), 16)
            wire = '<' + ''.join(codes[c] for c in fmt)
            length = 3 + struct.calcsize(wire)
            payload += b'\xa3\x95\x80' + struct.pack(
                '<BB4s16s64s', index, length, name.encode(), fmt.encode(), labels.encode())
            formats[name] = (index, wire, labels.split(','))
        for r in fixture():
            index, wire, fields = formats[r['mavpackettype']]
            payload += b'\xa3\x95' + bytes([index]) + struct.pack(wire, *(r[k] for k in fields))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'fixture.BIN'
            path.write_bytes(payload)
            result = read_bin(path)
            self.assertTrue(result['qualified'], result['errors'])


if __name__ == '__main__':
    unittest.main()
