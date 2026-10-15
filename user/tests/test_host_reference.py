"""Independent Python/stdlib CRC and byte-layout cross-checks."""
import binascii
import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location(
    'host_reference', Path(__file__).parents[1] / 'tools' / 'host_protocol.py')
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


class HostReferenceTests(unittest.TestCase):
    def test_command(self):
        frame = reference.motion_frame(65535, 1234, 1, -2, 0.5)
        self.assertEqual(len(frame), 26)
        self.assertEqual(frame[:8].hex(), 'aa550101ffff1000')
        self.assertEqual(frame[8:24], struct.pack('<Ifff', 1234, 1, -2, 0.5))
        self.assertEqual(binascii.crc_hqx(b'123456789', 0xffff), 0x29b1)
        with self.assertRaises(ValueError):
            reference.motion_frame(0, 0, float('nan'), 0, 0)

    def test_decode(self):
        payload = struct.pack('<IffH', 12, 1.5, -0.2, 3)
        raw = struct.pack('<2sBBHH', b'\xaa\x55', 1, 0x11, 2, 14) + payload
        raw += struct.pack('<H', binascii.crc_hqx(raw[2:], 0xffff))
        for split in range(len(raw) + 1):
            decoder = reference.FeedbackDecoder()
            messages = decoder.feed(raw[:split]) + decoder.feed(raw[split:])
            self.assertEqual(len(messages), 1)
            self.assertEqual(messages[0]['valid_mask'], 3)
            self.assertAlmostEqual(messages[0]['wz_rad_s'], -0.2, places=6)
        decoder = reference.FeedbackDecoder()
        bad = bytearray(raw); bad[-1] ^= 1
        self.assertEqual(len(decoder.feed(b'noise' + bad + raw * 3)), 3)
        self.assertEqual(decoder.crc_errors, 1)


if __name__ == '__main__':
    unittest.main()
