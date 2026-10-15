"""No serial devices or third-party packages required."""
import io
import json
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
import host_protocol as protocol
import serial_console as console


class Clock:
    def __init__(self, stall=False):
        self.now = 0.0
        self.stall = stall

    def clock(self):
        return self.now

    def sleep(self, duration):
        self.now += 0.2 if self.stall else duration
        self.stall = False


class RecordingMock(console.MockLink):
    def __init__(self, skip=0):
        super().__init__()
        self.writes = []
        self.skip = skip

    def write(self, frame):
        if self.skip:
            self.skip -= 1
            return False
        self.writes.append((self.now, frame))
        return super().write(frame)


class SerialConsoleTests(unittest.TestCase):
    def test_rate_zero_tail_and_invalid_measurements(self):
        clock = Clock()
        link = RecordingMock()
        log = io.StringIO()
        result = console.run_session(link, 0.25, (0.1, 0, 0), log,
                                     clock=clock.clock, sleep=clock.sleep)
        self.assertEqual(result['motion_frames'], 25)
        self.assertEqual(result['tx_frames'], 35)
        self.assertAlmostEqual(result['average_motion_tx_hz'], 100, places=1)
        self.assertEqual(link.accepted, 35)
        self.assertEqual(result['crc_errors'], 0)
        for _, frame in link.writes[-10:]:
            self.assertEqual(struct.unpack_from('<fff', frame, 12), (0, 0, 0))
        records = [json.loads(line) for line in log.getvalue().splitlines()]
        self.assertGreater(len(records), 10)
        chassis = next(x for x in records if x['type'] == 0x10)
        self.assertEqual(chassis['wheel_rad_s'], [None] * 4)
        self.assertEqual(chassis['valid_mask'], 0)

    def test_monitor_is_read_only(self):
        clock = Clock()
        link = RecordingMock()
        result = console.run_session(link, 0.1, clock=clock.clock, sleep=clock.sleep)
        self.assertEqual(result['tx_frames'], 0)
        self.assertEqual(link.accepted, 0)
        self.assertGreater(result['rx_frames'], 0)

    def test_stall_does_not_catch_up(self):
        clock = Clock(stall=True)
        link = RecordingMock()
        result = console.run_session(link, 0.25, (0.1, 0, 0),
                                     clock=clock.clock, sleep=clock.sleep)
        self.assertLess(result['motion_frames'], 10)
        for (before, _), (after, _) in zip(link.writes, link.writes[1:]):
            self.assertGreaterEqual(after - before, 0.01 - 1e-8)

    def test_backpressure_skips_without_queue(self):
        clock = Clock()
        link = RecordingMock(skip=3)
        result = console.run_session(link, 0.1, (0.1, 0, 0),
                                     clock=clock.clock, sleep=clock.sleep)
        self.assertEqual(result['tx_skipped'], 3)
        sequences = [struct.unpack_from('<H', frame, 4)[0] for _, frame in link.writes]
        self.assertEqual(sequences, list(range(len(sequences))))

    def test_bad_commands_do_not_refresh_mock_watchdog(self):
        link = console.MockLink()
        link.write(protocol.motion_frame(0, 0, 0.1, 0, 0))
        link.poll(0.15)
        invalid = protocol.frame_bytes(1, 1, struct.pack('<Ifff', 150, float('nan'), 0, 0))
        link.write(invalid)
        bad_crc = bytearray(protocol.motion_frame(2, 150, 1, 0, 0))
        bad_crc[-1] ^= 1
        link.write(bad_crc)
        link.poll(0.2)
        self.assertEqual(link.accepted, 1)
        self.assertEqual(link.commands.crc_errors, 1)
        self.assertGreater(link.commands.invalid_frames, 0)
        decoder = protocol.FeedbackDecoder()
        records = []
        while link.pending:
            records.extend(decoder.feed(link.read()))
        self.assertEqual([x for x in records if x['type'] == 0x10][-1]['status_mask'], 1)

    def test_nan_is_hidden_even_if_valid_bit_is_set(self):
        result = console.visible_message(dict(type=0x11, yaw_rad=float('nan'), wz_rad_s=1, valid_mask=1))
        self.assertIsNone(result['yaw_rad'])
        self.assertIsNone(result['wz_rad_s'])


if __name__ == '__main__':
    unittest.main()
