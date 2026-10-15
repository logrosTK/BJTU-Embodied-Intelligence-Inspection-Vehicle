"""USART2 protocol test console. Existing V1 messages only; never auto-enables.

No hardware: python serial_console.py demo --seconds 1
Receive:     python serial_console.py monitor --port COM5 --log feedback.jsonl
Motion:      python serial_console.py motion --port COM5 --vx 0.1 --seconds 2
Binary RX:   python serial_console.py decode capture.bin
Real ports require pyserial; demo/decode use only the Python standard library.
"""
import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
import struct
import sys
import time

from host_protocol import FeedbackDecoder, MotionDecoder, frame_bytes, motion_frame


@contextmanager
# 会话期间临时提高 Windows 定时精度，退出时恢复，便于按 100Hz 发送。
def precise_timer():
    """Request 1ms Windows timer resolution only during this console session."""
    timer = None
    if sys.platform == 'win32':
        import ctypes
        api = ctypes.WinDLL('winmm')
        if api.timeBeginPeriod(1) == 0:
            timer = api
    try:
        yield
    finally:
        if timer is not None:
            timer.timeEndPeriod(1)


class SerialLink:
    # 打开真实串口：115200、8N1；收发不自动附带电机使能命令。
    def __init__(self, port):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError('Real ports need pyserial: python -m pip install pyserial') from exc
        self.port = serial.Serial(port=None, baudrate=115200, bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                  timeout=0, write_timeout=0.05,
                                  xonxoff=False, rtscts=False, dsrdtr=False)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = port
        self.port.open()

    def poll(self, now):
        pass

    # 上一帧还没发完就跳过本次，避免历史速度指令在串口队列里积压。
    def write(self, frame):
        # No unbounded backlog of historical velocity frames.
        if self.port.out_waiting:
            return False
        if self.port.write(frame) != len(frame):
            raise OSError('Partial serial write; stopping sender')
        return True

    def read(self):
        return self.port.read(min(512, self.port.in_waiting))

    def close(self):
        self.port.close()


class MockLink:
    """Protocol-only fake device. All wheel/gyro valid bits remain ZERO.
    No simulated sensor data is presented as measurements. RX chunks are split.
    """
    def __init__(self):
        self.commands = MotionDecoder()
        self.pending = bytearray()
        self.now = 0.0
        self.last_motion = None
        self.accepted = 0
        self.last_feedback = -0.02
        self.last_diag = -1.0
        self.sequence = 0

    def write(self, frame):
        for _ in self.commands.feed(frame):
            self.accepted += 1
            self.last_motion = self.now
        return True

    def emit(self, kind, payload):
        # Bounded demo storage; a stalled consumer sees a gap and resyncs.
        if len(self.pending) > 1024:
            self.pending.clear()
        self.pending.extend(frame_bytes(kind, self.sequence, payload))
        self.sequence = (self.sequence + 1) & 0xffff

    # 模拟 50Hz 反馈、1Hz 诊断和 200ms 指令超时；所有传感器有效标记保持为零。
    def poll(self, now):
        self.now = now
        stamp = int(now * 1000) & 0xffffffff
        if now - self.last_feedback >= 0.02 - 1e-9:
            self.last_feedback = now
            timeout = self.last_motion is None or now - self.last_motion >= 0.2 - 1e-9
            self.emit(0x10, struct.pack('<I8fHH', stamp, *([0.0] * 8), 0, int(timeout)))
            self.emit(0x11, struct.pack('<IffH', 0, 0, 0, 0))
        if now - self.last_diag >= 1.0 - 1e-9:
            self.last_diag = now
            self.emit(0x12, struct.pack('<5I', stamp, self.commands.crc_errors, 0, 0, 0))

    def read(self):
        value = bytes(self.pending[:17])
        del self.pending[:17]
        return value

    def close(self):
        pass


# 显示或记录时，把无效测量值转为 None，避免把默认零值误认为真实反馈。
def visible_message(message):
    """Make invalid measurements explicit in display/logs; retain masks."""
    result = dict(message)
    valid = result.get('valid_mask', 0)
    if result['type'] == 0x10:
        for field, shift in (('wheel_rad_s', 0), ('steering_target_rad', 4)):
            result[field] = [v if valid & (1 << (i + shift)) and math.isfinite(v) else None
                             for i, v in enumerate(result[field])]
    elif result['type'] == 0x11:
        for field, bit in (('yaw_rad', 1), ('wz_rad_s', 2)):
            if not valid & bit or not math.isfinite(result[field]):
                result[field] = None
    return result


# 运行联调会话：按 100Hz 发速度并持续收反馈；延迟时跳过旧周期，不集中补发。
# velocity=None 表示只监听；发送模式退出时尽力补发十帧零速度，不保证物理链路送达。
def run_session(link, seconds, velocity=None, log=None, clock=time.perf_counter, sleep=time.sleep):
    """Bounded RX per pass; high-resolution monotonic 100Hz TX, no catch-up.
    perf_counter is required here: Python 3.12 Windows monotonic may use the
    ~15.6ms GetTickCount64 clock, which cannot schedule a 10ms transmit period.
    Returns statistics; motion exit sends a short, best-effort zero-speed tail.
    """
    decoder = FeedbackDecoder()
    stats = {'tx_frames': 0, 'tx_skipped': 0, 'rx_frames': 0, 'crc_errors': 0}
    latest = {}
    start = clock()
    next_tx = start
    next_display = start + 1.0
    sequence = 0

    def send(values):
        nonlocal sequence, next_tx
        sent_at = clock()
        frame = motion_frame(sequence, int((sent_at-start)*1000), *values)
        if link.write(frame):
            sequence = (sequence + 1) & 0xffff
            stats['tx_frames'] += 1
        else:
            stats['tx_skipped'] += 1
        # Preserve normal timing phase; skip missed cycles instead of bursting.
        next_tx += 0.01
        if next_tx < clock():
            next_tx = clock() + 0.01

    def receive():
        for message in decoder.feed(link.read()):
            stats['rx_frames'] += 1
            item = visible_message(message)
            item['pc_rx_elapsed_ms'] = round((clock()-start)*1000, 3)
            latest[item['type']] = item
            if log:
                log.write(json.dumps(item, allow_nan=False) + '\n')
        stats['crc_errors'] = decoder.crc_errors

    try:
        while seconds == 0 or clock()-start < seconds:
            link.poll(clock()-start)
            if velocity is not None and clock() >= next_tx:
                send(velocity)
            receive()
            now = clock()
            if now >= next_display:
                print(json.dumps({'elapsed_s': round(now-start, 2), **stats, 'latest': latest}, allow_nan=False))
                next_display = now + 1.0
            sleep(0.001)
    finally:
        stats['motion_frames'] = stats['tx_frames']
        stats['session_elapsed_s'] = round(clock()-start, 6)
        stats['average_motion_tx_hz'] = round(stats['motion_frames']/max(clock()-start, 1e-9), 2)
        if velocity is not None:
            # No implicit enable/disable command. A failed physical link cannot
            # guarantee delivery; never loop forever trying to flush it.
            try:
                for _ in range(10):
                    link.poll(clock()-start)
                    send((0.0, 0.0, 0.0))
                    receive()
                    sleep(0.01)
            except (OSError, RuntimeError):
                stats['zero_tail_failed'] = True
        stats['crc_errors'] = decoder.crc_errors
    return stats


# 命令行入口，选择真实串口监听、发速度、无硬件演示或离线解码。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('monitor', 'motion', 'demo'):
        command = commands.add_parser(name)
        if name != 'demo':
            command.add_argument('--port', required=True)
        command.add_argument('--seconds', type=float, default=0 if name == 'monitor' else 5)
        command.add_argument('--log', type=Path, help='Create a new JSONL file; existing files are not overwritten')
        if name != 'monitor':
            for field in ('vx', 'vy', 'wz'):
                command.add_argument('--'+field, type=float, default=0.0)
    commands.add_parser('decode').add_argument('capture', type=Path)
    args = parser.parse_args(argv)
    if args.command == 'decode':
        decoder = FeedbackDecoder()
        with args.capture.open('rb') as source:
            for chunk in iter(lambda: source.read(512), b''):
                for message in decoder.feed(chunk):
                    print(json.dumps(visible_message(message), allow_nan=False))
        print(f'crc_errors={decoder.crc_errors}, trailing_bytes={len(decoder.buffer)}', file=sys.stderr)
        return 0
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be finite and nonnegative (0 runs until interrupted)')
    velocity = None if args.command == 'monitor' else (args.vx, args.vy, args.wz)
    if velocity is not None:
        try:
            motion_frame(0, 0, *velocity)  # Validate before opening the port.
        except (ValueError, OverflowError) as exc:
            parser.error(str(exc))
    link = None
    log = None
    try:
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log = args.log.open('x', encoding='utf-8')
        link = MockLink() if args.command == 'demo' else SerialLink(args.port)
        if args.command == 'demo':
            print('DEMO: protocol mock; all wheel/gyro validity bits are zero. No hardware I/O.')
        with precise_timer():
            result = run_session(link, args.seconds, velocity, log)
        print(json.dumps(result, allow_nan=False))
        return 0
    except KeyboardInterrupt:
        print('Stopped; motion mode attempted a zero-speed tail.', file=sys.stderr)
        return 130
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        if log:
            log.close()
        if link:
            link.close()


if __name__ == '__main__':
    raise SystemExit(main())
