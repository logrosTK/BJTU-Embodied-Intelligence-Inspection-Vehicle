"""ROS2-side byte codec reference. No serial I/O or ROS dependencies.

Pass incoming UART byte chunks to FeedbackDecoder.feed(). Invalid fields remain
explicitly invalid; target steering angles are never measured steering angles.
"""
import binascii
import math
import struct

HEADER = struct.Struct('<2sBBHH')
LENGTHS = {0x10: 40, 0x11: 14, 0x12: 20}


# 打包通用串口帧并附加 CRC；与 STM32 使用相同的帧格式和小端编码。
def frame_bytes(kind, sequence, payload):
    """Envelope for supported payloads; the device still validates type/length."""
    if not 0 <= kind <= 255 or len(payload) > 128:
        raise ValueError('Invalid message type or payload length')
    frame = HEADER.pack(b'\xaa\x55', 1, kind, sequence & 0xffff, len(payload)) + payload
    return frame + struct.pack('<H', binascii.crc_hqx(frame[2:], 0xffff))


# 生成已确认的速度指令帧；vx、vy 单位为 m/s，wz 为 rad/s，不接受 NaN 或无穷值。
def motion_frame(sequence, sender_ms, vx, vy, wz):
    """Build the approved 26-byte velocity command (m/s, m/s, rad/s)."""
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        raise ValueError('Motion values must be finite')
    payload = struct.pack('<Ifff', sender_ms & 0xffffffff, vx, vy, wz)
    return frame_bytes(1, sequence, payload)


class _FrameDecoder:
    def __init__(self, lengths):
        self.lengths = lengths
        self.buffer = bytearray()
        self.crc_errors = 0
        self.invalid_frames = 0

    # 从任意长度的字节块中找完整帧，处理拆包、粘包和 CRC 错误后的重新同步。
    def frames(self, chunk):
        # Process incrementally to keep internal storage bounded by one frame.
        for byte in chunk:
            self.buffer.append(byte)
            while self.buffer:
                b = self.buffer
                if b[0] != 0xaa or (len(b) >= 2 and b[1] != 0x55):
                    del b[0]
                    continue
                if len(b) < HEADER.size:
                    break
                _, version, kind, sequence, length = HEADER.unpack_from(b)
                if version != 1 or self.lengths.get(kind) != length:
                    self.invalid_frames += 1
                    del b[0]
                    continue
                total = length + 10
                if len(b) < total:
                    break
                if binascii.crc_hqx(b[2:total-2], 0xffff) != struct.unpack_from('<H', b, total-2)[0]:
                    self.crc_errors += 1
                    del b[0]
                    continue
                payload = bytes(b[8:total-2])
                del b[:total]
                yield kind, sequence, payload


class FeedbackDecoder(_FrameDecoder):
    """Incremental decoder: supports partial frames, coalescing and resync."""
    def __init__(self):
        super().__init__(LENGTHS)

    # 将底盘、陀螺仪和诊断帧解码为字典；保留有效标记，舵角字段表示目标值。
    def feed(self, chunk):
        messages = []
        for kind, sequence, payload in self.frames(chunk):
            message = {'type': kind, 'sequence': sequence}
            if kind == 0x10:
                values = struct.unpack('<I8fHH', payload)
                message.update(stm32_ms=values[0], wheel_rad_s=values[1:5],
                               steering_target_rad=values[5:9], valid_mask=values[9],
                               status_mask=values[10])
            elif kind == 0x11:
                stamp, yaw, wz, valid = struct.unpack('<IffH', payload)
                message.update(stm32_rx_ms=stamp, yaw_rad=yaw,
                               wz_rad_s=wz, valid_mask=valid)
            else:
                values = struct.unpack('<5I', payload)
                message.update(zip(('stm32_ms', 'crc_errors', 'rx_overflows',
                                    'telemetry_replaced', 'uart_errors'), values))
            messages.append(message)
        return messages


class MotionDecoder(_FrameDecoder):
    """Mock/diagnostic receiver for the EXISTING 0x01 only; not a firmware port."""
    def __init__(self):
        super().__init__({1: 16})

    # 供模拟设备解析速度指令；只接受现有 0x01 消息，并检查速度数值是否有效。
    def feed(self, chunk):
        messages = []
        for kind, sequence, payload in self.frames(chunk):
            stamp, vx, vy, wz = struct.unpack('<Ifff', payload)
            if not all(math.isfinite(v) for v in (vx, vy, wz)):
                self.invalid_frames += 1
                continue
            messages.append(dict(type=kind, sequence=sequence, sender_ms=stamp,
                                 vx=vx, vy=vy, wz=wz))
        return messages
