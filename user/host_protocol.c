#include "host_protocol.h"
#include "byte_codec.h"
#include <math.h>
#include <string.h>

/* 计算 CRC16/CCITT-FALSE 校验值，用于检查串口帧是否损坏。 */
uint16_t Host_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned b = 0; b < 8; ++b)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U :
                                                           (uint32_t)crc << 1);
    }
    return crc;
}

/* 打包帧头、类型、序号、长度、负载和 CRC；返回整帧字节数，参数错误返回 0。 */
size_t Host_Encode(uint8_t type, uint16_t sequence, const uint8_t *payload,
                   uint16_t length, uint8_t out[HOST_MAX_FRAME])
{
    if (length > HOST_MAX_PAYLOAD || out == NULL || (length && payload == NULL))
        return 0;
    out[0] = 0xAA; out[1] = 0x55; out[2] = 1; out[3] = type;
    WriteU16LE(out + 4, sequence); WriteU16LE(out + 6, length);
    if (length) memcpy(out + 8, payload, length);
    WriteU16LE(out + 8 + length, Host_Crc16(out + 2, 6U + length));
    return 10U + length;
}

/* 清空解析缓存与统计，上电时没有有效运动指令。 */
void Host_Init(HostParser *p) { memset(p, 0, sizeof(*p)); }
/* 丢数据后废弃半帧和旧运动指令，等待完整新帧。 */
void Host_OnGap(HostParser *p) { p->used = 0; p->motion.received = false; }

static void Discard(HostParser *p, uint16_t n)
{
    p->used -= n;
    memmove(p->buffer, p->buffer + n, p->used);
}

/* 逐字节解析速度帧，支持拆包、粘包和错帧后重新找帧头；坏帧不刷新超时计时。
   rx_ms 必须使用接收时刻，不能用较晚的解析时刻代替。 */
void Host_Feed(HostParser *p, const uint8_t *data, size_t length, uint32_t rx_ms)
{
    for (size_t i = 0; i < length; ++i) {
        p->buffer[p->used++] = data[i];
        while (p->used) {
            uint8_t *b = p->buffer;
            if (b[0] != 0xAA || (p->used >= 2 && b[1] != 0x55)) {
                Discard(p, 1); continue;
            }
            if (p->used < 8) break;
            if (b[2] != 1 || b[3] != HOST_MSG_MOTION || ReadU16LE(b + 6) != 16) {
                ++p->invalid_frames; Discard(p, 1); continue;
            }
            if (p->used < 26) break;
            if (Host_Crc16(b + 2, 22) != ReadU16LE(b + 24)) {
                ++p->crc_errors; Discard(p, 1); continue;
            }
            float vx = ReadF32LE(b + 12), vy = ReadF32LE(b + 16), wz = ReadF32LE(b + 20);
            if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz)) {
                ++p->invalid_frames;
            } else {
                p->motion = (HostMotion){
                    .sender_ms = ReadU32LE(b + 8), .received_ms = rx_ms,
                    .vx = vx, .vy = vy, .wz = wz, .sequence = ReadU16LE(b + 4),
                    .received = true
                };
                ++p->accepted_frames;
            }
            Discard(p, 26);
        }
    }
}

/* 读取最新速度；未收到指令或已满 200ms 未更新时返回 false，并输出零速度。 */
bool Host_GetMotion(const HostParser *p, uint32_t now_ms, HostMotion *out)
{
    *out = p->motion;
    bool valid = out->received && (uint32_t)(now_ms - out->received_ms) < HOST_COMMAND_TIMEOUT_MS;
    if (!valid) out->vx = out->vy = out->wz = 0.0f;
    return valid;
}
