#include "hwt101ct.h"
#include "byte_codec.h"
#include <string.h>

/* Manufacturer HWT101 protocol, not the sensor's physical +/-400 deg/s range:
   https://wit-motion.yuque.com/wumwnr/ltst03/seuiba
   0x52 bytes 4..5 = raw Wz, 6..7 = corrected Wz; signed int16 /32768 *2000 deg/s.
   0x53 bytes 6..7 = signed yaw /32768 *180 degrees. */
#define PI_F 3.14159265358979323846f

/* 清空陀螺仪解析器，角度和角速度初始均无效。 */
void Hwt_Init(Hwt101ct *h) { memset(h, 0, sizeof(*h)); }
/* 丢数据时废弃半帧，并使已有测量值失效。 */
void Hwt_OnGap(Hwt101ct *h) { h->used = 0; h->sample.valid = 0; }

static void Discard(Hwt101ct *h)
{
    --h->used;
    memmove(h->frame, h->frame + 1, h->used);
}

/* 校验并解析 0x52 角速度帧和 0x53 角度帧，转换为 rad/s 和 rad。
   两种数据分别记录接收时间；角速度按协议的 2000°/s 编码比例换算。 */
void Hwt_Feed(Hwt101ct *h, const uint8_t *data, size_t length, uint32_t rx_ms)
{
    for (size_t i = 0; i < length; ++i) {
        h->frame[h->used++] = data[i];
        while (h->used) {
            uint8_t *b = h->frame;
            if (b[0] != 0x55 || (h->used >= 2 && b[1] != 0x52 && b[1] != 0x53)) {
                Discard(h); continue;
            }
            if (h->used < 11) break;
            uint8_t sum = 0;
            for (unsigned j = 0; j < 10; ++j) sum = (uint8_t)(sum + b[j]);
            if (sum != b[10]) { ++h->checksum_errors; Discard(h); continue; }
            if (b[1] == 0x52) {
                h->sample.raw_wz_rad_s = ReadI16LE(b + 4) * (2000.0f * PI_F / (32768.0f * 180.0f));
                h->sample.wz_rad_s = ReadI16LE(b + 6) * (2000.0f * PI_F / (32768.0f * 180.0f));
                h->sample.wz_ms = rx_ms; h->sample.valid |= 2;
            } else {
                h->sample.yaw_rad = ReadI16LE(b + 6) * (PI_F / 32768.0f);
                h->sample.yaw_ms = rx_ms; h->sample.valid |= 1;
                h->sample.firmware_version = ReadU16LE(b + 8);
            }
            ++h->accepted_frames; h->used = 0;
        }
    }
}

/* 获取测量快照，分别检查角度和角速度是否过期；无效字段置零并清除对应标记。 */
HwtSample Hwt_GetSample(const Hwt101ct *h, uint32_t now_ms, uint32_t max_age_ms)
{
    HwtSample s = h->sample;
    if (!max_age_ms || (uint32_t)(now_ms - s.yaw_ms) >= max_age_ms) s.valid &= (uint8_t)~1U;
    if (!max_age_ms || (uint32_t)(now_ms - s.wz_ms) >= max_age_ms) s.valid &= (uint8_t)~2U;
    if (!(s.valid & 1)) s.yaw_rad = 0.0f;
    if (!(s.valid & 2)) s.wz_rad_s = s.raw_wz_rad_s = 0.0f;
    return s;
}
