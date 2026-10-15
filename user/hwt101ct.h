#ifndef USER_HWT101CT_H
#define USER_HWT101CT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    float yaw_rad, wz_rad_s, raw_wz_rad_s;
    uint32_t yaw_ms, wz_ms;
    uint16_t firmware_version;
    uint8_t valid; /* bit0 yaw, bit1 corrected angular velocity */
} HwtSample;

typedef struct {
    uint8_t frame[11], used;
    HwtSample sample;
    uint32_t checksum_errors, accepted_frames;
} Hwt101ct;

void Hwt_Init(Hwt101ct *h);
void Hwt_OnGap(Hwt101ct *h);
void Hwt_Feed(Hwt101ct *h, const uint8_t *data, size_t length, uint32_t rx_ms);
/* Caller supplies the data-age limit; zero marks all data invalid. */
HwtSample Hwt_GetSample(const Hwt101ct *h, uint32_t now_ms, uint32_t max_age_ms);

#endif
