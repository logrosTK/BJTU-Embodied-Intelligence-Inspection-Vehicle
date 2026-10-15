#ifndef USER_HOST_PROTOCOL_H
#define USER_HOST_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HOST_MAX_PAYLOAD 128U
#define HOST_MAX_FRAME (HOST_MAX_PAYLOAD + 10U)
#define HOST_COMMAND_TIMEOUT_MS 200U
#define HOST_MSG_MOTION 0x01U
#define HOST_MSG_CHASSIS 0x10U
#define HOST_MSG_GYRO 0x11U
#define HOST_MSG_DIAGNOSTICS 0x12U

typedef struct {
    uint32_t sender_ms;
    uint32_t received_ms;
    float vx, vy, wz;
    uint16_t sequence;
    bool received;
} HostMotion;

typedef struct {
    uint8_t buffer[26]; /* Only the approved 16-byte motion payload is accepted. */
    uint16_t used;
    HostMotion motion;
    uint32_t crc_errors, invalid_frames, accepted_frames;
} HostParser;

uint16_t Host_Crc16(const uint8_t *data, size_t length);
size_t Host_Encode(uint8_t type, uint16_t sequence, const uint8_t *payload,
                   uint16_t length, uint8_t out[HOST_MAX_FRAME]);
void Host_Init(HostParser *p);
/* Feed in RX order. Timestamp is the DMA receive event time, not parse time. */
void Host_Feed(HostParser *p, const uint8_t *data, size_t length, uint32_t rx_ms);
/* A transport gap invalidates the command and any partial frame immediately. */
void Host_OnGap(HostParser *p);
bool Host_GetMotion(const HostParser *p, uint32_t now_ms, HostMotion *out);

#endif
