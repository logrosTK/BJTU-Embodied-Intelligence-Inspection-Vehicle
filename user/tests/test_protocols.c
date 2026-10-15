#include "host_protocol.h"
#include "hwt101ct.h"
#include "dm3519.h"
#include "byte_codec.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void CheckNear(float a, float b) { assert(fabsf(a - b) < 0.0001f); }

static void TestHost(void)
{
    assert(Host_Crc16((const uint8_t *)"123456789", 9) == 0x29B1);
    uint8_t payload[16] = {0}, frame[HOST_MAX_FRAME];
    WriteU32LE(payload, 1234); WriteF32LE(payload + 4, 1.0f);
    WriteF32LE(payload + 8, -2.0f); WriteF32LE(payload + 12, 0.5f);
    size_t n = Host_Encode(HOST_MSG_MOTION, 65535, payload, 16, frame);
    assert(n == 26 && frame[4] == 0xFF && frame[5] == 0xFF);
    /* Every split point, then repeated coalesced frames including payload headers. */
    for (size_t split = 0; split <= n; ++split) {
        HostParser p; Host_Init(&p);
        Host_Feed(&p, frame, split, 20); Host_Feed(&p, frame + split, n - split, 21);
        assert(p.accepted_frames == 1); CheckNear(p.motion.vx, 1); CheckNear(p.motion.vy, -2);
        HostMotion m;
        assert(Host_GetMotion(&p, p.motion.received_ms + 199, &m));
        assert(!Host_GetMotion(&p, p.motion.received_ms + 200, &m)); CheckNear(m.vx, 0);
    }
    HostParser p; Host_Init(&p);
    uint8_t stream[52]; memcpy(stream, frame, 26); memcpy(stream + 26, frame, 26);
    Host_Feed(&p, stream, sizeof(stream), 10); assert(p.accepted_frames == 2);
    frame[25] ^= 1; Host_Feed(&p, frame, n, 11); assert(p.crc_errors == 1);
    frame[25] ^= 1;
    const uint8_t bad_length[] = {0xAA,0x55,1,1,0,0,0xFF,0xFF};
    Host_Feed(&p, bad_length, sizeof(bad_length), 12);
    Host_Feed(&p, frame, n, 13); assert(p.accepted_frames == 3);
    WriteF32LE(payload + 4, NAN); Host_Encode(1, 0, payload, 16, frame);
    Host_Feed(&p, frame, 26, 50); assert(p.accepted_frames == 3 && p.motion.received_ms == 13);
    Host_OnGap(&p); HostMotion m; assert(!Host_GetMotion(&p, 51, &m));
    WriteF32LE(payload + 4, INFINITY); Host_Encode(1, 1, payload, 16, frame);
    Host_Feed(&p, frame, 26, 60); assert(!p.motion.received);
    WriteF32LE(payload + 4, 0.2f); Host_Encode(1, 0, payload, 16, frame);
    Host_Feed(&p, frame, 26, UINT32_MAX - 50U);
    assert(Host_GetMotion(&p, 20, &m)); assert(!Host_GetMotion(&p, 150, &m));
    /* Deterministic noise fuzz followed by a good frame: recovery stays bounded. */
    uint32_t random = 7;
    for (unsigned i = 0; i < 10000; ++i) {
        uint8_t noise[31];
        for (unsigned j = 0; j < sizeof(noise); ++j) {
            random = random * 1664525U + 1013904223U; noise[j] = (uint8_t)(random >> 24);
        }
        Host_Feed(&p, noise, sizeof(noise), i);
        uint32_t before = p.accepted_frames;
        Host_Feed(&p, frame, 26, i); assert(p.accepted_frames == before + 1);
    }
}

static void Sum(uint8_t *p) { p[10] = 0; for (unsigned i = 0; i < 10; ++i) p[10] += p[i]; }

static void TestGyro(void)
{
    Hwt101ct h; Hwt_Init(&h);
    uint8_t velocity[11] = {0x55,0x52,0,0,0,0,0,0,0,0,0};
    WriteU16LE(velocity + 4, 1000); WriteU16LE(velocity + 6, (uint16_t)-500); Sum(velocity);
    uint8_t angle[11] = {0x55,0x53,0,0,0,0,0,0x40,0x23,0x01,0}; Sum(angle);
    for (unsigned i = 0; i < 11; ++i) Hwt_Feed(&h, velocity + i, 1, 10);
    Hwt_Feed(&h, angle, 11, 12);
    HwtSample s = Hwt_GetSample(&h, 20, 100);
    assert(s.valid == 3 && s.firmware_version == 0x123);
    CheckNear(s.yaw_rad, 1.57079632679f);
    CheckNear(s.wz_rad_s, -0.532632218f); /* -500/32768*2000*pi/180 */
    assert(s.raw_wz_rad_s > 1.06f && s.raw_wz_rad_s < 1.07f);
    angle[10] ^= 1; Hwt_Feed(&h, angle, 11, 80); assert(h.checksum_errors >= 1);
    angle[10] ^= 1; Hwt_Feed(&h, angle, 11, 90);
    s = Hwt_GetSample(&h, 110, 100); assert(s.valid == 1 && s.wz_rad_s == 0);
    Hwt_OnGap(&h); assert(Hwt_GetSample(&h, 111, 100).valid == 0);
    WriteU16LE(angle + 6, 0x8000); Sum(angle); Hwt_Feed(&h, angle, 11, UINT32_MAX - 5);
    s = Hwt_GetSample(&h, 10, 100); CheckNear(s.yaw_rad, -3.14159265359f);
    assert(s.valid == 1 && Hwt_GetSample(&h, 100, 100).valid == 0);
}

static void Param(Dm3519 *m, uint8_t reg, float value)
{
    DmCanFrame f = {.id=0x11,.length=8,.data={1,0,0x33,0,0,0,0,0}};
    f.data[3] = reg; WriteF32LE(f.data + 4, value); Dm3519_MarkRequest(m, reg);
    assert(Dm3519_Accept(m, &f, 1));
}

static void TestMotor(void)
{
    Dm3519 m; Dm3519_Init(&m, 1, 0x11);
    DmCanFrame status = {.id=0x11,.length=8,.data={0x11,0x80,0,0xA0,2,0,40,41}};
    assert(Dm3519_Accept(&m, &status, 10)); assert(!Dm3519_FeedbackValid(&m, 20, 100));
    Param(&m,0x15,12.5f); Param(&m,0x16,40); Param(&m,0x17,8); Param(&m,0x06,35);
    assert(Dm3519_FeedbackValid(&m, 20, 100));
    CheckNear(m.velocity_output_rad_s, 10.012210f); /* output shaft: no /19.2 */
    assert(m.raw_velocity == 2560 && m.raw_torque == 512 && m.state == 1);
    DmCanFrame f; assert(!Dm3519_VelocityFrame(&m, 6.283f, &f));
    f = (DmCanFrame){.id=0x11,.length=8,.data={1,0,0x33,0x0A,3,0,0,0}};
    Dm3519_MarkRequest(&m,0x0A); assert(Dm3519_Accept(&m,&f,20));
    assert(Dm3519_VelocityFrame(&m,6.283f,&f));
    const uint8_t expected[4] = {0x56,0x0E,0xC9,0x40};
    assert(f.id == 0x201 && f.length == 4 && memcmp(f.data,expected,4) == 0);
    assert(!Dm3519_VelocityFrame(&m,36,&f) && !Dm3519_VelocityFrame(&m,NAN,&f));
    Dm3519_StatusRequest(&m,&f); assert(f.id == 0x7FF && f.data[2] == 0xCC);
    Dm3519_EnableFrame(&m,true,&f); assert(f.id == 1 && f.data[7] == 0xFC);
    Dm3519_EnableFrame(&m,false,&f); assert(f.id == 1 && f.data[7] == 0xFD);
    Dm3519_ReadRegister(&m,0x15,&f); assert(f.id == 0x7FF && f.data[2] == 0x33);
    status.id = 0x12; assert(!Dm3519_Accept(&m,&status,50)); status.id = 0x11;
    status.data[0] = 0x81; assert(Dm3519_Accept(&m,&status,50));
    assert(!Dm3519_FeedbackValid(&m,51,100));
    status.data[0] = 0x11; assert(Dm3519_Accept(&m,&status,UINT32_MAX - 10U));
    assert(Dm3519_FeedbackValid(&m,20,100)); assert(!Dm3519_FeedbackValid(&m,100,100));
}

void TestProtocols(void)
{
    TestHost(); TestGyro(); TestMotor(); puts("PASS: protocol split/coalesce, CRC/resync/fuzz, watchdog/wrap, HWT scaling, motor frames/ranges");
}
