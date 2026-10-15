#ifndef USER_BYTE_CODEC_H
#define USER_BYTE_CODEC_H

#include <stdint.h>
#include <string.h>
#include <float.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Protocol requires IEEE 754 binary32");

static inline uint16_t ReadU16LE(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t ReadU32LE(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t ReadI16LE(const uint8_t *p)
{
    uint32_t u = ReadU16LE(p);
    return u < 32768U ? (int32_t)u : (int32_t)u - 65536;
}
static inline void WriteU16LE(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static inline void WriteU32LE(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline float ReadF32LE(const uint8_t *p)
{
    uint32_t u = ReadU32LE(p);
    float f; memcpy(&f, &u, sizeof(f)); return f;
}
static inline void WriteF32LE(uint8_t *p, float f)
{
    uint32_t u; memcpy(&u, &f, sizeof(u)); WriteU32LE(p, u);
}

#endif
