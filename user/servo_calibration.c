#include "servo_calibration.h"
#include <math.h>

static bool PulseValid(uint16_t pulse) { return pulse >= 500U && pulse <= 2500U; }

/* 检查负向、中位、正向三个标定点及脉宽范围，支持舵机正装或反装。 */
bool ServoPwm_ValidCalibration(const ServoPwmCalibration *c)
{
    if (!c || !isfinite(c->negative_rad) || !isfinite(c->positive_rad) ||
        c->negative_rad >= 0.0f || c->positive_rad <= 0.0f ||
        c->positive_rad - c->negative_rad > 6.28318530718f ||
        !PulseValid(c->negative_us) || !PulseValid(c->center_us) || !PulseValid(c->positive_us)) return false;
    return (c->negative_us < c->center_us && c->center_us < c->positive_us) ||
           (c->negative_us > c->center_us && c->center_us > c->positive_us);
}

/* 按标定点将弧度角转换为整数微秒脉宽；角度越界则失败，不强行截到边界。 */
bool ServoPwm_Encode(const ServoPwmCalibration *c, float angle, uint16_t *pulse)
{
    if (!pulse || !ServoPwm_ValidCalibration(c) || !isfinite(angle) ||
        angle < c->negative_rad || angle > c->positive_rad) return false;
    float fraction = angle < 0.0f ? angle / c->negative_rad : angle / c->positive_rad;
    float end = angle < 0.0f ? c->negative_us : c->positive_us;
    float value = c->center_us + fraction * (end - c->center_us);
    *pulse = (uint16_t)lroundf(value);
    return true;
}
