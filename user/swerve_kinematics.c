#include "swerve_kinematics.h"
#include <math.h>
#include <stddef.h>

#define SWERVE_PI 3.14159265358979323846f
#define SWERVE_TAU (2.0f * SWERVE_PI)

/* 由车体前进、横移和旋转速度计算四轮目标；轮序为左前、右前、左后、右后。
   轮速用 rad/s、转角用 rad；某轮速度为零时保持传入的参考角。 */
bool Swerve_Inverse(const SwerveGeometry *g, const SwerveTwist *t,
                    const float reference[SWERVE_WHEELS], SwerveTargets *out)
{
    if (!g || !t || !reference || !out ||
        !isfinite(g->wheel_radius_m) || g->wheel_radius_m <= 0.0f ||
        !isfinite(g->pivot_track_m) || g->pivot_track_m <= 0.0f ||
        !isfinite(g->pivot_wheelbase_m) || g->pivot_wheelbase_m <= 0.0f ||
        !isfinite(t->vx_m_s) || !isfinite(t->vy_m_s) || !isfinite(t->wz_rad_s)) return false;

    SwerveTargets result;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
        if (!isfinite(reference[i])) return false;
        float x = (i < 2U ? 0.5f : -0.5f) * g->pivot_wheelbase_m;
        float y = ((i & 1U) ? -0.5f : 0.5f) * g->pivot_track_m;
        float vx = t->vx_m_s - t->wz_rad_s * y;
        float vy = t->vy_m_s + t->wz_rad_s * x;
        if (!isfinite(vx) || !isfinite(vy)) return false;
        float speed = hypotf(vx, vy) / g->wheel_radius_m;
        if (!isfinite(speed)) return false;
        result.wheel[i].speed_rad_s = speed;
        result.wheel[i].angle_rad = (vx == 0.0f && vy == 0.0f) ? reference[i] : atan2f(vy, vx);
    }
    *out = result;
    return true;
}

/* A reference within the allowed interval lets us test the nearest 2*pi
   equivalent and, if outside the interval, its adjacent equivalent. */
static bool Candidate(float angle, float reference, SwerveSteeringLimit limit, float *out)
{
    float value = reference + remainderf(angle - reference, SWERVE_TAU);
    if (!isfinite(value)) return false;
    if (value < limit.min_rad) value += SWERVE_TAU;
    else if (value > limit.max_rad) value -= SWERVE_TAU;
    if (!isfinite(value) || value < limit.min_rad || value > limit.max_rad) return false;
    *out = value;
    return true;
}

/* 在舵角限制内选转动距离较短的等效方案；必要时把轮向转 180°并反转轮速。 */
bool Swerve_Optimize(const SwerveTargets *in,
                     const SwerveSteeringLimit limits[SWERVE_WHEELS],
                     const float reference[SWERVE_WHEELS], SwerveTargets *out)
{
    if (!in || !limits || !reference || !out) return false;
    SwerveTargets result;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
        SwerveSteeringLimit l = limits[i];
        SwerveWheel w = in->wheel[i];
        if (!isfinite(l.min_rad) || !isfinite(l.max_rad) || l.min_rad >= l.max_rad ||
            l.max_rad - l.min_rad > SWERVE_TAU || !isfinite(reference[i]) ||
            reference[i] < l.min_rad || reference[i] > l.max_rad ||
            !isfinite(w.speed_rad_s) || !isfinite(w.angle_rad)) return false;
        result.wheel[i] = (SwerveWheel){0.0f, reference[i]};
        if (w.speed_rad_s == 0.0f) continue;

        float direct = 0.0f, reverse = 0.0f;
        bool have_direct = Candidate(w.angle_rad, reference[i], l, &direct);
        bool have_reverse = Candidate(w.angle_rad + SWERVE_PI, reference[i], l, &reverse);
        if (!have_direct && !have_reverse) return false;
        /* On a tie preserve the input drive direction. */
        bool flip = have_reverse && (!have_direct ||
                    fabsf(reverse - reference[i]) < fabsf(direct - reference[i]));
        result.wheel[i] = (SwerveWheel){flip ? -w.speed_rad_s : w.speed_rad_s,
                                                     flip ? reverse : direct};
    }
    *out = result;
    return true;
}

/* 任意轮超速时，四轮按同一比例降速，保留速度比例，转角不变。 */
bool Swerve_LimitSpeeds(const SwerveTargets *in, const float max_rad_s[SWERVE_WHEELS],
                        SwerveTargets *out, float *scale)
{
    if (!in || !max_rad_s || !out) return false;
    float factor = 1.0f;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
        if (!isfinite(max_rad_s[i]) || max_rad_s[i] <= 0.0f ||
            !isfinite(in->wheel[i].speed_rad_s) || !isfinite(in->wheel[i].angle_rad)) return false;
        float magnitude = fabsf(in->wheel[i].speed_rad_s);
        if (magnitude > max_rad_s[i]) factor = fminf(factor, max_rad_s[i] / magnitude);
    }
    SwerveTargets result = *in;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) result.wheel[i].speed_rad_s *= factor;
    *out = result;
    if (scale) *scale = factor;
    return true;
}
