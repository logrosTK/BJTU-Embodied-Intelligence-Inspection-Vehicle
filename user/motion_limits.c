#include "motion_limits.h"
#include <float.h>
#include <math.h>

static bool Positive(float x) { return isfinite(x) && x > 0.0f; }
static bool TwistFinite(const SwerveTwist *t)
{ return t && isfinite(t->vx_m_s) && isfinite(t->vy_m_s) && isfinite(t->wz_rad_s); }
/* Accept only roundoff-scale excess when a previous output is reused. */
static bool Within(float value, float maximum)
{ return value <= maximum || value - maximum <= 4.0f * FLT_EPSILON * maximum; }

/* 检查速度、加速度和转向速度限制是否都是有效正数。 */
bool MotionLimits_Valid(const MotionLimits *l)
{
    if (!l || !Positive(l->linear_speed_m_s) || !Positive(l->yaw_speed_rad_s) ||
        !Positive(l->linear_accel_m_s2) || !Positive(l->yaw_accel_rad_s2)) return false;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i)
        if (!Positive(l->wheel_speed_rad_s[i]) || !Positive(l->wheel_accel_rad_s2[i]) ||
            !Positive(l->steering_speed_rad_s[i])) return false;
    return true;
}

static float Clamp(float x, float low, float high) { return fmaxf(low, fminf(x, high)); }
static float Ramp(float desired, float previous, float step)
{
    float delta = desired - previous;
    if (delta > step) return previous + step;
    if (delta < -step) return previous - step;
    return desired;
}

/* 限制车体平移和旋转速度，再限制每步变化，避免速度指令突变；dt 单位为秒。 */
bool MotionLimits_Twist(const MotionLimits *l, const SwerveTwist *desired,
                        const SwerveTwist *previous, float dt, SwerveTwist *out)
{
    if (!out || !MotionLimits_Valid(l) || !TwistFinite(desired) ||
        !TwistFinite(previous) || !Positive(dt)) return false;
    float speed = hypotf(desired->vx_m_s, desired->vy_m_s);
    float old_speed = hypotf(previous->vx_m_s, previous->vy_m_s);
    if (!isfinite(speed) || !isfinite(old_speed) || !Within(old_speed, l->linear_speed_m_s) ||
        fabsf(previous->wz_rad_s) > l->yaw_speed_rad_s) return false;
    SwerveTwist goal = *desired;
    if (speed > l->linear_speed_m_s) {
        float scale = l->linear_speed_m_s / speed;
        goal.vx_m_s *= scale; goal.vy_m_s *= scale;
    }
    goal.wz_rad_s = Clamp(goal.wz_rad_s, -l->yaw_speed_rad_s, l->yaw_speed_rad_s);
    float dx = goal.vx_m_s - previous->vx_m_s, dy = goal.vy_m_s - previous->vy_m_s;
    float delta = hypotf(dx, dy), linear_step = l->linear_accel_m_s2 * dt;
    float yaw_step = l->yaw_accel_rad_s2 * dt;
    if (!isfinite(delta) || !isfinite(linear_step) || !isfinite(yaw_step)) return false;
    if (delta > linear_step) {
        float scale = linear_step / delta;
        goal.vx_m_s = previous->vx_m_s + dx * scale;
        goal.vy_m_s = previous->vy_m_s + dy * scale;
    }
    goal.wz_rad_s = Ramp(goal.wz_rad_s, previous->wz_rad_s, yaw_step);
    if (!TwistFinite(&goal)) return false;
    *out = goal;
    return true;
}

/* 限制各轮轮速、加减速和转角变化；previous 是上一拍指令，不是传感器测量值。 */
bool MotionLimits_Wheels(const MotionLimits *l, const SwerveTargets *desired,
                         const SwerveTargets *previous, float dt, SwerveTargets *out)
{
    if (!out || !previous || !MotionLimits_Valid(l) || !Positive(dt)) return false;
    SwerveTargets result;
    if (!Swerve_LimitSpeeds(desired, l->wheel_speed_rad_s, &result, 0)) return false;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
        SwerveWheel p = previous->wheel[i];
        float drive_step = l->wheel_accel_rad_s2[i] * dt;
        float steer_step = l->steering_speed_rad_s[i] * dt;
        if (!isfinite(p.speed_rad_s) || !isfinite(p.angle_rad) ||
            !Within(fabsf(p.speed_rad_s), l->wheel_speed_rad_s[i]) ||
            !isfinite(drive_step) || !isfinite(steer_step)) return false;
        result.wheel[i].speed_rad_s = Ramp(result.wheel[i].speed_rad_s, p.speed_rad_s, drive_step);
        result.wheel[i].angle_rad = Ramp(result.wheel[i].angle_rad, p.angle_rad, steer_step);
        if (!isfinite(result.wheel[i].speed_rad_s) || !isfinite(result.wheel[i].angle_rad)) return false;
    }
    *out = result;
    return true;
}

/* 按目标轮向与实测舵角的夹角降低轮速；必须有四轮有效实测角度，当前 PWM 舵机不调用。 */
bool MotionLimits_CompensateMeasured(const SwerveTargets *desired, const float measured[SWERVE_WHEELS],
                                    unsigned valid_mask, SwerveTargets *out)
{
    if (!desired || !measured || !out || valid_mask != 0x0FU) return false;
    SwerveTargets result = *desired;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
        if (!isfinite(measured[i]) || !isfinite(desired->wheel[i].angle_rad) ||
            !isfinite(desired->wheel[i].speed_rad_s)) return false;
        float error = desired->wheel[i].angle_rad - measured[i];
        if (!isfinite(error)) return false;
        result.wheel[i].speed_rad_s *= fmaxf(0.0f, cosf(error));
    }
    *out = result;
    return true;
}
