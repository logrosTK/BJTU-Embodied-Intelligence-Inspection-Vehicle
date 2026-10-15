#include "chassis_planner.h"
#include <math.h>
#include <stddef.h>

/* 建立规划初值：驱动速度为零，舵角使用明确给出的参考值；不操作硬件。 */
bool ChassisPlanner_Init(ChassisPlanner *p, const ChassisPlannerConfig *config,
                         const float reference[SWERVE_WHEELS], uint32_t now)
{
    if (!p || !config || !reference || !config->max_step_ms ||
        !isfinite(config->wheel_hold_rad_s) || config->wheel_hold_rad_s < 0.0f) return false;
    const SwerveTwist zero = {0}; SwerveTargets wheels, checked;
    if (!Swerve_Inverse(&config->geometry, &zero, reference, &wheels) ||
        !Swerve_Optimize(&wheels, config->steering, reference, &checked)) return false;
    ChassisPlanner result = {
        .config = *config,
        .plan = {.wheels = checked, .timestamp_ms = now, .saturation_scale = 1.0f,
                 .command_timeout = true},
        .initialized = true
    };
    *p = result;
    return true;
}

/* 推进一拍连续运动规划：平滑车速、计算轮向、限制转向，再平滑轮速，无停车等待阶段。
   夹角降速依据转向指令，不是实测舵角；指令超时则减速并保持转角，新指令自动恢复。 */
bool ChassisPlanner_Step(ChassisPlanner *p, const MotionLimits *limits,
                         const SwerveTwist *request, bool valid, uint32_t now)
{
    if (!p || !p->initialized || !MotionLimits_Valid(limits) ||
        (valid && (!request || !isfinite(request->vx_m_s) ||
                   !isfinite(request->vy_m_s) || !isfinite(request->wz_rad_s)))) return false;
    uint32_t elapsed = now - p->plan.timestamp_ms;
    if (!elapsed) return true;
    if (elapsed > p->config.max_step_ms) elapsed = p->config.max_step_ms;
    float dt = elapsed * 0.001f;
    ChassisPlan result = p->plan;
    const SwerveTwist zero = {0};
    if (!MotionLimits_Twist(limits, valid ? request : &zero, &p->plan.body_reference,
                            dt, &result.body_reference)) return false;
    SwerveTargets goal = p->plan.wheels;
    result.saturation_scale = 1.0f;
    if (valid) {
        float reference[SWERVE_WHEELS];
        for (unsigned i = 0; i < SWERVE_WHEELS; ++i) reference[i] = p->plan.wheels.wheel[i].angle_rad;
        SwerveTargets raw;
        if (!Swerve_Inverse(&p->config.geometry, &result.body_reference, reference, &raw)) return false;
        for (unsigned i = 0; i < SWERVE_WHEELS; ++i)
            if (raw.wheel[i].speed_rad_s < p->config.wheel_hold_rad_s) raw.wheel[i].speed_rad_s = 0.0f;
        if (!Swerve_Optimize(&raw, p->config.steering, reference, &goal) ||
            !Swerve_LimitSpeeds(&goal, limits->wheel_speed_rad_s, &goal, &result.saturation_scale)) return false;

        /* First calculate the bounded next steering COMMAND. Drive results of
           this pass are discarded. No measurement or servo-arrival assertion. */
        SwerveTargets staged;
        if (!MotionLimits_Wheels(limits, &goal, &p->plan.wheels, dt, &staged)) return false;
        for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
            float error = goal.wheel[i].angle_rad - staged.wheel[i].angle_rad;
            goal.wheel[i].speed_rad_s *= fmaxf(0.0f, cosf(error));
            goal.wheel[i].angle_rad = staged.wheel[i].angle_rad;
        }
    } else {
        /* Hold the last command angle throughout timeout braking. */
        for (unsigned i = 0; i < SWERVE_WHEELS; ++i) goal.wheel[i].speed_rad_s = 0.0f;
    }
    /* Apply signed drive slew AFTER alignment reduction, so a direction
       change cannot cause a discontinuous speed cut or immediate reversal. */
    if (!MotionLimits_Wheels(limits, &goal, &p->plan.wheels, dt, &result.wheels)) return false;
    result.timestamp_ms = now; result.command_timeout = !valid;
    p->plan = result;
    return true;
}
