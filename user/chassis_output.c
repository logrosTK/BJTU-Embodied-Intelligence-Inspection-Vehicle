#include "chassis_output.h"
#include "inspection_config.h"
#include <float.h>
#include <math.h>

_Static_assert(SWERVE_WHEELS == INSPECTION_WHEELS && SWERVE_WHEELS == SERVO_PWM_WHEELS,
               "Wheel count mismatch");

static bool Positive(float x) { return isfinite(x) && x > 0.0f; }
static bool Within(float value, float maximum)
{ return value <= maximum || value - maximum <= 4.0f * FLT_EPSILON * maximum; }

/* 将规划结果转换为四轮速度 CAN 帧和舵机脉宽，仅生成数据，不发送或使能。
   四轮检查全部通过才更新 out；失败时不能把旧 out 当作新指令继续使用。 */
bool ChassisOutput_Build(const ChassisOutputConfig *c, const ChassisOutputInput *in,
                         ChassisOutputBatch *out, ChassisOutputDiagnostics *diagnostics)
{
    ChassisOutputDiagnostics d = {0};
    if (!c || !in || !out || !in->plan || !in->motors || !in->limits ||
        !c->plan_max_age_ms || !c->device_max_age_ms) {
        d.global_errors = CHASSIS_OUTPUT_ARGUMENT;
        if (diagnostics) *diagnostics = d;
        return false;
    }
    ChassisOutputBatch result = {0};
    const ChassisPlan *p = in->plan;
    if (in->bus_off) d.global_errors |= CHASSIS_OUTPUT_BUS_OFF;
    if ((uint32_t)(in->now_ms - p->timestamp_ms) >= c->plan_max_age_ms)
        d.global_errors |= CHASSIS_OUTPUT_PLAN_AGE;
    if (!isfinite(p->body_reference.vx_m_s) || !isfinite(p->body_reference.vy_m_s) ||
        !isfinite(p->body_reference.wz_rad_s) || !isfinite(p->saturation_scale) ||
        p->saturation_scale < 0.0f || p->saturation_scale > 1.0f)
        d.global_errors |= CHASSIS_OUTPUT_PLAN_NUMBER;
    bool limits_valid = MotionLimits_Valid(in->limits);
    if (!limits_valid) d.global_errors |= CHASSIS_OUTPUT_LIMITS;

    bool failed = d.global_errors != 0;
    for (unsigned i = 0; i < SWERVE_WHEELS; ++i) {
        const Dm3519 *m = &in->motors[i];
        SwerveWheel wheel = p->wheels.wheel[i];
        uint16_t e = 0;
        if (c->drive_sign[i] != 1 && c->drive_sign[i] != -1) e |= CHASSIS_WHEEL_DIRECTION;
        if (!Positive(c->external_ratio[i])) e |= CHASSIS_WHEEL_RATIO;
        if (!ServoPwm_ValidCalibration(&c->steering[i])) e |= CHASSIS_WHEEL_CALIBRATION;
        if (m->can_id != i + 1U || m->master_id != INSPECTION_MASTER_BASE + i) e |= CHASSIS_WHEEL_ADDRESS;
        if (m->parameters != 31U || m->control_mode != 3U || !Positive(m->p_max) ||
            !Positive(m->v_max) || !Positive(m->t_max) || !Positive(m->max_speed)) e |= CHASSIS_WHEEL_PARAMETERS;
        if (!m->feedback_received || (uint32_t)(in->now_ms - m->feedback_ms) >= c->device_max_age_ms ||
            !isfinite(m->velocity_output_rad_s)) e |= CHASSIS_WHEEL_FEEDBACK;
        if (m->state > 1U) e |= CHASSIS_WHEEL_FAULT;
        if (!isfinite(wheel.speed_rad_s) ||
            (limits_valid && !Within(fabsf(wheel.speed_rad_s), in->limits->wheel_speed_rad_s[i])))
            e |= CHASSIS_WHEEL_SPEED;
        if (!isfinite(wheel.angle_rad) || (!(e & CHASSIS_WHEEL_CALIBRATION) &&
            !ServoPwm_Encode(&c->steering[i], wheel.angle_rad, &result.steering_us[i])))
            e |= CHASSIS_WHEEL_STEERING;

        if (!e && limits_valid) {
            float output_speed = wheel.speed_rad_s * c->external_ratio[i] * c->drive_sign[i];
            if (!Dm3519_VelocityFrame(m, output_speed, &result.drive[i])) e |= CHASSIS_WHEEL_ENCODING;
        }
        result.steering_rad[i] = wheel.angle_rad;
        d.wheel_errors[i] = e; failed = failed || e != 0;
    }
    if (diagnostics) *diagnostics = d;
    if (failed) return false;
    result.planned_ms = p->timestamp_ms; result.built_ms = in->now_ms;
    result.command_timeout = p->command_timeout;
    *out = result;
    return true;
}
