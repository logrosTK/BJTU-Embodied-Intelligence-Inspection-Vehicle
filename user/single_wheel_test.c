#include "single_wheel_test.h"
#include <math.h>

static bool Positive(float x) { return isfinite(x) && x > 0; }
static float Ramp(float target, float previous, float step)
{
    float d=target-previous;
    if (d>step) return previous+step;
    if (d<-step) return previous-step;
    return target;
}
/* 建立单轮测试状态，保留传入的 PWM 参考值并将驱动目标置零；不输出到硬件。 */
bool SingleWheelTest_Init(SingleWheelTest *t, const SingleWheelTestConfig *c,
                          const uint16_t reference[4], uint32_t now)
{
    if (!t || !c || !reference || c->min_pulse_us<500 || c->max_pulse_us>2500 ||
        c->min_pulse_us>=c->max_pulse_us || !Positive(c->max_drive_rad_s) ||
        !Positive(c->drive_accel_rad_s2) || !Positive(c->pulse_rate_us_s) ||
        !c->lease_ms || c->lease_ms>=0x80000000UL || !c->max_step_ms) return false;
    SingleWheelTest result={.timestamp_ms=now,.expired=true,.active=true,.config=*c};
    for (unsigned i=0;i<4;++i) {
        if (reference[i]<c->min_pulse_us || reference[i]>c->max_pulse_us) return false;
        result.pulse_us[i]=reference[i];
    }
    *t=result; return true;
}

/* 接受一轮的脉宽或原始输出轴速度目标；切换轮子、类型或测试舵机前必须确认实际停转。
   received 是指令接收时间，非法目标不会延长测试有效期。 */
bool SingleWheelTest_Target(SingleWheelTest *t, unsigned wheel, WheelTestKind kind,
                            float target, bool stopped, uint32_t received)
{
    if (!t || !t->active || wheel>=4 || (kind!=WHEEL_TEST_PULSE && kind!=WHEEL_TEST_DRIVE) ||
        !isfinite(target)) return false;
    bool switching=t->received && (wheel!=t->wheel || kind!=t->kind);
    if ((switching || kind==WHEEL_TEST_PULSE || !t->received) && !stopped) return false;
    if (switching) for (unsigned i=0;i<4;++i) if (t->drive_rad_s[i]!=0) return false;
    if (kind==WHEEL_TEST_PULSE) {
        if (target<t->config.min_pulse_us || target>t->config.max_pulse_us || target!=roundf(target)) return false;
    } else if (fabsf(target)>t->config.max_drive_rad_s) return false;
    t->wheel=(uint8_t)wheel; t->kind=kind; t->target=target;
    t->received_ms=received; t->received=true;
    return true;
}

/* 平滑推进单轮测试；续期超时则把驱动目标降为零，并保持最后的舵机脉宽。 */
bool SingleWheelTest_Step(SingleWheelTest *t, uint32_t now)
{
    if (!t || !t->active) return false;
    uint32_t elapsed=now-t->timestamp_ms;
    if (elapsed>t->config.max_step_ms) elapsed=t->config.max_step_ms;
    float dt=elapsed*0.001f;
    bool expired=!t->received || (uint32_t)(now-t->received_ms)>=t->config.lease_ms;
    float drive_step=t->config.drive_accel_rad_s2*dt, pulse_step=t->config.pulse_rate_us_s*dt;
    if (!isfinite(drive_step) || !isfinite(pulse_step)) return false;
    for (unsigned i=0;i<4;++i) {
        float goal=(!expired && t->kind==WHEEL_TEST_DRIVE && i==t->wheel) ? t->target : 0;
        t->drive_rad_s[i]=Ramp(goal,t->drive_rad_s[i],drive_step);
        if (!expired && t->kind==WHEEL_TEST_PULSE && i==t->wheel)
            t->pulse_us[i]=Ramp(t->target,t->pulse_us[i],pulse_step);
    }
    t->expired=expired; t->timestamp_ms=now;
    return true;
}

/* 驱动指令为零且实测确认停转后才能结束测试；不会自动恢复导航。 */
bool SingleWheelTest_End(SingleWheelTest *t, bool stopped)
{
    if (!t || !t->active || !stopped) return false;
    for (unsigned i=0;i<4;++i) if (t->drive_rad_s[i]!=0) return false;
    t->active=false; return true;
}
