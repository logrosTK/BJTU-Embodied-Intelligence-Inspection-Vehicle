#include "motion_limits.h"
#include "servo_pwm.h"
#include "inspection_app.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846f
static void Near(float a, float b) { assert(fabsf(a-b) < 0.00002f); }

TIM_TypeDef test_tim3;
static unsigned pwm_active, pwm_starts, pwm_stops, fail_start;

void Test_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
    assert(timer->Instance->CR1 & TIM_CR1_UDIS); /* No half-updated PWM frame. */
    assert(channel <= TIM_CHANNEL_4);
    timer->Instance->compare[channel/4U] = value;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    ++pwm_starts;
    assert(timer->Instance->EGR == TIM_EGR_UG);
    for (unsigned i=0;i<4;++i) assert(timer->Instance->compare[i] == 1500);
    if (pwm_starts == fail_start) return HAL_ERROR;
    pwm_active |= 1U << (channel/4U);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer; ++pwm_stops; pwm_active &= ~(1U << (channel/4U)); return HAL_OK;
}

static void TestServo(void)
{
    /* Deliberately asymmetric calibration anchors to exercise both halves. */
    ServoPwmCalibration c = {-2,2.5f,600,1500,2400}; uint16_t pulse = 99;
    assert(ServoPwm_ValidCalibration(&c));
    assert(ServoPwm_Encode(&c,0,&pulse) && pulse == 1500);
    assert(ServoPwm_Encode(&c,-1,&pulse) && pulse == 1050);
    assert(ServoPwm_Encode(&c,1.25f,&pulse) && pulse == 1950);
    assert(ServoPwm_Encode(&c,2.5f,&pulse) && pulse == 2400);
    c.negative_us=2400; c.positive_us=600;
    assert(ServoPwm_Encode(&c,-1,&pulse) && pulse == 1950);
    assert(ServoPwm_Encode(&c,1.25f,&pulse) && pulse == 1050);
    assert(!ServoPwm_Encode(&c,NAN,&pulse) && pulse == 1050);
    assert(!ServoPwm_Encode(&c,-2.1f,&pulse) && pulse == 1050);
    c.negative_us=400; assert(!ServoPwm_ValidCalibration(&c));
    c.negative_us=2000; c.positive_us=2100; assert(!ServoPwm_ValidCalibration(&c));

    ServoPwmCalibration all[4];
    for (unsigned i=0;i<4;++i) assert(Inspection_SteeringCalibration(i&1 ? -1 : 1,&all[i]));
    assert(!Inspection_SteeringCalibration(0,&c));
    assert(ServoPwm_Encode(&all[0],0,&pulse) && pulse == 1500);
    assert(ServoPwm_Encode(&all[0],all[0].positive_rad,&pulse) && pulse == 2389);
    assert(ServoPwm_Encode(&all[1],all[1].positive_rad,&pulse) && pulse == 611);
    ServoPwm4 p={0}; TIM_HandleTypeDef timer={.Instance=TIM3,.Init={83,19999}};
    assert(!ServoPwm4_Start(&p));
    timer.Init.Prescaler=84; assert(!ServoPwm4_Configure(&p,&timer,all));
    timer.Init.Prescaler=83;
    assert(ServoPwm4_Configure(&p,&timer,all));
    assert(pwm_starts==0 && pwm_active==0); /* Configure cannot move the robot. */
    assert(ServoPwm4_Start(&p) && pwm_active==15);
    assert(!ServoPwm4_Start(&p) && !ServoPwm4_Configure(&p,&timer,all));
    float angles[4]={0.4f,-0.4f,0.8f,-0.8f};
    assert(ServoPwm4_SetAngles(&p,angles));
    assert(test_tim3.compare[0]==test_tim3.compare[1]);
    assert(test_tim3.compare[2]==test_tim3.compare[3]);
    assert(!(test_tim3.CR1 & TIM_CR1_UDIS));
    uint32_t before[4]; memcpy(before,test_tim3.compare,sizeof(before));
    angles[3]=PI;
    assert(!ServoPwm4_SetAngles(&p,angles)); assert(memcmp(before,test_tim3.compare,sizeof(before))==0);
    angles[3]=-0.8f;
    test_tim3.CR1 |= TIM_CR1_UDIS;
    assert(ServoPwm4_SetAngles(&p,angles)); assert(test_tim3.CR1 & TIM_CR1_UDIS);
    test_tim3.CR1 &= ~TIM_CR1_UDIS;
    assert(ServoPwm4_Stop(&p) && !p.running && pwm_active==0);
    for (unsigned i=0;i<4;++i) assert(test_tim3.compare[i]==0);
    assert(!ServoPwm4_SetAngles(&p,angles));
    fail_start=pwm_starts+3;
    assert(!ServoPwm4_Start(&p)); assert(!p.running && pwm_active==0 && pwm_stops>=8);
    fail_start=0;
    puts("PASS: servo mapping/direction/limits, PWM opt-in, coherent CCR updates, invalid-frame atomicity, startup rollback");
}

static MotionLimits Limits(void)
{
    MotionLimits l={.linear_speed_m_s=2,.yaw_speed_rad_s=1,
        .linear_accel_m_s2=0.5f,.yaw_accel_rad_s2=1};
    for (unsigned i=0;i<4;++i) {
        l.wheel_speed_rad_s[i]=32; l.wheel_accel_rad_s2[i]=8; l.steering_speed_rad_s[i]=PI;
    }
    return l;
}

static void TestLimits(void)
{
    MotionLimits l=Limits();
    SwerveTwist desired={3,4,10}, previous={0}, out;
    assert(MotionLimits_Twist(&l,&desired,&previous,0.02f,&out));
    Near(out.vx_m_s,0.006f); Near(out.vy_m_s,0.008f); Near(out.wz_rad_s,0.02f);
    for (unsigned n=0;n<500;++n) {
        SwerveTwist next;
        assert(MotionLimits_Twist(&l,&desired,&out,0.02f,&next));
        assert(hypotf(next.vx_m_s-out.vx_m_s,next.vy_m_s-out.vy_m_s)<=0.010001f);
        assert(fabsf(next.wz_rad_s-out.wz_rad_s)<=0.020001f);
        out=next;
    }
    Near(out.vx_m_s,1.2f); Near(out.vy_m_s,1.6f); Near(out.wz_rad_s,1);
    desired=(SwerveTwist){-3,-4,-10};
    assert(MotionLimits_Twist(&l,&desired,&out,0.02f,&out));
    Near(out.vx_m_s,1.194f); Near(out.vy_m_s,1.592f); Near(out.wz_rad_s,0.98f);
    SwerveTwist saved=out;
    assert(!MotionLimits_Twist(&l,&desired,&out,0,&out));
    assert(!MotionLimits_Twist(&l,&desired,&out,NAN,&out));
    assert(memcmp(&out,&saved,sizeof(out))==0);
    /* Saturated outputs must remain valid as the next call's previous value,
       including float rounding around the circular linear-speed boundary. */
    for (unsigned n=0;n<360;++n) {
        float angle=n*PI/180;
        SwerveTwist large={3*cosf(angle),3*sinf(angle),0}, rest={0}, saturated;
        assert(MotionLimits_Twist(&l,&large,&rest,10,&saturated));
        assert(MotionLimits_Twist(&l,&large,&saturated,0.01f,&saturated));
        assert(hypotf(saturated.vx_m_s,saturated.vy_m_s)<=2.000001f);
    }

    SwerveTargets goal={.wheel={{100,2},{-50,-2},{10,1},{-10,-1}}};
    SwerveTargets old={0}, limited;
    assert(MotionLimits_Wheels(&l,&goal,&old,0.02f,&limited));
    for (unsigned i=0;i<4;++i) {
        Near(fabsf(limited.wheel[i].speed_rad_s),0.16f);
        Near(fabsf(limited.wheel[i].angle_rad),PI*0.02f);
    }
    /* Reverse drive commands traverse zero; mechanical angles do NOT wrap. */
    goal.wheel[0]=(SwerveWheel){-20,-2}; old.wheel[0]=(SwerveWheel){20,2};
    assert(MotionLimits_Wheels(&l,&goal,&old,0.02f,&old));
    Near(old.wheel[0].speed_rad_s,19.84f); Near(old.wheel[0].angle_rad,2-PI*0.02f);
    SwerveTargets before=old;
    goal.wheel[3].angle_rad=NAN;
    assert(!MotionLimits_Wheels(&l,&goal,&old,0.02f,&old)); assert(memcmp(&old,&before,sizeof(old))==0);
    l.linear_accel_m_s2=0; assert(!MotionLimits_Valid(&l));
    l=Limits(); l.steering_speed_rad_s[2]=INFINITY; assert(!MotionLimits_Valid(&l));

    SwerveTargets measured_goal={.wheel={{10,0},{10,0},{-10,0},{10,0}}};
    float measured[4]={0,PI/3,PI/2,PI};
    assert(MotionLimits_CompensateMeasured(&measured_goal,measured,15,&limited));
    Near(limited.wheel[0].speed_rad_s,10); Near(limited.wheel[1].speed_rad_s,5);
    Near(limited.wheel[2].speed_rad_s,0); Near(limited.wheel[3].speed_rad_s,0);
    before=limited;
    assert(!MotionLimits_CompensateMeasured(&measured_goal,measured,7,&limited));
    measured[0]=NAN; assert(!MotionLimits_CompensateMeasured(&measured_goal,measured,15,&limited));
    assert(memcmp(&limited,&before,sizeof(limited))==0);
    puts("PASS: vector speed/accel limits, yaw limits, signed drive ramps, continuous steering ramps, measured-angle compensation validity");
}

void TestMotion(void) { TestServo(); TestLimits(); }
