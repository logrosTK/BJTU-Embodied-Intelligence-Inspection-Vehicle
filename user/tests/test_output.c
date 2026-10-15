#include "chassis_output.h"
#include "byte_codec.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    ChassisOutputConfig config;
    ChassisPlan plan;
    Dm3519 motors[4];
    MotionLimits limits;
    ChassisOutputInput input;
} Fixture;

static void Init(Fixture *f)
{
    memset(f,0,sizeof(*f));
    f->config.plan_max_age_ms=20; f->config.device_max_age_ms=100;
    f->plan.timestamp_ms=100; f->plan.saturation_scale=1;
    f->limits=(MotionLimits){.linear_speed_m_s=1.5f,.yaw_speed_rad_s=1,
        .linear_accel_m_s2=0.5f,.yaw_accel_rad_s2=1};
    for (unsigned i=0;i<4;++i) {
        f->config.drive_sign[i]=1; f->config.external_ratio[i]=1;
        f->config.steering[i]=(ServoPwmCalibration){-2,2,600,1500,2400};
        f->motors[i]=(Dm3519){.can_id=(uint16_t)(i+1),.master_id=(uint16_t)(0x11+i),
            .p_max=12.5f,.v_max=40,.t_max=8,.max_speed=35,.parameters=31,.control_mode=3,
            .feedback_ms=100,.feedback_received=true,.state=0};
        f->limits.wheel_speed_rad_s[i]=24.5f;
        f->limits.wheel_accel_rad_s2[i]=8; f->limits.steering_speed_rad_s[i]=3.14159265359f;
    }
    f->input=(ChassisOutputInput){.plan=&f->plan,.motors=f->motors,.limits=&f->limits,.now_ms=110};
}

static ChassisOutputDiagnostics Reject(Fixture *f)
{
    ChassisOutputBatch out,before;
    memset(&out,0xA5,sizeof(out)); before=out;
    ChassisOutputDiagnostics d;
    assert(!ChassisOutput_Build(&f->config,&f->input,&out,&d));
    assert(memcmp(&out,&before,sizeof(out))==0); /* Never leak a partial batch. */
    return d;
}

static void TestMapping(void)
{
    Fixture f; Init(&f);
    f.plan.wheels=(SwerveTargets){.wheel={{6.283f,0},{-2,1},{3,-1},{-4,2}}};
    f.config.drive_sign[1]=-1; f.config.drive_sign[3]=-1;
    f.config.external_ratio[2]=2; f.config.external_ratio[3]=0.5f;
    f.config.steering[1]=(ServoPwmCalibration){-2,2,2400,1500,600};
    ChassisOutputBatch out; ChassisOutputDiagnostics d;
    assert(ChassisOutput_Build(&f.config,&f.input,&out,&d));
    assert(d.global_errors==0);
    const float expected[4]={6.283f,2,6,2};
    const uint16_t pulses[4]={1500,1050,1050,2400};
    const uint8_t golden[4]={0x56,0x0E,0xC9,0x40};
    for (unsigned i=0;i<4;++i) {
        assert(d.wheel_errors[i]==0 && out.drive[i].id==0x201+i && out.drive[i].length==4);
        assert(fabsf(ReadF32LE(out.drive[i].data)-expected[i])<0.00001f);
        assert(out.steering_us[i]==pulses[i]);
        assert(out.steering_rad[i]==f.plan.wheels.wheel[i].angle_rad);
    }
    assert(memcmp(out.drive[0].data,golden,4)==0); /* No internal /19.2. */
    assert(out.planned_ms==100 && out.built_ms==110 && !out.command_timeout);
    /* Disabled devices may be PREPARED; no motor-enable packet is generated. */
    f.plan.command_timeout=true;
    assert(ChassisOutput_Build(&f.config,&f.input,&out,NULL) && out.command_timeout);
}

static void TestRejects(void)
{
    Fixture f; ChassisOutputDiagnostics d;
    Init(&f); f.config.drive_sign[3]=0;
    assert(Reject(&f).wheel_errors[3]&CHASSIS_WHEEL_DIRECTION);
    Init(&f); f.config.external_ratio[1]=NAN;
    assert(Reject(&f).wheel_errors[1]&CHASSIS_WHEEL_RATIO);
    Init(&f); f.config.steering[2].center_us=500;
    assert(Reject(&f).wheel_errors[2]&CHASSIS_WHEEL_CALIBRATION);
    Init(&f); f.motors[2].can_id=2;
    assert(Reject(&f).wheel_errors[2]&CHASSIS_WHEEL_ADDRESS);
    Init(&f); f.motors[0].master_id=0x12;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_ADDRESS);
    Init(&f); f.motors[0].parameters=7;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_PARAMETERS);
    Init(&f); f.motors[0].max_speed=NAN;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_PARAMETERS);
    Init(&f); f.motors[0].control_mode=1;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_PARAMETERS);
    Init(&f); f.motors[1].feedback_received=false;
    assert(Reject(&f).wheel_errors[1]&CHASSIS_WHEEL_FEEDBACK);
    Init(&f); f.motors[1].state=8;
    assert(Reject(&f).wheel_errors[1]&CHASSIS_WHEEL_FAULT);
    Init(&f); f.plan.wheels.wheel[0].speed_rad_s=25;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_SPEED);
    Init(&f); f.plan.wheels.wheel[0].speed_rad_s=NAN;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_SPEED);
    Init(&f); f.plan.wheels.wheel[2].angle_rad=2.1f;
    assert(Reject(&f).wheel_errors[2]&CHASSIS_WHEEL_STEERING);
    Init(&f); f.config.external_ratio[0]=10; f.plan.wheels.wheel[0].speed_rad_s=4;
    assert(Reject(&f).wheel_errors[0]&CHASSIS_WHEEL_ENCODING);
    Init(&f); f.input.bus_off=true;
    assert(Reject(&f).global_errors&CHASSIS_OUTPUT_BUS_OFF);
    Init(&f); f.plan.body_reference.vx_m_s=NAN;
    assert(Reject(&f).global_errors&CHASSIS_OUTPUT_PLAN_NUMBER);
    Init(&f); f.limits.wheel_speed_rad_s[0]=0;
    assert(Reject(&f).global_errors&CHASSIS_OUTPUT_LIMITS);
    Init(&f); f.input.plan=NULL;
    assert(Reject(&f).global_errors&CHASSIS_OUTPUT_ARGUMENT);
    /* Report all four failures in one pass, with no usable batch. */
    Init(&f);
    for (unsigned i=0;i<4;++i) f.config.drive_sign[i]=0;
    d=Reject(&f);
    for (unsigned i=0;i<4;++i) assert(d.wheel_errors[i]&CHASSIS_WHEEL_DIRECTION);
}

static void TestAges(void)
{
    Fixture f; ChassisOutputBatch out;
    Init(&f); f.input.now_ms=119;
    assert(ChassisOutput_Build(&f.config,&f.input,&out,NULL));
    f.input.now_ms=120; assert(Reject(&f).global_errors&CHASSIS_OUTPUT_PLAN_AGE);
    Init(&f); f.input.now_ms=200; f.plan.timestamp_ms=200;
    for (unsigned i=0;i<4;++i) assert(Reject(&f).wheel_errors[i]&CHASSIS_WHEEL_FEEDBACK);
    Init(&f); f.input.now_ms=4; f.plan.timestamp_ms=UINT32_MAX-5;
    for (unsigned i=0;i<4;++i) f.motors[i].feedback_ms=UINT32_MAX-5;
    assert(ChassisOutput_Build(&f.config,&f.input,&out,NULL));
    f.plan.timestamp_ms=5; /* Future timestamp is rejected too. */
    assert(Reject(&f).global_errors&CHASSIS_OUTPUT_PLAN_AGE);
}

void TestOutput(void)
{
    TestMapping(); TestRejects(); TestAges();
    puts("PASS: four-wheel CAN/PWM output mapping, signs/external ratio, all-or-nothing validation, ages/wrap, detailed rejections");
}
