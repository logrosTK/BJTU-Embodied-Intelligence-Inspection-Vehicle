/* Offline replay of the SAME planner and host parser compiled into firmware.
   This simulates command processing, NOT tire/contact/servo/vehicle physics.
   No serial, CAN, PWM or ROS dependency. See simulate.ps1. */
#include "chassis_planner.h"
#include "chassis_output.h"
#include "host_protocol.h"
#include "inspection_config.h"
#include "byte_codec.h"
#include <math.h>
#include <stdio.h>

/* 按时间生成直行、横移、倒车、旋转、断连和恢复场景，供无硬件验证使用。 */
static SwerveTwist Scenario(uint32_t now, const char **name, bool *send)
{
    *send = true;
    if (now < 2000) {*name="straight"; return (SwerveTwist){0.6f,0,0};}
    if (now < 4000) {*name="sideways"; return (SwerveTwist){0,0.6f,0};}
    if (now < 6000) {*name="reverse"; return (SwerveTwist){-0.6f,0,0};}
    if (now < 8000) {*name="mixed"; return (SwerveTwist){0.3f,0.2f,0.5f};}
    if (now < 9000) {*name="yaw"; return (SwerveTwist){0,0,0.6f};}
    if (now < 10000) {*name="disconnect"; *send=false; return (SwerveTwist){0};}
    if (now < 12000) {*name="recovery"; return (SwerveTwist){0.4f,0,0};}
    *name="stop"; return (SwerveTwist){0};
}

/* 复用固件解析器和规划器离线回放，检查指令连续性并导出 CSV，不模拟真实车辆动力学。 */
int main(void)
{
    ChassisPlannerConfig config={
        .geometry={INSPECTION_WHEEL_RADIUS_M,INSPECTION_PIVOT_TRACK_M,INSPECTION_PIVOT_WHEELBASE_M},
        .max_step_ms=INSPECTION_CONTROL_MAX_STEP_MS, .wheel_hold_rad_s=INSPECTION_WHEEL_HOLD_RAD_S
    };
    const float reference[4]={0};
    /* Simulation fixture: driver MAX_SPD/VMAX are assumed above the manual's
       physical reference ONLY here. Real firmware reads them via CAN. */
    float wheel_limit=INSPECTION_MOTOR_NO_LOAD_RPM*(6.28318530718f/60)*INSPECTION_MOTOR_SPEED_FRACTION;
    MotionLimits limits={
        .linear_speed_m_s=wheel_limit*INSPECTION_WHEEL_RADIUS_M,
        .linear_accel_m_s2=INSPECTION_LINEAR_ACCEL_M_S2,
        .yaw_speed_rad_s=INSPECTION_YAW_SPEED_RAD_S,
        .yaw_accel_rad_s2=INSPECTION_YAW_ACCEL_RAD_S2
    };
    ChassisOutputConfig output_config={.plan_max_age_ms=INSPECTION_CONTROL_MAX_STEP_MS,
                                       .device_max_age_ms=INSPECTION_DEVICE_MAX_AGE_MS};
    Dm3519 mock_motors[4];
    for (unsigned i=0;i<4;++i) {
        config.steering[i]=(SwerveSteeringLimit){-INSPECTION_STEERING_LIMIT_RAD,INSPECTION_STEERING_LIMIT_RAD};
        limits.wheel_speed_rad_s[i]=wheel_limit;
        limits.wheel_accel_rad_s2[i]=INSPECTION_LINEAR_ACCEL_M_S2/INSPECTION_WHEEL_RADIUS_M;
        limits.steering_speed_rad_s[i]=INSPECTION_STEERING_SPEED_RAD_S;
        /* Explicit SIMULATION fixtures, not measured installed directions or
           real motor register values. They are never used by firmware init. */
        output_config.drive_sign[i]=1; output_config.external_ratio[i]=1;
        output_config.steering[i]=(ServoPwmCalibration){-INSPECTION_STEERING_LIMIT_RAD,
            INSPECTION_STEERING_LIMIT_RAD,611,1500,2389};
        mock_motors[i]=(Dm3519){.can_id=(uint16_t)(i+1),.master_id=(uint16_t)(INSPECTION_MASTER_BASE+i),
            .p_max=1,.t_max=1,.v_max=wheel_limit/INSPECTION_MOTOR_SPEED_FRACTION,
            .max_speed=wheel_limit/INSPECTION_MOTOR_SPEED_FRACTION,
            .parameters=31,.control_mode=3,.state=0,.feedback_received=true};
    }
    ChassisPlanner planner; HostParser host;
    Host_Init(&host);
    if (!ChassisPlanner_Init(&planner,&config,reference,0)) return 1;
    puts("ms,scenario,command_valid,command_age_ms,timeout,request_vx,request_vy,request_wz,body_reference_vx,body_reference_vy,body_reference_wz,fl_rad_s,fr_rad_s,rl_rad_s,rr_rad_s,fl_angle_rad,fr_angle_rad,rl_angle_rad,rr_angle_rad,saturation_scale,fl_pwm_us,fr_pwm_us,rl_pwm_us,rr_pwm_us,can201_bytes,can202_bytes,can203_bytes,can204_bytes");
    uint16_t sequence=0;
    float max_speed_step=0, max_angle_step=0;
    for (uint32_t now=10;now<=14000;now+=10) {
        const char *name; bool send;
        SwerveTwist input=Scenario(now,&name,&send);
        if (send) {
            uint8_t payload[16],frame[HOST_MAX_FRAME];
            WriteU32LE(payload,now); WriteF32LE(payload+4,input.vx_m_s);
            WriteF32LE(payload+8,input.vy_m_s); WriteF32LE(payload+12,input.wz_rad_s);
            size_t n=Host_Encode(HOST_MSG_MOTION,sequence++,payload,16,frame);
            if (now==2050) frame[n-1]^=1; /* A bad CRC must not refresh watchdog. */
            /* Split every command across two chunks, as a UART may do. */
            Host_Feed(&host,frame,7,now); Host_Feed(&host,frame+7,n-7,now);
        }
        HostMotion motion;
        bool valid=Host_GetMotion(&host,now,&motion);
        SwerveTwist request={motion.vx,motion.vy,motion.wz};
        ChassisPlan before=planner.plan;
        if (!ChassisPlanner_Step(&planner,&limits,&request,valid,now)) {
            fprintf(stderr,"Planner rejected tick %lu\n",(unsigned long)now); return 2;
        }
        const ChassisPlan *p=&planner.plan;
        for (unsigned i=0;i<4;++i) mock_motors[i].feedback_ms=now;
        ChassisOutputInput output_input={.plan=p,.motors=mock_motors,.limits=&limits,.now_ms=now};
        ChassisOutputBatch batch;
        if (!ChassisOutput_Build(&output_config,&output_input,&batch,NULL)) {
            fprintf(stderr,"Output mapping rejected tick %lu\n",(unsigned long)now); return 5;
        }
        for (unsigned i=0;i<4;++i) {
            float speed_step=fabsf(p->wheels.wheel[i].speed_rad_s-before.wheels.wheel[i].speed_rad_s);
            float angle_step=fabsf(p->wheels.wheel[i].angle_rad-before.wheels.wheel[i].angle_rad);
            if (!isfinite(speed_step) || !isfinite(angle_step) ||
                speed_step>limits.wheel_accel_rad_s2[i]*0.01f+0.00001f ||
                angle_step>limits.steering_speed_rad_s[i]*0.01f+0.00001f) {
                fprintf(stderr,"Continuity check failed at %lu\n",(unsigned long)now); return 4;
            }
            max_speed_step=fmaxf(max_speed_step,speed_step);
            max_angle_step=fmaxf(max_angle_step,angle_step);
        }
        printf("%lu,%s,%u,%lu,%u,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g",
               (unsigned long)now,name,valid?1U:0U,(unsigned long)(now-motion.received_ms),
               p->command_timeout?1U:0U,request.vx_m_s,request.vy_m_s,request.wz_rad_s,
               p->body_reference.vx_m_s,p->body_reference.vy_m_s,p->body_reference.wz_rad_s);
        for (unsigned i=0;i<4;++i) printf(",%.8g",p->wheels.wheel[i].speed_rad_s);
        for (unsigned i=0;i<4;++i) printf(",%.8g",p->wheels.wheel[i].angle_rad);
        printf(",%.8g",p->saturation_scale);
        for (unsigned i=0;i<4;++i) printf(",%u",(unsigned)batch.steering_us[i]);
        for (unsigned i=0;i<4;++i) {
            const uint8_t *b=batch.drive[i].data;
            printf(",0x%02X%02X%02X%02X",(unsigned)b[0],(unsigned)b[1],(unsigned)b[2],(unsigned)b[3]);
        }
        putchar('\n');
    }
    fprintf(stderr,"Replay complete: 1400 samples, accepted=%lu, crc_errors=%lu. Command simulation only.\n",
            (unsigned long)host.accepted_frames,(unsigned long)host.crc_errors);
    fprintf(stderr,"Continuity verified: max 10ms wheel-speed step %.7g rad/s; steering-command step %.6g deg.\n",
            max_speed_step,max_angle_step*(180.0f/3.14159265359f));
    fprintf(stderr,"1400 complete CAN/PWM batches prepared with mock device state. No hardware I/O.\n");
    return host.crc_errors==1 ? 0 : 3;
}
