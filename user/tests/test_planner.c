#include "chassis_planner.h"
#include "inspection_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ChassisPlannerConfig Config(void)
{
    ChassisPlannerConfig c = {
        .geometry={0.0625f,0.285f,0.395f}, .max_step_ms=20, .wheel_hold_rad_s=0.05f
    };
    for (unsigned i=0;i<4;++i) c.steering[i]=(SwerveSteeringLimit){-2.09439510239f,2.09439510239f};
    return c;
}
static MotionLimits Limits(void)
{
    MotionLimits l={.linear_speed_m_s=1.99f,.yaw_speed_rad_s=1,
        .linear_accel_m_s2=0.5f,.yaw_accel_rad_s2=1};
    for (unsigned i=0;i<4;++i) {
        l.wheel_speed_rad_s[i]=31.84f; l.wheel_accel_rad_s2[i]=8;
        l.steering_speed_rad_s[i]=3.14159265359f;
    }
    return l;
}
static void CheckStep(const ChassisPlan *before,const ChassisPlan *after,
                      const MotionLimits *l,float dt)
{
    assert(hypotf(after->body_reference.vx_m_s,after->body_reference.vy_m_s)<=l->linear_speed_m_s+0.00001f);
    assert(hypotf(after->body_reference.vx_m_s-before->body_reference.vx_m_s,
                  after->body_reference.vy_m_s-before->body_reference.vy_m_s)<=l->linear_accel_m_s2*dt+0.00001f);
    assert(fabsf(after->body_reference.wz_rad_s-before->body_reference.wz_rad_s)<=l->yaw_accel_rad_s2*dt+0.00001f);
    for (unsigned i=0;i<4;++i) {
        assert(fabsf(after->wheels.wheel[i].speed_rad_s)<=l->wheel_speed_rad_s[i]+0.00001f);
        assert(fabsf(after->wheels.wheel[i].speed_rad_s-before->wheels.wheel[i].speed_rad_s)<=l->wheel_accel_rad_s2[i]*dt+0.00001f);
        assert(fabsf(after->wheels.wheel[i].angle_rad-before->wheels.wheel[i].angle_rad)<=l->steering_speed_rad_s[i]*dt+0.00001f);
        assert(fabsf(after->wheels.wheel[i].angle_rad)<=2.094396f);
    }
}

static void TestContinuous(void)
{
    ChassisPlannerConfig c=Config(); MotionLimits l=Limits(); ChassisPlanner p;
    const float reference[4]={0};
    assert(ChassisPlanner_Init(&p,&c,reference,0));
    /* Switch among straight, sideways, reverse, mixed rotation and stop.
       Every transition is checked for signed speed and angle continuity. */
    const SwerveTwist requests[]={{0.6f,0,0},{0,0.6f,0},{-0.6f,0,0},
                                  {0.3f,-0.4f,1},{0,0,-1},{0,0,0}};
    uint32_t now=0; unsigned turning_with_drive=0;
    for (unsigned segment=0;segment<6;++segment) {
        for (unsigned step=0;step<300;++step) {
            ChassisPlan before=p.plan; now+=10;
            assert(ChassisPlanner_Step(&p,&l,&requests[segment],true,now));
            assert(!p.plan.command_timeout); CheckStep(&before,&p.plan,&l,0.01f);
            for (unsigned i=0;i<4;++i)
                if (fabsf(p.plan.wheels.wheel[i].angle_rad-before.wheels.wheel[i].angle_rad)>0.0001f &&
                    fabsf(p.plan.wheels.wheel[i].speed_rad_s)>0.2f) ++turning_with_drive;
        }
    }
    assert(turning_with_drive>50); /* No global stop/steer/wait state. */
    for (unsigned i=0;i<4;++i) assert(fabsf(p.plan.wheels.wheel[i].speed_rad_s)<0.00001f);
    /* Starting sideways should move wheel references on the FIRST step, while
       drive demand is reduced using the staged command angle, not a sensor. */
    assert(ChassisPlanner_Init(&p,&c,reference,0));
    SwerveTwist side={0,0.6f,0};
    assert(ChassisPlanner_Step(&p,&l,&side,true,10));
    for (unsigned i=0;i<4;++i) {
        assert(p.plan.wheels.wheel[i].angle_rad>0.03f);
        assert(p.plan.wheels.wheel[i].speed_rad_s>0 && p.plan.wheels.wheel[i].speed_rad_s<0.01f);
    }
}

static void TestWatchdogRecovery(void)
{
    ChassisPlannerConfig c=Config(); MotionLimits l=Limits(); ChassisPlanner p;
    const float reference[4]={0}; SwerveTwist motion={0.5f,0.2f,0.1f};
    assert(ChassisPlanner_Init(&p,&c,reference,0));
    uint32_t now=0;
    for (unsigned n=0;n<200;++n) {now+=10; assert(ChassisPlanner_Step(&p,&l,&motion,true,now));}
    float held[4]; for (unsigned i=0;i<4;++i) held[i]=p.plan.wheels.wheel[i].angle_rad;
    for (unsigned n=0;n<200;++n) {
        ChassisPlan before=p.plan; now+=10;
        assert(ChassisPlanner_Step(&p,&l,NULL,false,now)); CheckStep(&before,&p.plan,&l,0.01f);
        assert(p.plan.command_timeout);
        for (unsigned i=0;i<4;++i) {
            assert(p.plan.wheels.wheel[i].angle_rad==held[i]);
            assert(fabsf(p.plan.wheels.wheel[i].speed_rad_s)<=fabsf(before.wheels.wheel[i].speed_rad_s));
        }
    }
    for (unsigned i=0;i<4;++i) assert(p.plan.wheels.wheel[i].speed_rad_s==0);
    /* No rearm API or latch: a fresh valid command resumes on the next step. */
    assert(ChassisPlanner_Step(&p,&l,&motion,true,now+10));
    assert(!p.plan.command_timeout && p.plan.wheels.wheel[0].speed_rad_s>0);
}

static void TestTimeAndInvalid(void)
{
    ChassisPlannerConfig c=Config(); MotionLimits l=Limits(); ChassisPlanner p;
    const float reference[4]={0}; SwerveTwist t={1,0,0};
    assert(ChassisPlanner_Init(&p,&c,reference,UINT32_MAX-5U));
    ChassisPlan before=p.plan;
    assert(ChassisPlanner_Step(&p,&l,&t,true,4)); CheckStep(&before,&p.plan,&l,0.01f);
    before=p.plan;
    assert(ChassisPlanner_Step(&p,&l,&t,true,1004)); CheckStep(&before,&p.plan,&l,0.02f);
    before=p.plan;
    assert(ChassisPlanner_Step(&p,&l,&t,true,1004)); assert(memcmp(&p.plan,&before,sizeof(before))==0);
    t.vx_m_s=NAN; assert(!ChassisPlanner_Step(&p,&l,&t,true,1014));
    assert(memcmp(&p.plan,&before,sizeof(before))==0);
    c.max_step_ms=0; assert(!ChassisPlanner_Init(&p,&c,reference,0));
    assert(memcmp(&p.plan,&before,sizeof(before))==0);
    c=Config(); c.steering[3].max_rad=-1;
    assert(!ChassisPlanner_Init(&p,&c,reference,0));
    assert(memcmp(&p.plan,&before,sizeof(before))==0);
}

void TestPlanner(void)
{
    TestContinuous(); TestWatchdogRecovery(); TestTimeAndInvalid();
    puts("PASS: continuous chassis planning, timeout braking/angle hold/auto-resume, tick wrap/stall cap, atomic failures");
}
