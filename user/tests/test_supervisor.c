#include "drive_supervisor.h"
#include "single_wheel_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void NewFrame(DriveSupervisorInput *i)
{
    i->command_generation++; i->now_ms+=10; i->command_received_ms=i->now_ms;
}

static void TestSupervisor(void)
{
    DriveSupervisor s; DriveDirective d;
    DriveSupervisorInput i={.hardware_ready=true,.calibration_ready=true,
        .command_generation=1,.command_fresh=true};
    assert(DriveSupervisor_Init(&s,1000));
    assert(DriveSupervisor_Step(&s,&i,&d) && !d.allow_motion && d.mode==DRIVE_DISARMED);
    assert(DriveSupervisor_Request(&s,DRIVE_ENABLE,&i)==DRIVE_ACCEPTED);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_ENABLING && d.zero_drive && d.request_enable_mask==15);
    i.enabled_mask=7; i.now_ms=999;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_ENABLING && d.request_enable_mask==8);
    i.now_ms=1000;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_FAULT_WAIT);
    assert(d.faults&DRIVE_FAULT_ENABLE_TIMEOUT);
    i.now_ms=1001;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_FAULT_WAIT);
    NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_ENABLING);
    i.enabled_mask=15; i.now_ms=1020;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_RUNNING && d.allow_motion);
    i.command_fresh=false;
    assert(DriveSupervisor_Step(&s,&i,&d) && !d.allow_motion && d.hold_steering);
    i.command_fresh=true; NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.allow_motion);
    assert(DriveSupervisor_Request(&s,DRIVE_STOP,&i)==DRIVE_ACCEPTED);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_SOFT_STOP && d.zero_drive);
    assert(DriveSupervisor_Step(&s,&i,&d) && !d.allow_motion); /* Old frame cannot resume. */
    NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.allow_motion);

    i.faults=1; i.enabled_mask=0;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_FAULT_WAIT && d.fault_stop_required && d.zero_drive);
    assert(d.request_disable_mask==0); /* Physical fault-stop policy is not guessed. */
    NewFrame(&i); /* Packet DURING fault must not resume later. */
    assert(DriveSupervisor_Step(&s,&i,&d) && !d.allow_motion);
    i.faults=0; NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_FAULT_WAIT);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_FAULT_WAIT);
    /* A queued OLD packet parsed after recovery advances the counter, but its
       original RX timestamp must still prevent resuming movement. */
    i.command_generation++; i.command_received_ms=i.now_ms-5;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_FAULT_WAIT);
    NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_ENABLING);
    i.enabled_mask=15;
    assert(DriveSupervisor_Step(&s,&i,&d) && d.allow_motion && (d.fault_history&1));
    assert(DriveSupervisor_Request(&s,DRIVE_DISABLE,&i)==DRIVE_ACCEPTED);
    NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_DISARMED && !d.allow_motion);
    i.enabled_mask=0;
    assert(DriveSupervisor_Request(&s,DRIVE_ENTER_CALIBRATION,&i)==DRIVE_ACCEPTED);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.allow_calibration && !d.allow_motion);
    NewFrame(&i);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_CALIBRATING);
    assert(DriveSupervisor_Request(&s,DRIVE_EXIT_CALIBRATION,&i)==DRIVE_NOT_READY);
    i.calibration_stopped=true;
    assert(DriveSupervisor_Request(&s,DRIVE_EXIT_CALIBRATION,&i)==DRIVE_ACCEPTED);
    assert(DriveSupervisor_Step(&s,&i,&d) && d.mode==DRIVE_DISARMED);
    i.hardware_ready=false;
    assert(DriveSupervisor_Request(&s,DRIVE_ENABLE,&i)==DRIVE_NOT_READY);
    i.hardware_ready=true; i.now_ms=UINT32_MAX-5;
    assert(DriveSupervisor_Request(&s,DRIVE_ENABLE,&i)==DRIVE_ACCEPTED);
    i.now_ms=994;
    assert(DriveSupervisor_Step(&s,&i,&d) && (d.faults&DRIVE_FAULT_ENABLE_TIMEOUT));
    puts("PASS: supervisor explicit enable/disable, feedback-confirmed enable, soft-stop/timeout auto-resume, post-recovery fresh-frame barrier, calibration isolation");
}

static void TestSingleWheel(void)
{
    SingleWheelTest t;
    SingleWheelTestConfig c={.min_pulse_us=1400,.max_pulse_us=1600,.max_drive_rad_s=1,
        .drive_accel_rad_s2=8,.pulse_rate_us_s=1000,.lease_ms=200,.max_step_ms=20};
    const uint16_t reference[4]={1500,1500,1500,1500};
    assert(SingleWheelTest_Init(&t,&c,reference,0));
    assert(!SingleWheelTest_Target(&t,1,WHEEL_TEST_PULSE,1550,false,0));
    assert(SingleWheelTest_Target(&t,1,WHEEL_TEST_PULSE,1550,true,0));
    for (uint32_t now=10;now<=100;now+=10) assert(SingleWheelTest_Step(&t,now));
    assert(t.pulse_us[1]==1550 && t.pulse_us[0]==1500 && t.pulse_us[2]==1500 && t.pulse_us[3]==1500);
    assert(!SingleWheelTest_Target(&t,1,WHEEL_TEST_PULSE,1550.5f,true,199));
    assert(!SingleWheelTest_Target(&t,1,WHEEL_TEST_DRIVE,NAN,true,199));
    assert(SingleWheelTest_Step(&t,200) && t.expired && t.pulse_us[1]==1550);
    assert(SingleWheelTest_Target(&t,2,WHEEL_TEST_DRIVE,1,true,210));
    for (uint32_t now=220;now<=300;now+=10) assert(SingleWheelTest_Step(&t,now));
    assert(t.drive_rad_s[2]>0.7f && t.drive_rad_s[0]==0 && t.drive_rad_s[1]==0 && t.drive_rad_s[3]==0);
    assert(!SingleWheelTest_Target(&t,3,WHEEL_TEST_DRIVE,1,true,300));
    assert(!SingleWheelTest_Target(&t,2,WHEEL_TEST_PULSE,1500,false,300));
    assert(!SingleWheelTest_End(&t,true));
    assert(SingleWheelTest_Target(&t,2,WHEEL_TEST_DRIVE,0,false,300));
    for (uint32_t now=310;now<=430;now+=10) assert(SingleWheelTest_Step(&t,now));
    assert(t.drive_rad_s[2]==0 && !SingleWheelTest_End(&t,false));
    assert(SingleWheelTest_End(&t,true));
    assert(!SingleWheelTest_Target(&t,0,WHEEL_TEST_DRIVE,1,true,431));
    assert(SingleWheelTest_Init(&t,&c,reference,UINT32_MAX-5));
    assert(SingleWheelTest_Target(&t,0,WHEEL_TEST_DRIVE,1,true,UINT32_MAX-5));
    assert(SingleWheelTest_Step(&t,4) && fabsf(t.drive_rad_s[0]-0.08f)<0.00001f);
    assert(SingleWheelTest_Step(&t,300) && t.expired && t.drive_rad_s[0]==0);
    puts("PASS: one-wheel raw calibration targets, stopped-feedback gate, no axis switching while drive commanded, independent 200ms lease, slew/expiry/wrap");
}

void TestControlModes(void) { TestSupervisor(); TestSingleWheel(); }
