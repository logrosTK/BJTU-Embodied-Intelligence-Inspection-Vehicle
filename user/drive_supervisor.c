#include "drive_supervisor.h"

static bool Valid(const DriveSupervisor *s, const DriveSupervisorInput *i)
{
    return s && i && s->enable_timeout_ms && s->enable_timeout_ms < 0x80000000UL &&
           s->mode <= DRIVE_FAULT_WAIT && !(i->enabled_mask & ~15U);
}

/* 初始化启停管理，上电处于未使能状态；timeout 是等待四轮使能完成的最长毫秒数。 */
bool DriveSupervisor_Init(DriveSupervisor *s, uint32_t timeout)
{
    if (!s || !timeout || timeout >= 0x80000000UL) return false;
    *s = (DriveSupervisor){.mode=DRIVE_DISARMED,.enable_timeout_ms=timeout};
    return true;
}

static void Enable(DriveSupervisor *s, const DriveSupervisorInput *i)
{
    s->armed = true; s->mode = DRIVE_ENABLING; s->enable_started_ms = i->now_ms;
    s->internal_fault = 0; s->recovery_edge_seen = false;
}

/* 只认可停车或恢复正常之后新收到的有效帧，防止排队中的旧指令触发恢复。 */
static bool NewCommand(const DriveSupervisor *s, const DriveSupervisorInput *i)
{
    uint32_t after=i->command_received_ms-s->resume_after_ms;
    return i->command_fresh && i->command_generation!=s->command_barrier &&
           after && after<0x80000000UL && (uint32_t)(i->now_ms-i->command_received_ms)<0x80000000UL;
}

/* 处理内部启用、停车、失能和标定请求；当前尚未绑定串口命令或硬件发送。 */
DriveResult DriveSupervisor_Request(DriveSupervisor *s, DriveAction action,
                                     const DriveSupervisorInput *i)
{
    if (!Valid(s,i)) return DRIVE_BAD_ARGUMENT;
    switch (action) {
    case DRIVE_ENABLE:
        if (s->mode == DRIVE_RUNNING || s->mode == DRIVE_ENABLING) return DRIVE_ACCEPTED;
        if (s->mode != DRIVE_DISARMED && s->mode != DRIVE_SOFT_STOP) return DRIVE_BAD_STATE;
        if (i->faults || !i->hardware_ready) return DRIVE_NOT_READY;
        Enable(s,i); return DRIVE_ACCEPTED;
    case DRIVE_STOP:
        if (!s->armed || s->mode == DRIVE_FAULT_WAIT) return DRIVE_BAD_STATE;
        s->mode = DRIVE_SOFT_STOP; s->command_barrier = i->command_generation;
        s->resume_after_ms = i->now_ms;
        return DRIVE_ACCEPTED;
    case DRIVE_DISABLE:
        s->armed = false; s->mode = DRIVE_DISARMED;
        s->internal_fault = 0; s->recovery_edge_seen = false;
        return DRIVE_ACCEPTED;
    case DRIVE_ENTER_CALIBRATION:
        if (s->mode != DRIVE_DISARMED || s->armed) return DRIVE_BAD_STATE;
        if (i->faults || !i->calibration_ready || i->enabled_mask) return DRIVE_NOT_READY;
        s->mode = DRIVE_CALIBRATING; return DRIVE_ACCEPTED;
    case DRIVE_EXIT_CALIBRATION:
        if (s->mode != DRIVE_CALIBRATING) return DRIVE_BAD_STATE;
        if (!i->calibration_stopped) return DRIVE_NOT_READY;
        s->mode = DRIVE_DISARMED; s->armed = false; return DRIVE_ACCEPTED;
    default: return DRIVE_BAD_ARGUMENT;
    }
}

/* 根据实时故障、四轮使能反馈和新指令，给出本拍是否允许运动等控制意图。
   故障恢复后需再收到新速度帧才继续；这里只决定状态，不执行实际制动或 CAN/PWM。 */
bool DriveSupervisor_Step(DriveSupervisor *s, const DriveSupervisorInput *i, DriveDirective *out)
{
    if (!Valid(s,i) || !out) return false;
    uint32_t live = i->faults;
    if (s->armed && !i->hardware_ready) live |= DRIVE_FAULT_NOT_READY;
    if (s->mode == DRIVE_RUNNING && i->enabled_mask != 15U) live |= DRIVE_FAULT_DISABLED;
    if (live) {
        s->fault_history |= live; s->mode = DRIVE_FAULT_WAIT;
        s->command_barrier = i->command_generation; s->recovery_edge_seen = false;
    } else if (s->mode == DRIVE_FAULT_WAIT) {
        if (!s->armed) {
            s->mode = DRIVE_DISARMED; s->internal_fault = 0;
        } else if (!s->recovery_edge_seen) {
            s->recovery_edge_seen = true; s->command_barrier = i->command_generation;
            s->resume_after_ms = i->now_ms;
        } else if (NewCommand(s,i)) {
            Enable(s,i);
        }
    } else if (s->mode == DRIVE_SOFT_STOP && NewCommand(s,i)) {
        if (i->enabled_mask == 15U) s->mode = DRIVE_RUNNING;
        else Enable(s,i);
    }
    if (s->mode == DRIVE_ENABLING) {
        if (i->enabled_mask == 15U) s->mode = DRIVE_RUNNING;
        else if ((uint32_t)(i->now_ms - s->enable_started_ms) >= s->enable_timeout_ms) {
            s->internal_fault = DRIVE_FAULT_ENABLE_TIMEOUT;
            s->fault_history |= s->internal_fault; s->mode = DRIVE_FAULT_WAIT;
            s->recovery_edge_seen = false; s->command_barrier = i->command_generation;
        }
    }
    DriveDirective d = {.mode=s->mode,.faults=live | s->internal_fault,.fault_history=s->fault_history,
                        .zero_drive=true,.hold_steering=true};
    if (s->mode == DRIVE_RUNNING) {
        d.allow_motion = i->command_fresh; d.zero_drive = !d.allow_motion;
        d.hold_steering = !d.allow_motion;
    } else if (s->mode == DRIVE_ENABLING) {
        d.request_enable_mask = (uint8_t)(15U & ~i->enabled_mask);
    } else if (s->mode == DRIVE_CALIBRATING) {
        d.allow_calibration = i->calibration_ready && !live;
    } else if (s->mode == DRIVE_DISARMED) {
        d.request_disable_mask = 15U;
    } else if (s->mode == DRIVE_FAULT_WAIT) {
        d.fault_stop_required = true;
    }
    *out = d;
    return true;
}
