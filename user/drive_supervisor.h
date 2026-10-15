#ifndef USER_DRIVE_SUPERVISOR_H
#define USER_DRIVE_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { DRIVE_DISARMED, DRIVE_ENABLING, DRIVE_RUNNING, DRIVE_SOFT_STOP,
               DRIVE_CALIBRATING, DRIVE_FAULT_WAIT } DriveMode;
typedef enum { DRIVE_ENABLE, DRIVE_STOP, DRIVE_DISABLE, DRIVE_ENTER_CALIBRATION,
               DRIVE_EXIT_CALIBRATION } DriveAction;
typedef enum { DRIVE_ACCEPTED, DRIVE_BAD_ARGUMENT, DRIVE_BAD_STATE, DRIVE_NOT_READY } DriveResult;

#define DRIVE_FAULT_ENABLE_TIMEOUT (1UL << 30)
#define DRIVE_FAULT_NOT_READY (1UL << 29)
#define DRIVE_FAULT_DISABLED (1UL << 28)

typedef struct {
    uint32_t now_ms, command_generation, command_received_ms;
    uint32_t faults; /* Live caller/device faults; reserve bits28..30 above. */
    uint8_t enabled_mask; /* Fresh four-motor feedback, bits0..3. */
    bool command_fresh, hardware_ready, calibration_ready;
    bool calibration_stopped; /* Test commands zero AND fresh actual-stop confirmation. */
} DriveSupervisorInput;

typedef struct {
    DriveMode mode;
    bool allow_motion, zero_drive, hold_steering, allow_calibration;
    bool fault_stop_required;
    uint8_t request_enable_mask, request_disable_mask;
    uint32_t faults, fault_history;
} DriveDirective;

typedef struct {
    DriveMode mode;
    bool armed, recovery_edge_seen;
    uint32_t command_barrier, resume_after_ms, enable_started_ms, enable_timeout_ms;
    uint32_t internal_fault, fault_history;
} DriveSupervisor;

/* INTERNAL foreground state machine ONLY: returns intent, no HAL/CAN/PWM,
   no wire opcodes, no implicit integration into Inspection_Poll.
   command_generation must advance for EACH accepted velocity frame, not only
   value changes (e.g. HostParser.accepted_frames). A fault recovery requires a
   NEW valid frame with original RX timestamp strictly after the first healthy
   observation (not its later parse time). Same-ms samples wait for the next
   100Hz frame. Boot/explicit disable
   always require explicit enable. STOP permits later new velocity to resume.
   Caller derives hardware_ready/faults from fresh device data and calibration;
   these booleans are not measurements obtained by this module.
   Fault mode requests inhibit/stop only; physical brake/disable behavior is
   intentionally left to the executor policy, which is not confirmed yet.
   In calibration, zero_drive inhibits NAVIGATION; allow_calibration separately
   permits the single-wheel test references. Exit requires calibration_stopped.
   Enable timeout uses unsigned tick subtraction; configure below 2^31 ms. */
bool DriveSupervisor_Init(DriveSupervisor *s, uint32_t enable_timeout_ms);
DriveResult DriveSupervisor_Request(DriveSupervisor *s, DriveAction action,
                                     const DriveSupervisorInput *input);
bool DriveSupervisor_Step(DriveSupervisor *s, const DriveSupervisorInput *input,
                          DriveDirective *out);

#endif
