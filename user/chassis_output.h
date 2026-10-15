#ifndef USER_CHASSIS_OUTPUT_H
#define USER_CHASSIS_OUTPUT_H

#include "chassis_planner.h"
#include "dm3519.h"
#include "servo_calibration.h"

enum {
    CHASSIS_OUTPUT_ARGUMENT = 1U,
    CHASSIS_OUTPUT_BUS_OFF = 2U,
    CHASSIS_OUTPUT_PLAN_AGE = 4U,
    CHASSIS_OUTPUT_PLAN_NUMBER = 8U,
    CHASSIS_OUTPUT_LIMITS = 16U
};
enum {
    CHASSIS_WHEEL_DIRECTION = 1U,
    CHASSIS_WHEEL_RATIO = 2U,
    CHASSIS_WHEEL_CALIBRATION = 4U,
    CHASSIS_WHEEL_ADDRESS = 8U,
    CHASSIS_WHEEL_PARAMETERS = 16U,
    CHASSIS_WHEEL_FEEDBACK = 32U,
    CHASSIS_WHEEL_FAULT = 64U,
    CHASSIS_WHEEL_SPEED = 128U,
    CHASSIS_WHEEL_STEERING = 256U,
    CHASSIS_WHEEL_ENCODING = 512U
};

typedef struct {
    int drive_sign[SWERVE_WHEELS]; /* Must explicitly be +1 or -1. */
    float external_ratio[SWERVE_WHEELS]; /* Output-shaft speed / wheel speed. */
    ServoPwmCalibration steering[SWERVE_WHEELS];
    uint32_t plan_max_age_ms, device_max_age_ms;
} ChassisOutputConfig;

typedef struct {
    const ChassisPlan *plan;
    const Dm3519 *motors; /* Four entries, FL/FR/RL/RR; IDs 1..4, Master 11..14 hex. */
    const MotionLimits *limits; /* Wheel-shaft limits for the configured ratios. */
    uint32_t now_ms;
    bool bus_off;
} ChassisOutputInput;

typedef struct {
    DmCanFrame drive[SWERVE_WHEELS];
    uint16_t steering_us[SWERVE_WHEELS];
    float steering_rad[SWERVE_WHEELS];
    uint32_t planned_ms, built_ms;
    bool command_timeout;
} ChassisOutputBatch;

typedef struct {
    uint32_t global_errors;
    uint16_t wheel_errors[SWERVE_WHEELS];
} ChassisOutputDiagnostics;

/* Pure data preparation: no HAL, enable, transmission, or actuator side effect.
   Success means ONE complete valid four-wheel batch, NOT permission to drive or
   proof of installed calibration. A disabled motor may be encoded; an enabled
   hardware state and actual stop/fault behavior belong to the future executor.
   Timeout plans are allowed so their deceleration targets can be encoded.
   Reject stale plans/feedback, bus-off, motor faults, incomplete directions,
   calibration, parameters, speed/angle limit violations, and wrong wheel IDs.
   External ratios multiply wheel speed; never apply the internal 19.2 again.
   On failure, out is unchanged and MUST NOT be reused as a fresh batch.
   Diagnostics are always overwritten when non-NULL. All calls are foreground.
   Atomicity is only for BUILDING data; this API does not make hardware CAN/PWM
   delivery simultaneous. No per-wheel clipping or partial returned batch. */
bool ChassisOutput_Build(const ChassisOutputConfig *config, const ChassisOutputInput *input,
                         ChassisOutputBatch *out, ChassisOutputDiagnostics *diagnostics);

#endif
