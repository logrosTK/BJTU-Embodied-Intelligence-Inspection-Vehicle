#ifndef USER_INSPECTION_CONFIG_H
#define USER_INSPECTION_CONFIG_H

#define INSPECTION_WHEELS 4U
/* Confirmed dimensions: use STEERING PIVOT spacing, not tire-center track. */
#define INSPECTION_WHEEL_RADIUS_M 0.0625f
#define INSPECTION_PIVOT_TRACK_M 0.285f
#define INSPECTION_PIVOT_WHEELBASE_M 0.395f
#define INSPECTION_EXTERNAL_DRIVE_RATIO 1.0f
/* V1.2 motor manual p20: 435rpm no-load at 24V. User requested 70%.
   This is a software ceiling, not guaranteed loaded chassis performance. */
#define INSPECTION_MOTOR_NO_LOAD_RPM 435.0f
#define INSPECTION_MOTOR_SPEED_FRACTION 0.70f
/* Temporary commissioning values, NOT measured hardware specifications. */
#define INSPECTION_LINEAR_ACCEL_M_S2 0.5f
#define INSPECTION_YAW_SPEED_RAD_S 1.0f
#define INSPECTION_YAW_ACCEL_RAD_S2 1.0f
#define INSPECTION_STEERING_SPEED_RAD_S 3.14159265359f
#define INSPECTION_STEERING_LIMIT_RAD 2.09439510239f /* +/-120deg */
/* Calculation preview only, no automatic actuator output. */
#define INSPECTION_CONTROL_PERIOD_MS 10U
#define INSPECTION_CONTROL_MAX_STEP_MS 20U
#define INSPECTION_WHEEL_HOLD_RAD_S 0.05f
#define INSPECTION_MASTER_BASE 0x11U
#define INSPECTION_DEVICE_MAX_AGE_MS 100U
#define INSPECTION_FEEDBACK_PERIOD_MS 20U
#define INSPECTION_DIAGNOSTICS_PERIOD_MS 1000U
/* 50 status requests/s per motor, interleaved across four motors. */
#define INSPECTION_CAN_POLL_INTERVAL_MS 5U
#define INSPECTION_PARAMETER_INTERVAL_MS 20U

#endif
