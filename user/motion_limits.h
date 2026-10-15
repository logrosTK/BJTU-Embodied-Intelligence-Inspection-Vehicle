#ifndef USER_MOTION_LIMITS_H
#define USER_MOTION_LIMITS_H

#include "swerve_kinematics.h"

typedef struct {
    float linear_speed_m_s, yaw_speed_rad_s;
    float linear_accel_m_s2, yaw_accel_rad_s2;
    float wheel_speed_rad_s[SWERVE_WHEELS];
    float wheel_accel_rad_s2[SWERVE_WHEELS];
    float steering_speed_rad_s[SWERVE_WHEELS];
} MotionLimits;

bool MotionLimits_Valid(const MotionLimits *limits);
/* Pure foreground calculations; finite, strictly positive dt (seconds).
   On failure outputs stay unchanged. Input/output may alias.
   Previous values are explicit COMMAND references, never sensor estimates. */

/* Bound linear vector magnitude/yaw rate, then vector acceleration/yaw accel.
   Initial previous command should be zero. Rate limits are symmetric. */
bool MotionLimits_Twist(const MotionLimits *limits, const SwerveTwist *desired,
                        const SwerveTwist *previous, float dt_s, SwerveTwist *out);

/* After inverse kinematics and steering optimization, uniformly saturate
   wheel speeds, then bound each signed speed change and continuous steering
   angle change. Mechanical angles must already lie inside calibrated limits.
   NO wrapping across physical end stops. NO claim of actual steering arrival.
   This primitive does not implement a drive/steering synchronization policy. */
bool MotionLimits_Wheels(const MotionLimits *limits, const SwerveTargets *desired,
                         const SwerveTargets *previous, float dt_s, SwerveTargets *out);

/* Optional future closed-loop helper, requires four FRESH measured body-frame
   steering angles. Never pass PWM command angles as measurements. valid mask
   must be exactly 0x0f; with today's PWM hardware this function is NOT called.
   Multiply each drive speed by max(0, cos(target - measured)); angles unchanged. */
bool MotionLimits_CompensateMeasured(const SwerveTargets *desired,
                                    const float measured_rad[SWERVE_WHEELS],
                                    unsigned valid_mask, SwerveTargets *out);

#endif
