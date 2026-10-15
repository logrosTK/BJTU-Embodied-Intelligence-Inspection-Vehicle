#ifndef USER_SWERVE_KINEMATICS_H
#define USER_SWERVE_KINEMATICS_H

#include <stdbool.h>

#define SWERVE_WHEELS 4U

/* Wheel order: front left, front right, rear left, rear right.
   Origin: rectangle center; X forward, Y left, Z up (CCW positive).
   Ideal planar model at the steering pivots; no slip/offset compensation. */
typedef struct {
    float wheel_radius_m;
    float pivot_track_m;
    float pivot_wheelbase_m;
} SwerveGeometry;

typedef struct { float vx_m_s, vy_m_s, wz_rad_s; } SwerveTwist;
typedef struct { float speed_rad_s, angle_rad; } SwerveWheel;
typedef struct { SwerveWheel wheel[SWERVE_WHEELS]; } SwerveTargets;
typedef struct { float min_rad, max_rad; } SwerveSteeringLimit;

/* All functions run in foreground, use no HAL, and issue no actuator commands.
   On failure, outputs are unchanged; callers MUST check the returned bool.
   Input and output targets may alias. All units are SI. */

/* Unconstrained inverse kinematics: speed >= 0, atan2 angle in [-pi, pi].
   A wheel with exactly zero velocity keeps its supplied reference angle.
   References are required, finite values; they are NOT measured servo angles. */
bool Swerve_Inverse(const SwerveGeometry *geometry, const SwerveTwist *twist,
                    const float reference_rad[SWERVE_WHEELS], SwerveTargets *out);

/* Select the closest feasible equivalent angle to each supplied reference.
   A pi turn with reversed drive speed produces the same velocity vector.
   Limits are calibrated BODY-frame angles, min < max, span <= 2*pi;
   references must already be within limits. No guessed servo range/zero.
   At zero wheel speed, hold the reference. Fail if any wheel has no solution. */
bool Swerve_Optimize(const SwerveTargets *in,
                     const SwerveSteeringLimit limits[SWERVE_WHEELS],
                     const float reference_rad[SWERVE_WHEELS], SwerveTargets *out);

/* Uniformly reduce ALL wheel speeds to honor explicitly supplied positive
   wheel-shaft speed limits, preserving the commanded velocity proportions.
   Angles are unchanged. Returned scale is in [0,1]; optional scale may be NULL.
   This is speed saturation, not acceleration or steering-rate limiting. */
bool Swerve_LimitSpeeds(const SwerveTargets *in,
                        const float max_rad_s[SWERVE_WHEELS],
                        SwerveTargets *out, float *scale);

#endif
