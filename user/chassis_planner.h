#ifndef USER_CHASSIS_PLANNER_H
#define USER_CHASSIS_PLANNER_H

#include "motion_limits.h"
#include <stdint.h>

typedef struct {
    SwerveGeometry geometry;
    SwerveSteeringLimit steering[SWERVE_WHEELS];
    uint32_t max_step_ms;
    float wheel_hold_rad_s;
} ChassisPlannerConfig;

typedef struct {
    SwerveTwist body_reference; /* Filtered request, NOT measured/achieved twist. */
    SwerveTargets wheels;       /* BODY-frame command references, NOT feedback. */
    uint32_t timestamp_ms;
    float saturation_scale;
    bool command_timeout;
} ChassisPlan;

typedef struct {
    ChassisPlannerConfig config;
    ChassisPlan plan;
    bool initialized;
} ChassisPlanner;

/* Pure foreground math with explicit command history: no HAL, motor enable,
   PWM output, or inference of actual steering position. Initialization starts
   drive references at zero and requires explicitly supplied steering references.
   Config/outputs remain unchanged on failure. */
bool ChassisPlanner_Init(ChassisPlanner *p, const ChassisPlannerConfig *config,
                         const float reference_rad[SWERVE_WHEELS], uint32_t now_ms);

/* Caller supplies command validity from the communication watchdog (200ms).
   Stale command: decelerate each drive reference to zero, hold last steering
   reference. A new valid command resumes automatically without a latch.
   Fresh command: smooth twist -> inverse -> optimize -> uniform saturation ->
   steering slew -> command-alignment speed reduction -> signed drive slew.
   Alignment uses the NEW steering COMMAND, never the measured-angle API.
   There is no stop/steer/wait phase or fixed steering-arrival delay.
   Near-zero wheel demands hold steering to avoid undefined-direction chatter.
   Step duration is capped by max_step_ms (no accumulated catch-up jump).
   Same-timestamp calls do nothing; invalid fresh inputs return false.
   Changing limits incompatibly with stored references requires reinitializing
   the planning history; this API does not choose a physical stop/fault policy. */
bool ChassisPlanner_Step(ChassisPlanner *p, const MotionLimits *limits,
                         const SwerveTwist *request, bool command_valid, uint32_t now_ms);

#endif
