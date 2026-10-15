#ifndef USER_INSPECTION_APP_H
#define USER_INSPECTION_APP_H

#include "host_protocol.h"
#include "dm3519.h"
#include "hwt101ct.h"
#include "swerve_kinematics.h"
#include "motion_limits.h"
#include "servo_pwm.h"
#include "chassis_planner.h"
#include "chassis_output.h"

/* Call after CubeMX peripheral initialization; call Poll continuously.
   This communication stage NEVER enables motors, sends velocity targets,
   writes motor/sensor configuration, or starts steering PWM. */
void Inspection_Init(void);
void Inspection_Poll(void);
bool Inspection_GetMotion(HostMotion *out);
/* Foreground calculation only. Returns false for missing/expired commands or
   invalid input, leaving out unchanged. Unconstrained model outputs MUST go
   through steering calibration/limits and drive limits before actuation.
   Does not change telemetry or send PWM/CAN commands. */
bool Inspection_CalculateWheels(const float reference_rad[SWERVE_WHEELS], SwerveTargets *out);
/* No actuator side effects. Limits require all four motor parameter sets;
   ceiling = 0.70 * min(435rpm physical reference, MAX_SPD, VMAX), per wheel.
   False leaves out unchanged. Steering/accel defaults are commissioning values. */
bool Inspection_GetMotionLimits(MotionLimits *out);
/* Foreground-only 100Hz calculation preview. NOT motor/steering feedback or
   an enabled-actuator state. Not transmitted as applied steering targets.
   Default startup history is zero drive and straight steering commands.
   False means unavailable; out is unchanged. No actuator side effects. */
bool Inspection_GetControlPreview(ChassisPlan *out);
/* Prepares CAN/PWM data only. Requires explicitly configured drive and steering
   directions plus fresh four-motor feedback. No automatic calls or sending. */
bool Inspection_BuildOutputPreview(ChassisOutputBatch *out, ChassisOutputDiagnostics *diagnostics);
/* Installs the provisional manual-based map for one wheel. No PWM start or
   implied verification of the installed direction. */
bool Inspection_ConfigureSteering(unsigned wheel, int direction);
/* Provisional manual-based PWM map: 500/1500/2500us at -135/0/+135deg,
   with a +/-120deg software limit. REQUIRES installed direction verification.
   Positive pulse direction can be reversed per wheel using direction=-1. */
bool Inspection_SteeringCalibration(int direction, ServoPwmCalibration *out);
const Dm3519 *Inspection_Motor(unsigned wheel);
const Hwt101ct *Inspection_Gyro(void);
/* Direction/ratio are shared by wheel feedback and output-preview mapping.
   Confirm the installed sign before use; no guessed direction defaults. */
bool Inspection_ConfigureWheelFeedback(unsigned wheel, int sign, float ratio);

#endif
