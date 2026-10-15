#ifndef USER_SERVO_PWM_H
#define USER_SERVO_PWM_H

#include "stm32f4xx_hal.h"
#include "servo_calibration.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    ServoPwmCalibration calibration[SERVO_PWM_WHEELS];
    bool configured, running;
} ServoPwm4;

/* Zero-initialize the object before first Configure. Foreground/exclusive TIM3
   owner only. Uses the project's 84MHz TIM3 / PSC83 / ARR19999 setup.
   Configure does not enable PWM. No automatic calls from Inspection_Init. */
bool ServoPwm4_Configure(ServoPwm4 *p, TIM_HandleTypeDef *timer,
                       const ServoPwmCalibration calibration[SERVO_PWM_WHEELS]);
/* Explicit start at calibrated center; NOT evidence that steering has settled. */
bool ServoPwm4_Start(ServoPwm4 *p);
/* All four angles are validated before any CCR changes; latches together at
   the next 20ms frame. The function does not wait for a servo to arrive. */
bool ServoPwm4_SetAngles(ServoPwm4 *p, const float angles_rad[SERVO_PWM_WHEELS]);
/* Stops PWM only; does not define chassis braking or drive-motor stop policy. */
bool ServoPwm4_Stop(ServoPwm4 *p);

#endif
