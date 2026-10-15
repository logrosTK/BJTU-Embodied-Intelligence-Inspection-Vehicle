#ifndef USER_SERVO_CALIBRATION_H
#define USER_SERVO_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_PWM_WHEELS 4U
#define SERVO_PWM_PERIOD_US 20000U

/* Pure mapping, no HAL. Body angle zero faces forward. Three anchors may
   increase OR decrease with angle; this servo family uses 500..2500us. */
typedef struct {
    float negative_rad, positive_rad;
    uint16_t negative_us, center_us, positive_us;
} ServoPwmCalibration;

bool ServoPwm_ValidCalibration(const ServoPwmCalibration *calibration);
/* Piecewise linear, rounded to whole us; rejects rather than clips.
   On failure, *pulse_us is unchanged. */
bool ServoPwm_Encode(const ServoPwmCalibration *calibration, float angle_rad, uint16_t *pulse_us);

#endif
