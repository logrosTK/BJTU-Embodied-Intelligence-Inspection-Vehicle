#ifndef USER_SINGLE_WHEEL_TEST_H
#define USER_SINGLE_WHEEL_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { WHEEL_TEST_PULSE, WHEEL_TEST_DRIVE } WheelTestKind;
typedef struct {
    uint16_t min_pulse_us, max_pulse_us;
    float max_drive_rad_s, drive_accel_rad_s2, pulse_rate_us_s;
    uint32_t lease_ms, max_step_ms;
} SingleWheelTestConfig;
typedef struct {
    float pulse_us[4], drive_rad_s[4]; /* COMMAND references, not measurements. */
    uint8_t wheel;
    WheelTestKind kind;
    uint32_t timestamp_ms, received_ms;
    float target;
    bool active, received, expired;
    SingleWheelTestConfig config;
} SingleWheelTest;

/* Pure INTERNAL calibration references, raw PWM us / MOTOR-output rad/s;
   no installed direction assumption and no normal navigation input.
   Start requires explicit current PWM references within the configured test
   window. This is not a servo-centering or actual-stopped detection API.
   Caller must grant exclusive calibration mode via the supervisor/executor.
   No HAL, no actual motor enabling or FLASH writes. */
bool SingleWheelTest_Init(SingleWheelTest *t, const SingleWheelTestConfig *config,
                          const uint16_t reference_us[4], uint32_t now_ms);
/* actual_drives_stopped must come from fresh MOTOR FEEDBACK, not zero commands.
   It is required for pulse tests and switching test wheel/type.
   Invalid targets never refresh the lease. PWM target must be integral us. */
bool SingleWheelTest_Target(SingleWheelTest *t, unsigned wheel, WheelTestKind kind,
                            float target, bool actual_drives_stopped, uint32_t received_ms);
/* Expired lease: stop drive references by ramp, freeze PWM reference.
   No zero/old normal navigation frame can renew this separate lease. */
bool SingleWheelTest_Step(SingleWheelTest *t, uint32_t now_ms);
/* End only after drive commands are zero AND caller confirms actual stopped.
   Leaves PWM references held; never re-enables normal navigation. */
bool SingleWheelTest_End(SingleWheelTest *t, bool actual_drives_stopped);

#endif
