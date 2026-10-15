#ifndef USER_DM3519_H
#define USER_DM3519_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
} DmCanFrame;

typedef struct {
    uint16_t can_id, master_id;
    float p_max, v_max, t_max, max_speed;
    uint8_t parameters; /* bits 0..4: PMAX, VMAX, TMAX, mode, MAX_SPD */
    uint32_t control_mode;
    uint16_t raw_position, raw_velocity, raw_torque;
    float position_motor_rad, velocity_output_rad_s, torque_output_nm;
    uint8_t state, mos_celsius, coil_celsius;
    uint32_t feedback_ms, feedback_count, parameter_errors;
    bool feedback_received;
    uint8_t requested_register;
    bool request_pending;
} Dm3519;

void Dm3519_Init(Dm3519 *m, uint16_t can_id, uint16_t master_id);
void Dm3519_ReadRegister(const Dm3519 *m, uint8_t reg, DmCanFrame *out);
void Dm3519_StatusRequest(const Dm3519 *m, DmCanFrame *out);
void Dm3519_EnableFrame(const Dm3519 *m, bool enable, DmCanFrame *out);
void Dm3519_MarkRequest(Dm3519 *m, uint8_t reg);
bool Dm3519_Accept(Dm3519 *m, const DmCanFrame *frame, uint32_t received_ms);
bool Dm3519_FeedbackValid(const Dm3519 *m, uint32_t now_ms, uint32_t max_age_ms);
/* Pure encoder; no HAL transmission or implicit motor enabling. */
bool Dm3519_VelocityFrame(const Dm3519 *m, float output_rad_s, DmCanFrame *out);

#endif
