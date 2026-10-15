#include "dm3519.h"
#include "byte_codec.h"
#include <math.h>
#include <string.h>

/* DM-S3519-1EC V1.2, pp.7-9 and 16-19. Velocity/torque are at the
   OUTPUT shaft; position is at the motor, BEFORE the internal reducer. */

/* 保存电机命令 ID 和反馈 Master ID，清空状态；不会向电机写配置。 */
void Dm3519_Init(Dm3519 *m, uint16_t can_id, uint16_t master_id)
{
    memset(m, 0, sizeof(*m)); m->can_id = can_id; m->master_id = master_id;
}

/* 构造只读寄存器查询帧，不执行发送。 */
void Dm3519_ReadRegister(const Dm3519 *m, uint8_t reg, DmCanFrame *out)
{
    *out = (DmCanFrame){.id = 0x7FF, .length = 8};
    WriteU16LE(out->data, m->can_id); out->data[2] = 0x33; out->data[3] = reg;
}
/* 查询帧提交成功后记录寄存器号，用于核对随后收到的回复。 */
void Dm3519_MarkRequest(Dm3519 *m, uint8_t reg)
{
    m->requested_register = reg; m->request_pending = true;
}

/* Supplemental commands verified in manufacturer motor-sdk Python/u2can/DM_CAN.py.
   These builders do not send commands automatically. */
/* 构造电机状态查询帧；仅查询，不使能电机。 */
void Dm3519_StatusRequest(const Dm3519 *m, DmCanFrame *out)
{
    *out = (DmCanFrame){.id = 0x7FF, .length = 8};
    WriteU16LE(out->data, m->can_id); out->data[2] = 0xCC;
}
/* 构造使能或失能帧；这里只生成字节，调用者另行决定是否发送。 */
void Dm3519_EnableFrame(const Dm3519 *m, bool enable, DmCanFrame *out)
{
    *out = (DmCanFrame){.id = m->can_id, .length = 8};
    memset(out->data, 0xFF, 7); out->data[7] = enable ? 0xFC : 0xFD;
}

/* 用读回的量程换算反馈：位置在电机侧，速度和力矩在减速后的输出轴侧。 */
static void Decode(Dm3519 *m)
{
    m->position_motor_rad = ((float)m->raw_position * (2.0f / 65535.0f) - 1.0f) * m->p_max;
    m->velocity_output_rad_s = ((float)m->raw_velocity * (2.0f / 4095.0f) - 1.0f) * m->v_max;
    m->torque_output_nm = ((float)m->raw_torque * (2.0f / 4095.0f) - 1.0f) * m->t_max;
}

/* 区分寄存器回复与运行反馈，更新参数、状态和时间；不匹配的帧直接拒绝。 */
bool Dm3519_Accept(Dm3519 *m, const DmCanFrame *frame, uint32_t received_ms)
{
    if (frame->id != m->master_id || frame->length != 8) return false;
    const uint8_t *b = frame->data;
    if (ReadU16LE(b) == m->can_id && b[2] == 0x33) {
        /* Never mistake a register reply for 12-bit velocity feedback. */
        if (!m->request_pending || b[3] != m->requested_register) return false;
        m->request_pending = false;
        uint8_t bit;
        float *value;
        switch (b[3]) {
        case 0x15: bit = 1; value = &m->p_max; break;
        case 0x16: bit = 2; value = &m->v_max; break;
        case 0x17: bit = 4; value = &m->t_max; break;
        case 0x06: bit = 16; value = &m->max_speed; break;
        case 0x0A:
            m->control_mode = ReadU32LE(b + 4);
            if (m->control_mode >= 1 && m->control_mode <= 3) m->parameters |= 8;
            else { m->parameters &= (uint8_t)~8U; ++m->parameter_errors; }
            return true;
        default: return false;
        }
        float f = ReadF32LE(b + 4);
        if (!isfinite(f) || f <= 0.0f) {
            m->parameters &= (uint8_t)~bit; ++m->parameter_errors; return false;
        }
        *value = f; m->parameters |= bit;
        if (m->feedback_received && (m->parameters & 7) == 7) Decode(m);
        return true;
    }
    if ((b[0] & 0x0FU) != (m->can_id & 0x0FU)) return false;
    uint8_t state = b[0] >> 4;
    if (state != 0 && state != 1 && state != 5 && state != 6 &&
        (state < 8 || state > 14)) return false;
    m->state = state;
    m->raw_position = (uint16_t)(((uint16_t)b[1] << 8) | b[2]);
    m->raw_velocity = (uint16_t)(((uint16_t)b[3] << 4) | (b[4] >> 4));
    m->raw_torque = (uint16_t)(((uint16_t)(b[4] & 0x0F) << 8) | b[5]);
    m->mos_celsius = b[6]; m->coil_celsius = b[7];
    m->feedback_ms = received_ms; m->feedback_received = true; ++m->feedback_count;
    if ((m->parameters & 7) == 7) Decode(m);
    return true;
}

/* 检查量程已知、状态正常且反馈未过期；返回 true 不代表电机已经使能。 */
bool Dm3519_FeedbackValid(const Dm3519 *m, uint32_t now_ms, uint32_t max_age_ms)
{
    return max_age_ms && m->feedback_received && (m->parameters & 7) == 7 &&
           m->state <= 1 && (uint32_t)(now_ms - m->feedback_ms) < max_age_ms;
}

/* 构造速度模式 CAN 帧，单位是输出轴 rad/s，不再除以内置减速比；超限则失败。 */
bool Dm3519_VelocityFrame(const Dm3519 *m, float output_rad_s, DmCanFrame *out)
{
    if (m->can_id < 1 || m->can_id > 15 || m->parameters != 31 ||
        m->control_mode != 3 || !isfinite(output_rad_s) ||
        fabsf(output_rad_s) > m->max_speed || fabsf(output_rad_s) > m->v_max)
        return false;
    *out = (DmCanFrame){.id = (uint16_t)(0x200U + m->can_id), .length = 4};
    WriteF32LE(out->data, output_rad_s); return true;
}
