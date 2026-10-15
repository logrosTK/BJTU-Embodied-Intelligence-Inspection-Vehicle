#include "servo_pwm.h"

static const uint32_t channels[4] = {TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};

/* 一次写好四路预装载值，避免跨过定时器更新时刻时只更新了一部分舵机。 */
static void WritePulses(ServoPwm4 *p, const uint16_t pulse[4])
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    uint32_t old_udis = p->timer->Instance->CR1 & TIM_CR1_UDIS;
    /* HAL_TIM_PWM_ConfigChannel enables OC preload. Inhibit hardware update
       while filling all four preload registers, even across a frame boundary. */
    p->timer->Instance->CR1 |= TIM_CR1_UDIS;
    for (unsigned i = 0; i < 4; ++i) __HAL_TIM_SET_COMPARE(p->timer, channels[i], pulse[i]);
    if (!old_udis) p->timer->Instance->CR1 &= ~TIM_CR1_UDIS;
    __set_PRIMASK(mask);
}

/* 检查 TIM3 配置并保存四路标定；对象首次使用前需清零，此函数不启动 PWM。 */
bool ServoPwm4_Configure(ServoPwm4 *p, TIM_HandleTypeDef *timer, const ServoPwmCalibration c[4])
{
    if (!p || !timer || !c || p->running || timer->Instance != TIM3 ||
        timer->Init.Prescaler != 83U || timer->Init.Period != SERVO_PWM_PERIOD_US - 1U) return false;
    for (unsigned i = 0; i < 4; ++i) if (!ServoPwm_ValidCalibration(&c[i])) return false;
    p->timer = timer;
    for (unsigned i = 0; i < 4; ++i) p->calibration[i] = c[i];
    p->configured = true;
    return true;
}

/* 真正启动四路 PWM，首帧输出各自中位；这会控制舵机，但不代表舵机已经到位。 */
bool ServoPwm4_Start(ServoPwm4 *p)
{
    if (!p || !p->configured || p->running || (p->timer->Instance->CR1 & TIM_CR1_UDIS)) return false;
    uint16_t center[4];
    for (unsigned i = 0; i < 4; ++i) center[i] = p->calibration[i].center_us;
    WritePulses(p, center);
    /* Load preloads before the first PWM pulse, while all channels are off. */
    p->timer->Instance->EGR = TIM_EGR_UG;
    p->running = true;
    for (unsigned i = 0; i < 4; ++i) {
        if (HAL_TIM_PWM_Start(p->timer, channels[i]) != HAL_OK) {
            (void)ServoPwm4_Stop(p); return false;
        }
    }
    return true;
}

/* 四个角度全部检查通过后统一更新 PWM，下一周期生效，不等待实际转向完成。 */
bool ServoPwm4_SetAngles(ServoPwm4 *p, const float angle[4])
{
    if (!p || !p->configured || !p->running || !angle) return false;
    uint16_t pulse[4];
    for (unsigned i = 0; i < 4; ++i)
        if (!ServoPwm_Encode(&p->calibration[i], angle[i], &pulse[i])) return false;
    WritePulses(p, pulse);
    return true;
}

/* 关闭四路舵机 PWM；只停止信号输出，不负责驱动电机制动或整车停车。 */
bool ServoPwm4_Stop(ServoPwm4 *p)
{
    if (!p || !p->configured) return false;
    bool ok = true;
    for (unsigned i = 0; i < 4; ++i)
        if (HAL_TIM_PWM_Stop(p->timer, channels[i]) != HAL_OK) ok = false;
    const uint16_t zero[4] = {0}; WritePulses(p, zero);
    p->running = !ok;
    return ok;
}
