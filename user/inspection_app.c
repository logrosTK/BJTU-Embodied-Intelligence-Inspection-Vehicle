#include "inspection_app.h"
#include "inspection_config.h"
#include "byte_codec.h"
#include "uart_dma.h"
#include "can_link.h"
#include "usart.h"
#include "can.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static UartDma host_uart, gyro_uart, debug_uart;
static CanLink motor_can;
static HostParser host;
static Hwt101ct gyro;
static Dm3519 motors[INSPECTION_WHEELS];
static struct { int sign; float ratio; } wheel_feedback[INSPECTION_WHEELS];
static ServoPwmCalibration steering_calibration[INSPECTION_WHEELS];
static ChassisPlanner planner;
static bool preview_valid;
static uint32_t last_control;
typedef struct { uint8_t type, length; bool ready; uint8_t payload[40]; } Telemetry;
static Telemetry pending[3];
static uint16_t tx_sequence;
static uint32_t last_feedback, last_diag, last_status, last_parameter, replaced;
static unsigned next_telemetry, poll_wheel, parameter_cursor;
static bool initialized;

static UartDma *FindUart(UART_HandleTypeDef *u)
{
    if (u == host_uart.uart) return &host_uart;
    if (u == gyro_uart.uart) return &gyro_uart;
    if (u == debug_uart.uart) return &debug_uart;
    return NULL;
}

/* 初始化串口 DMA、CAN 和设备解析器；当前只启动通讯，不使能电机或舵机。 */
void Inspection_Init(void)
{
    Host_Init(&host); Hwt_Init(&gyro);
    for (unsigned i = 0; i < INSPECTION_WHEELS; ++i)
        Dm3519_Init(&motors[i], (uint16_t)(i + 1U), (uint16_t)(INSPECTION_MASTER_BASE + i));
    for (unsigned i = 0; i < INSPECTION_WHEELS; ++i) {
        wheel_feedback[i].sign = 0;
        wheel_feedback[i].ratio = INSPECTION_EXTERNAL_DRIVE_RATIO;
        steering_calibration[i] = (ServoPwmCalibration){0};
    }
    /* Bind every pointer before starting IRQ-driven operations. */
    host_uart.uart = &huart2; gyro_uart.uart = &huart6; debug_uart.uart = &huart5;
    motor_can.can = &hcan1;
    (void)UartDma_Init(&host_uart, &huart2);
    (void)UartDma_Init(&gyro_uart, &huart6);
    (void)UartDma_Init(&debug_uart, &huart5);
    if (CanLink_Init(&motor_can, &hcan1) != HAL_OK) Error_Handler();
    last_feedback = last_diag = last_status = last_parameter = HAL_GetTick();
    last_control = last_feedback;
    planner.initialized = false; preview_valid = false;
    initialized = true;
}

/* 获取上位机速度指令；超过 200ms 未更新则返回 false，并将输出速度置零。 */
bool Inspection_GetMotion(HostMotion *out) { return Host_GetMotion(&host, HAL_GetTick(), out); }

/* 将有效车体速度换算为四轮轮速和转角；这里只做基础计算，尚未限速或输出。 */
bool Inspection_CalculateWheels(const float reference_rad[SWERVE_WHEELS], SwerveTargets *out)
{
    static const SwerveGeometry geometry = {
        .wheel_radius_m = INSPECTION_WHEEL_RADIUS_M,
        .pivot_track_m = INSPECTION_PIVOT_TRACK_M,
        .pivot_wheelbase_m = INSPECTION_PIVOT_WHEELBASE_M
    };
    _Static_assert(INSPECTION_WHEELS == SWERVE_WHEELS, "Wheel count mismatch");
    HostMotion motion;
    if (!initialized || !Inspection_GetMotion(&motion)) return false;
    SwerveTwist twist = {motion.vx, motion.vy, motion.wz};
    return Swerve_Inverse(&geometry, &twist, reference_rad, out);
}

/* 根据四个电机读回的参数计算速度上限，取允许最高转速的 70%；参数未齐则失败。 */
bool Inspection_GetMotionLimits(MotionLimits *out)
{
    if (!initialized || !out) return false;
    MotionLimits result = {
        .yaw_speed_rad_s = INSPECTION_YAW_SPEED_RAD_S,
        .linear_accel_m_s2 = INSPECTION_LINEAR_ACCEL_M_S2,
        .yaw_accel_rad_s2 = INSPECTION_YAW_ACCEL_RAD_S2
    };
    const float physical = INSPECTION_MOTOR_NO_LOAD_RPM * (6.28318530718f / 60.0f);
    float minimum = physical;
    for (unsigned i = 0; i < INSPECTION_WHEELS; ++i) {
        const Dm3519 *m = &motors[i];
        if (m->parameters != 31U || m->control_mode != 3U ||
            !isfinite(m->max_speed) || m->max_speed <= 0.0f ||
            !isfinite(m->v_max) || m->v_max <= 0.0f) return false;
        float speed = INSPECTION_MOTOR_SPEED_FRACTION * fminf(physical, fminf(m->max_speed, m->v_max));
        result.wheel_speed_rad_s[i] = speed / INSPECTION_EXTERNAL_DRIVE_RATIO;
        result.wheel_accel_rad_s2[i] = INSPECTION_LINEAR_ACCEL_M_S2 / INSPECTION_WHEEL_RADIUS_M;
        result.steering_speed_rad_s[i] = INSPECTION_STEERING_SPEED_RAD_S;
        minimum = fminf(minimum, result.wheel_speed_rad_s[i]);
    }
    result.linear_speed_m_s = minimum * INSPECTION_WHEEL_RADIUS_M;
    /* Pure yaw must also fit all wheel limits. Mixed motion is saturated later. */
    float pivot_radius = 0.5f * hypotf(INSPECTION_PIVOT_TRACK_M, INSPECTION_PIVOT_WHEELBASE_M);
    result.yaw_speed_rad_s = fminf(result.yaw_speed_rad_s, result.linear_speed_m_s / pivot_radius);
    if (!MotionLimits_Valid(&result)) return false;
    *out = result;
    return true;
}

/* 读取最近的平滑运动计算结果；这是指令预览，不是实测车速或舵角。 */
bool Inspection_GetControlPreview(ChassisPlan *out)
{
    if (!out || !preview_valid ||
        (uint32_t)(HAL_GetTick() - planner.plan.timestamp_ms) >= INSPECTION_DEVICE_MAX_AGE_MS) return false;
    *out = planner.plan;
    return true;
}

/* 按 100Hz 更新连续运动规划；当前结果只保存在内存中，不交给执行器。 */
static void PreviewControl(uint32_t now)
{
    MotionLimits limits;
    if (!Inspection_GetMotionLimits(&limits)) {
        preview_valid = false; planner.initialized = false; return;
    }
    if (!planner.initialized) {
        ChassisPlannerConfig config = {
            .geometry = {INSPECTION_WHEEL_RADIUS_M,INSPECTION_PIVOT_TRACK_M,INSPECTION_PIVOT_WHEELBASE_M},
            .max_step_ms = INSPECTION_CONTROL_MAX_STEP_MS,
            .wheel_hold_rad_s = INSPECTION_WHEEL_HOLD_RAD_S
        };
        float reference[SWERVE_WHEELS] = {0};
        for (unsigned i = 0; i < SWERVE_WHEELS; ++i)
            config.steering[i] = (SwerveSteeringLimit){-INSPECTION_STEERING_LIMIT_RAD,INSPECTION_STEERING_LIMIT_RAD};
        if (!ChassisPlanner_Init(&planner, &config, reference, now)) { preview_valid = false; return; }
    }
    HostMotion motion;
    bool valid = Host_GetMotion(&host, now, &motion);
    SwerveTwist request = {motion.vx,motion.vy,motion.wz};
    preview_valid = ChassisPlanner_Step(&planner, &limits, &request, valid, now);
    if (!preview_valid) planner.initialized = false;
    /* Preview state is intentionally NOT passed to CAN/PWM or telemetry.
       Installed signs, physical stop/fault policy and enable path are pending. */
}

/* 生成临时舵角与脉宽映射：中位 1500us、范围 ±120°；direction 必须经实测确认。 */
bool Inspection_SteeringCalibration(int direction, ServoPwmCalibration *out)
{
    if (!out || (direction != 1 && direction != -1)) return false;
    /* 120/135 of each 1000us half-range; not a claim of measured calibration. */
    const uint16_t low = 611U, high = 2389U;
    *out = (ServoPwmCalibration){-INSPECTION_STEERING_LIMIT_RAD, INSPECTION_STEERING_LIMIT_RAD,
                               direction > 0 ? low : high, 1500U, direction > 0 ? high : low};
    return true;
}

/* 为指定轮保存舵机方向和临时映射，不启动 PWM；轮号 0~3 为左前、右前、左后、右后。 */
bool Inspection_ConfigureSteering(unsigned wheel, int direction)
{
    return wheel < INSPECTION_WHEELS && Inspection_SteeringCalibration(direction, &steering_calibration[wheel]);
}

/* 检查设备反馈及方向配置，生成四轮 CAN/PWM 数据；仅供预览，不发送。 */
bool Inspection_BuildOutputPreview(ChassisOutputBatch *out, ChassisOutputDiagnostics *diagnostics)
{
    ChassisPlan plan; MotionLimits limits;
    if (!out || !Inspection_GetControlPreview(&plan) || !Inspection_GetMotionLimits(&limits)) {
        if (diagnostics) *diagnostics = (ChassisOutputDiagnostics){.global_errors = CHASSIS_OUTPUT_ARGUMENT};
        return false;
    }
    ChassisOutputConfig config = {.plan_max_age_ms = INSPECTION_CONTROL_MAX_STEP_MS,
                                  .device_max_age_ms = INSPECTION_DEVICE_MAX_AGE_MS};
    for (unsigned i = 0; i < INSPECTION_WHEELS; ++i) {
        config.drive_sign[i] = wheel_feedback[i].sign;
        config.external_ratio[i] = wheel_feedback[i].ratio;
        config.steering[i] = steering_calibration[i];
    }
    ChassisOutputInput input = {.plan = &plan, .motors = motors, .limits = &limits,
                                .now_ms = HAL_GetTick(), .bus_off = CanLink_BusOff(&motor_can)};
    return ChassisOutput_Build(&config, &input, out, diagnostics);
}

/* 读取指定电机的原始状态；轮号越界返回 NULL，数据是否新鲜需另行检查。 */
const Dm3519 *Inspection_Motor(unsigned wheel)
{
    return wheel < INSPECTION_WHEELS ? &motors[wheel] : NULL;
}
/* 读取陀螺仪解析器状态；使用测量值时还需检查对应字段是否过期。 */
const Hwt101ct *Inspection_Gyro(void) { return &gyro; }

/* 保存驱动正反方向和外部传动比，供反馈与输出映射共用；当前直连车轮的传动比为 1。 */
bool Inspection_ConfigureWheelFeedback(unsigned wheel, int sign, float ratio)
{
    if (wheel >= INSPECTION_WHEELS || (sign != 1 && sign != -1) ||
        !isfinite(ratio) || ratio <= 0.0f) return false;
    wheel_feedback[wheel].sign = sign; wheel_feedback[wheel].ratio = ratio;
    return true;
}

/* 在主循环解析串口和 CAN 接收队列；发现丢数据就清除旧指令或反馈的有效标记。 */
static void Receive(void)
{
    UartRxBlock b;
    if (UartDma_TakeGap(&host_uart)) Host_OnGap(&host);
    if (UartDma_TakeGap(&gyro_uart)) Hwt_OnGap(&gyro);
    (void)UartDma_TakeGap(&debug_uart);
    /* Bound work per pass; an input flood must not starve timeout/telemetry. */
    for (unsigned i = 0; i < UART_RX_BLOCK_COUNT && UartDma_Read(&host_uart, &b); ++i)
        Host_Feed(&host, b.data, b.length, b.received_ms);
    for (unsigned i = 0; i < UART_RX_BLOCK_COUNT && UartDma_Read(&gyro_uart, &b); ++i)
        Hwt_Feed(&gyro, b.data, b.length, b.received_ms);
    /* Bluetooth RX is reserved; it cannot enable or command the chassis. */
    for (unsigned i = 0; i < UART_RX_BLOCK_COUNT && UartDma_Read(&debug_uart, &b); ++i) {}
    if (CanLink_TakeGap(&motor_can))
        for (unsigned i = 0; i < INSPECTION_WHEELS; ++i) motors[i].feedback_received = false;
    CanRxItem item;
    for (unsigned i = 0; i < CAN_RX_QUEUE_COUNT && CanLink_Read(&motor_can, &item); ++i) {
        for (unsigned j = 0; j < INSPECTION_WHEELS; ++j) {
            if (item.frame.id == motors[j].master_id) {
                (void)Dm3519_Accept(&motors[j], &item.frame, item.received_ms); break;
            }
        }
    }
}

/* 轮流查询四个电机的状态和参数，只读，不修改电机配置。 */
static void PollMotors(uint32_t now)
{
    DmCanFrame f;
    if ((uint32_t)(now - last_status) >= INSPECTION_CAN_POLL_INTERVAL_MS) {
        Dm3519_StatusRequest(&motors[poll_wheel], &f);
        if (CanLink_Send(&motor_can, &f)) {
            last_status = now; poll_wheel = (poll_wheel + 1U) % INSPECTION_WHEELS;
        }
    }
    /* Read-only round robin. No mode/ID/baud/FLASH write and no guessed ranges.
       One query per wheel every 80ms; a lost response is retried next sweep. */
    if ((uint32_t)(now - last_parameter) >= INSPECTION_PARAMETER_INTERVAL_MS) {
        static const uint8_t regs[] = {0x15, 0x16, 0x17, 0x0A, 0x06};
        Dm3519 *m = &motors[parameter_cursor % INSPECTION_WHEELS];
        uint8_t reg = regs[parameter_cursor / INSPECTION_WHEELS];
        Dm3519_ReadRegister(m, reg, &f);
        if (CanLink_Send(&motor_can, &f)) {
            Dm3519_MarkRequest(m, reg); last_parameter = now;
            parameter_cursor = (parameter_cursor + 1U) % (INSPECTION_WHEELS * sizeof(regs));
        }
    }
}

/* 每类反馈只保留最新一份，发送较慢时覆盖旧数据，避免积压。 */
static void Prepare(unsigned slot, uint8_t type, uint8_t length)
{
    if (pending[slot].ready) ++replaced;
    pending[slot].type = type; pending[slot].length = length;
    memset(pending[slot].payload, 0, sizeof(pending[slot].payload));
    pending[slot].ready = true;
}

/* 打包底盘与陀螺仪反馈，并按数据时效填写有效标记；未接入的舵角目标保持无效。 */
static void PrepareFeedback(uint32_t now)
{
    Prepare(0, HOST_MSG_CHASSIS, 40);
    uint8_t *b = pending[0].payload;
    WriteU32LE(b, now);
    uint16_t valid = 0, status = 0;
    HostMotion motion;
    if (!Host_GetMotion(&host, now, &motion)) status |= 1;
    if (CanLink_BusOff(&motor_can)) status |= 2;
    for (unsigned i = 0; i < INSPECTION_WHEELS; ++i) {
        if (Dm3519_FeedbackValid(&motors[i], now, INSPECTION_DEVICE_MAX_AGE_MS) &&
            wheel_feedback[i].sign && !CanLink_BusOff(&motor_can)) {
            float speed = motors[i].velocity_output_rad_s * wheel_feedback[i].sign / wheel_feedback[i].ratio;
            if (isfinite(speed)) { WriteF32LE(b + 4 + i * 4, speed); valid |= (uint16_t)(1U << i); }
        }
    }
    /* Steering targets stay invalid until kinematics/calibration are connected. */
    WriteU16LE(b + 36, valid); WriteU16LE(b + 38, status);

    Prepare(1, HOST_MSG_GYRO, 14); b = pending[1].payload;
    HwtSample s = Hwt_GetSample(&gyro, now, INSPECTION_DEVICE_MAX_AGE_MS);
    uint32_t stamp = 0;
    if (s.valid == 3) {
        /* Two separate sensor packets, not a guaranteed synchronous sample:
           publish the older RX time so neither field appears artificially new. */
        stamp = (uint32_t)(now - s.yaw_ms) > (uint32_t)(now - s.wz_ms) ? s.yaw_ms : s.wz_ms;
    } else if (s.valid & 1) stamp = s.yaw_ms;
    else if (s.valid & 2) stamp = s.wz_ms;
    WriteU32LE(b, stamp); WriteF32LE(b + 4, s.yaw_rad);
    WriteF32LE(b + 8, s.wz_rad_s); WriteU16LE(b + 12, s.valid);
}

/* 打包通讯错误计数，同时向蓝牙串口发送简短调试信息。 */
static void PrepareDiagnostics(uint32_t now)
{
    Prepare(2, HOST_MSG_DIAGNOSTICS, 20);
    uint8_t *b = pending[2].payload;
    WriteU32LE(b, now); WriteU32LE(b + 4, host.crc_errors);
    WriteU32LE(b + 8, host_uart.rx_overflows); WriteU32LE(b + 12, replaced);
    WriteU32LE(b + 16, host_uart.errors);
    char line[UART_TX_MAX_BYTES]; HostMotion m;
    bool active = Host_GetMotion(&host, now, &m);
    int n = snprintf(line, sizeof(line), "ms=%lu cmd=%lu timeout=%u gyro=%lu can=%lu err=%lu\r\n",
                     (unsigned long)now, (unsigned long)host.accepted_frames,
                     active ? 0U : 1U, (unsigned long)gyro.accepted_frames,
                     (unsigned long)(motors[0].feedback_count + motors[1].feedback_count +
                                     motors[2].feedback_count + motors[3].feedback_count),
                     (unsigned long)motor_can.errors);
    if (n > 0 && (size_t)n < sizeof(line))
        (void)UartDma_Transmit(&debug_uart, (uint8_t *)line, (size_t)n);
}

/* 轮流发送各类最新反馈；DMA 忙就留待下一次主循环，不阻塞等待。 */
static void Transmit(void)
{
    if (host_uart.tx_busy || host_uart.recovery) return;
    for (unsigned i = 0; i < 3; ++i) {
        unsigned slot = (next_telemetry + i) % 3U;
        Telemetry *t = &pending[slot];
        if (!t->ready) continue;
        uint8_t frame[HOST_MAX_FRAME];
        size_t length = Host_Encode(t->type, tx_sequence, t->payload, t->length, frame);
        if (UartDma_Transmit(&host_uart, frame, length)) {
            /* Sequence belongs to actual DMA submissions, not replaced snapshots. */
            ++tx_sequence; t->ready = false; next_telemetry = (slot + 1U) % 3U;
        }
        break;
    }
}

/* 主循环入口：处理收发、设备查询、100Hz 规划预览、50Hz 反馈和 1Hz 诊断。 */
void Inspection_Poll(void)
{
    if (!initialized) return;
    uint32_t now = HAL_GetTick();
    UartDma_Poll(&host_uart, now); UartDma_Poll(&gyro_uart, now); UartDma_Poll(&debug_uart, now);
    Receive();
    /* UART/CAN IRQs may have stamped a frame on the next millisecond while
       the receive queues were being drained. Age checks need a newer time
       than every parsed receive timestamp; an old snapshot wraps the age. */
    now = HAL_GetTick();
    PollMotors(now);
    if ((uint32_t)(now - last_control) >= INSPECTION_CONTROL_PERIOD_MS) {
        last_control = now; PreviewControl(now);
    }
    if ((uint32_t)(now - last_feedback) >= INSPECTION_FEEDBACK_PERIOD_MS) {
        last_feedback = now; PrepareFeedback(now);
    }
    if ((uint32_t)(now - last_diag) >= INSPECTION_DIAGNOSTICS_PERIOD_MS) {
        last_diag = now; PrepareDiagnostics(now);
    }
    Transmit();
}

/* 以下 HAL 回调只转交接收、发送完成和错误事件；协议解析留在主循环。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)size; UartDma *p = FindUart(uart); if (p) UartDma_Capture(p);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    UartDma *p = FindUart(uart); if (p) UartDma_OnTxComplete(p);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    UartDma *p = FindUart(uart); if (p) UartDma_OnError(p);
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *uart)
{
    UartDma *p = FindUart(uart); if (p) UartDma_OnAbortComplete(p);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    if (can == motor_can.can) CanLink_OnRx(&motor_can);
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    if (can == motor_can.can) CanLink_OnError(&motor_can);
}
