#include "can_link.h"
#include "inspection_config.h"
#include <string.h>

_Static_assert((CAN_RX_QUEUE_COUNT & (CAN_RX_QUEUE_COUNT - 1U)) == 0,
               "CAN queue count must be a power of two");

/* 启动 CAN 接收，只放行四个 Master ID 的标准数据帧。 */
HAL_StatusTypeDef CanLink_Init(CanLink *p, CAN_HandleTypeDef *can)
{
    memset(p, 0, sizeof(*p)); p->can = can;
    /* Accept configured Master IDs; never accept extended or remote frames. */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0; filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = INSPECTION_MASTER_BASE << 5;
    filter.FilterIdLow = (INSPECTION_MASTER_BASE + 1U) << 5;
    filter.FilterMaskIdHigh = (INSPECTION_MASTER_BASE + 2U) << 5;
    filter.FilterMaskIdLow = (INSPECTION_MASTER_BASE + 3U) << 5;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE; filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK || HAL_CAN_Start(can) != HAL_OK)
        return HAL_ERROR;
    return HAL_CAN_ActivateNotification(can, CAN_IT_RX_FIFO0_MSG_PENDING |
             CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR);
}

/* 检查 CAN 是否因总线错误进入离线状态。 */
bool CanLink_BusOff(const CanLink *p) { return (p->can->Instance->ESR & CAN_ESR_BOFF) != 0; }

/* 有空闲邮箱才提交发送；总线离线或邮箱满时立即返回 false。 */
bool CanLink_Send(CanLink *p, const DmCanFrame *f)
{
    if (f->id > 0x7FF || f->length > 8 || CanLink_BusOff(p) ||
        HAL_CAN_GetTxMailboxesFreeLevel(p->can) == 0) return false;
    CAN_TxHeaderTypeDef header = {0}; uint32_t mailbox;
    header.StdId = f->id; header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA;
    header.DLC = f->length; header.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(p->can, &header, (uint8_t *)f->data, &mailbox) != HAL_OK) {
        ++p->errors; return false;
    }
    return true;
}

/* 接收中断最多搬运三帧到软件队列，记录接收时间，不在中断内解析电机协议。 */
void CanLink_OnRx(CanLink *p)
{
    /* bxCAN FIFO has three entries. Bound ISR work even if traffic continues. */
    for (unsigned i = 0; i < 3 && HAL_CAN_GetRxFifoFillLevel(p->can, CAN_RX_FIFO0); ++i) {
        CAN_RxHeaderTypeDef h; uint8_t data[8];
        if (HAL_CAN_GetRxMessage(p->can, CAN_RX_FIFO0, &h, data) != HAL_OK) {
            ++p->errors; p->gap = 1; break;
        }
        if (h.IDE != CAN_ID_STD || h.RTR != CAN_RTR_DATA || h.DLC != 8) continue;
        if ((uint32_t)(p->head - p->tail) >= CAN_RX_QUEUE_COUNT) {
            ++p->overflows; p->gap = 1; continue;
        }
        CanRxItem *item = &p->queue[p->head & (CAN_RX_QUEUE_COUNT - 1U)];
        item->frame.id = (uint16_t)h.StdId; item->frame.length = 8;
        memcpy(item->frame.data, data, 8); item->received_ms = HAL_GetTick();
        __DMB(); ++p->head;
    }
}

/* 主循环按顺序取出一帧 CAN 数据；队列为空或发生丢帧时返回 false。 */
bool CanLink_Read(CanLink *p, CanRxItem *out)
{
    if (p->gap || p->head == p->tail) return false;
    __DMB(); *out = p->queue[p->tail & (CAN_RX_QUEUE_COUNT - 1U)];
    __DMB(); ++p->tail; return p->gap == 0;
}
/* 取出丢帧标记并清空旧队列，上层据此使电机反馈失效。 */
bool CanLink_TakeGap(CanLink *p)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq(); __DMB();
    bool gap = p->gap != 0;
    if (gap) { p->tail = p->head; p->gap = 0; }
    __DMB(); __set_PRIMASK(mask); return gap;
}
/* 记录 CAN 错误；接收溢出或总线离线时，通知上层数据已不连续。 */
void CanLink_OnError(CanLink *p)
{
    ++p->errors;
    if (HAL_CAN_GetError(p->can) & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_BOF)) p->gap = 1;
}
