#ifndef USER_CAN_LINK_H
#define USER_CAN_LINK_H

#include "stm32f4xx_hal.h"
#include "dm3519.h"

#define CAN_RX_QUEUE_COUNT 32U
typedef struct { DmCanFrame frame; uint32_t received_ms; } CanRxItem;
typedef struct {
    CAN_HandleTypeDef *can;
    CanRxItem queue[CAN_RX_QUEUE_COUNT];
    volatile uint32_t head, tail, overflows, errors;
    volatile uint8_t gap;
} CanLink;

HAL_StatusTypeDef CanLink_Init(CanLink *p, CAN_HandleTypeDef *can);
bool CanLink_Send(CanLink *p, const DmCanFrame *f);
bool CanLink_Read(CanLink *p, CanRxItem *out);
bool CanLink_TakeGap(CanLink *p);
void CanLink_OnRx(CanLink *p);
void CanLink_OnError(CanLink *p);
bool CanLink_BusOff(const CanLink *p);

#endif
