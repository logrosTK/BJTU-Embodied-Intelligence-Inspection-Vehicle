#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t counter; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; uint32_t gState; unsigned tag; } UART_HandleTypeDef;
typedef struct { uint32_t ESR; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; uint32_t error; } CAN_HandleTypeDef;

typedef struct { uint32_t CR1, EGR, compare[4]; } TIM_TypeDef;
typedef struct { uint32_t Prescaler, Period; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
extern TIM_TypeDef test_tim3;
#define TIM3 (&test_tim3)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_CR1_UDIS 2U
#define TIM_EGR_UG 1U
void Test_SetCompare(TIM_HandleTypeDef *, uint32_t, uint32_t);
#define __HAL_TIM_SET_COMPARE(p,c,v) Test_SetCompare((p),(c),(v))
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
typedef struct {
    uint32_t FilterBank, FilterMode, FilterScale, FilterIdHigh, FilterIdLow;
    uint32_t FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment;
    uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC; } CAN_RxHeaderTypeDef;

#define CAN_FILTERMODE_IDLIST 1
#define CAN_FILTERSCALE_16BIT 0
#define CAN_FILTER_FIFO0 0
#define CAN_RX_FIFO0 0
#define ENABLE 1
#define DISABLE 0
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define CAN_ESR_BOFF 4U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_RX_FIFO0_OVERRUN 2U
#define CAN_IT_BUSOFF 4U
#define CAN_IT_ERROR 8U
#define HAL_CAN_ERROR_RX_FOV0 1U
#define HAL_CAN_ERROR_BOF 2U
#define HAL_UART_STATE_READY 1U
#define __HAL_DMA_GET_COUNTER(p) ((p)->counter)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
static inline void __DMB(void) { __asm__ volatile ("" ::: "memory"); }

uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
void Error_Handler(void);

#endif
