#ifndef USER_UART_DMA_H
#define USER_UART_DMA_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>

#define UART_DMA_RX_BYTES 1024U
#define UART_RX_BLOCK_BYTES 256U
#define UART_RX_BLOCK_COUNT 16U
#define UART_TX_MAX_BYTES 138U

typedef struct {
    uint8_t data[UART_RX_BLOCK_BYTES];
    uint16_t length;
    uint32_t received_ms;
} UartRxBlock;

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t dma_rx[UART_DMA_RX_BYTES];
    uint8_t tx_active[UART_TX_MAX_BYTES];
    UartRxBlock rx[UART_RX_BLOCK_COUNT];
    volatile uint32_t head, tail;
    volatile uint8_t rx_gap, recovery, tx_busy;
    uint16_t dma_position;
    uint32_t retry_ms;
    volatile uint32_t rx_overflows, errors;
} UartDma;

/* All ordinary APIs are main-loop only. DMA/UART IRQs for a port must have
   equal preemption priority so Capture cannot reenter itself.
   Keep IRQ latency below half the DMA buffer time (~44ms at 115200/8N1). */
HAL_StatusTypeDef UartDma_Init(UartDma *p, UART_HandleTypeDef *uart);
void UartDma_Poll(UartDma *p, uint32_t now_ms);
bool UartDma_TakeGap(UartDma *p);
bool UartDma_Read(UartDma *p, UartRxBlock *out);
bool UartDma_Transmit(UartDma *p, const uint8_t *data, size_t length);

/* Forward HAL callbacks here; no decoding or transmit waits in IRQs. */
void UartDma_Capture(UartDma *p);
void UartDma_OnTxComplete(UartDma *p);
void UartDma_OnError(UartDma *p);
void UartDma_OnAbortComplete(UartDma *p);

#endif
