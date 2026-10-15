#include "uart_dma.h"
#include <string.h>

_Static_assert((UART_RX_BLOCK_COUNT & (UART_RX_BLOCK_COUNT - 1U)) == 0,
               "RX queue count must be a power of two");

static uint32_t Lock(void)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq(); __DMB(); return mask;
}
static void Unlock(uint32_t mask) { __DMB(); __set_PRIMASK(mask); }

/* 启动循环 DMA 接收；保留半满和全满中断，连续发送、没有空闲间隔时也能接收。 */
HAL_StatusTypeDef UartDma_Init(UartDma *p, UART_HandleTypeDef *uart)
{
    memset(p, 0, sizeof(*p)); p->uart = uart;
    HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(uart, p->dma_rx, UART_DMA_RX_BYTES);
    if (result != HAL_OK) {
        p->recovery = 1; p->rx_gap = 1; p->retry_ms = HAL_GetTick(); ++p->errors;
    }
    /* HT and TC intentionally stay enabled: continuous traffic may never IDLE. */
    return result;
}

/* 中断中按 DMA 实际位置提取新增字节，带接收时间入队；不在这里解析协议。 */
void UartDma_Capture(UartDma *p)
{
    if (p->recovery || p->rx_gap) return;
    /* Read the current hardware cursor, not callback Size. A delayed HT/TC/IDLE
       notification can otherwise copy stale bytes twice after another event. */
    uint16_t pos = (uint16_t)(UART_DMA_RX_BYTES - __HAL_DMA_GET_COUNTER(p->uart->hdmarx));
    pos %= UART_DMA_RX_BYTES;
    uint16_t old = p->dma_position;
    uint16_t count = (uint16_t)((pos + UART_DMA_RX_BYTES - old) % UART_DMA_RX_BYTES);
    if (!count) return;
    uint32_t needed = (count + UART_RX_BLOCK_BYTES - 1U) / UART_RX_BLOCK_BYTES;
    if ((uint32_t)(p->head - p->tail) + needed > UART_RX_BLOCK_COUNT) {
        ++p->rx_overflows; p->rx_gap = 1; p->dma_position = pos; return;
    }
    uint32_t rx_ms = HAL_GetTick();
    while (count) {
        UartRxBlock *b = &p->rx[p->head & (UART_RX_BLOCK_COUNT - 1U)];
        uint16_t n = count > UART_RX_BLOCK_BYTES ? UART_RX_BLOCK_BYTES : count;
        uint16_t first = UART_DMA_RX_BYTES - old;
        if (first > n) first = n;
        memcpy(b->data, p->dma_rx + old, first);
        if (n > first) memcpy(b->data + first, p->dma_rx, n - first);
        b->length = n; b->received_ms = rx_ms;
        __DMB(); ++p->head;
        old = (uint16_t)((old + n) % UART_DMA_RX_BYTES);
        count -= n;
    }
    p->dma_position = pos;
}

/* 取出并清除丢数据标记，同时丢弃不完整队列；返回 true 时上层必须重置解析状态。 */
bool UartDma_TakeGap(UartDma *p)
{
    uint32_t mask = Lock();
    bool gap = p->rx_gap != 0;
    if (gap) {
        p->tail = p->head; p->rx_gap = 0;
        if (!p->recovery)
            p->dma_position = (uint16_t)((UART_DMA_RX_BYTES - __HAL_DMA_GET_COUNTER(p->uart->hdmarx)) % UART_DMA_RX_BYTES);
    }
    Unlock(mask); return gap;
}

/* 主循环取出一块接收数据和原接收时间；队列为空或正在恢复时返回 false。 */
bool UartDma_Read(UartDma *p, UartRxBlock *out)
{
    if (p->rx_gap || p->recovery || p->tail == p->head) return false;
    __DMB();
    const UartRxBlock *b = &p->rx[p->tail & (UART_RX_BLOCK_COUNT - 1U)];
    out->length = b->length; out->received_ms = b->received_ms;
    memcpy(out->data, b->data, b->length);
    __DMB(); ++p->tail;
    /* If a loss happened while copying, the caller must not parse this block. */
    return p->rx_gap == 0 && p->recovery == 0;
}

/* 复制数据到专用缓冲后启动 DMA 发送；忙时返回 false，不等待，也不排长队。 */
bool UartDma_Transmit(UartDma *p, const uint8_t *data, size_t length)
{
    if (!length || length > UART_TX_MAX_BYTES || data == NULL) return false;
    uint32_t mask = Lock();
    if (p->recovery || p->tx_busy || p->uart->gState != HAL_UART_STATE_READY) {
        Unlock(mask); return false;
    }
    memcpy(p->tx_active, data, length);
    p->tx_busy = 1;
    HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(p->uart, p->tx_active, (uint16_t)length);
    if (result != HAL_OK) {
        p->tx_busy = 0; ++p->errors;
        p->rx_gap = 1; p->recovery = 1; p->retry_ms = HAL_GetTick();
    }
    Unlock(mask); return result == HAL_OK;
}

/* 发送完成后才释放 DMA 发送缓冲，防止发送过程中被新数据覆盖。 */
void UartDma_OnTxComplete(UartDma *p) { p->tx_busy = 0; }
/* 串口出错先标记数据中断，恢复操作交给主循环。 */
void UartDma_OnError(UartDma *p)
{
    ++p->errors; p->rx_gap = 1;
    if (p->recovery == 0) { p->recovery = 1; p->retry_ms = HAL_GetTick(); }
}
/* 异步终止完成后，允许主循环重新启动接收。 */
void UartDma_OnAbortComplete(UartDma *p)
{
    p->tx_busy = 0; p->recovery = 3;
}

/* 主循环推进串口错误恢复：异步终止后重启 DMA，失败则稍后重试。 */
void UartDma_Poll(UartDma *p, uint32_t now_ms)
{
    if (!p->recovery) return;
    if ((int32_t)(now_ms - p->retry_ms) < 0) return;
    if (p->recovery == 1) {
        p->recovery = 2;
        /* Asynchronous abort: an RX line error must not block the main loop. */
        if (HAL_UART_Abort_IT(p->uart) != HAL_OK) {
            p->recovery = 1; p->retry_ms = now_ms + 10U;
        }
    }
    if (p->recovery == 3) {
        uint32_t mask = Lock();
        p->tail = p->head; p->dma_position = 0;
        HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(p->uart, p->dma_rx, UART_DMA_RX_BYTES);
        if (result == HAL_OK) p->recovery = 0;
        else { ++p->errors; p->recovery = 1; p->retry_ms = now_ms + 10U; }
        Unlock(mask);
    }
}
