#include "host_uart_tx.h"
#ifdef CONTROL_HOST_TEST
#include "control_test_hal.h"
#else
#include "usart.h"
#endif
#include <string.h>

#define TX_LINE_SIZE 512U
#define TX_QUEUE_SIZE 32U
#define TX_URGENT_SIZE 4U
typedef struct { uint16_t length; uint8_t bytes[TX_LINE_SIZE]; } TxLine;
static TxLine queue[TX_QUEUE_SIZE], urgent[TX_URGENT_SIZE], telemetry[3], assembling, active;
static uint8_t head, tail, count, urgent_head, urgent_tail, urgent_count;
static uint8_t telemetry_ready, telemetry_next, invalid_line;
static volatile uint8_t busy;
static uint8_t fault;
static uint32_t started;
static HostUartTxStats stats;

void HostUartTx_DiscardPending(void)
{
    head = tail = count = urgent_head = urgent_tail = urgent_count = 0U;
    telemetry_ready = telemetry_next = invalid_line = 0U;
    assembling.length = 0U;
}

void HostUartTx_Reset(void)
{
    /* 调用者先终止 DMA；绝不能在 DMA 持有 active 时覆盖它。 */
    busy = 0U;
    HostUartTx_DiscardPending();
    fault = 0U;
}

int HostUartTx_Write(const char *text, int length)
{
    int i;
    if (text == NULL || length <= 0) return -1;
    for (i = 0; i < length; ++i) {
        if (assembling.length < TX_LINE_SIZE - 1U && !invalid_line)
            assembling.bytes[assembling.length++] = (uint8_t)text[i];
        else invalid_line = 1U;
        if (text[i] != '\n') continue;
        assembling.bytes[assembling.length] = 0U;
        if (invalid_line) { stats.dropped++; fault = 1U; }
        else if (assembling.bytes[0] == '@' ||
                 (assembling.bytes[0] >= '0' && assembling.bytes[0] <= '9')) {
            uint8_t group = assembling.bytes[0] != '@' ? 2U :
                            (assembling.bytes[1] == 'W' ? 0U : 1U);
            if (telemetry_ready & (1U << group)) stats.replaced++;
            telemetry[group] = assembling;
            telemetry_ready |= (uint8_t)(1U << group);
        } else if (strncmp((char *)assembling.bytes, "# STOP ", 7U) == 0 ||
                   strncmp((char *)assembling.bytes, "# POSE STOP", 11U) == 0 ||
                   strncmp((char *)assembling.bytes, "# ROUND START", 13U) == 0 ||
                   strncmp((char *)assembling.bytes, "# ROUND STOP", 12U) == 0 ||
                   strncmp((char *)assembling.bytes, "# CAN FEEDBACK LOST", 19U) == 0 ||
                   strncmp((char *)assembling.bytes, "# MOTION STOP", 13U) == 0 ||
                   strncmp((char *)assembling.bytes, "# MOTOR STOP ", 13U) == 0 ||
                   strncmp((char *)assembling.bytes, "# MOVE STOP", 11U) == 0) {
            if (urgent_count < TX_URGENT_SIZE) {
                urgent[urgent_head] = assembling;
                urgent_head = (uint8_t)((urgent_head + 1U) % TX_URGENT_SIZE);
                urgent_count++;
            } else { stats.dropped++; fault = 1U; }
        } else if (count < TX_QUEUE_SIZE) {
            queue[head] = assembling;
            head = (uint8_t)((head + 1U) % TX_QUEUE_SIZE);
            count++;
        } else { stats.dropped++; fault = 1U; }
        assembling.length = 0U;
        invalid_line = 0U;
    }
    return length;
}

void HostUartTx_Process(uint32_t now)
{
    HAL_StatusTypeDef result;
    uint32_t primask;
    if (busy) {
        uint32_t age = now - started;
        if (age > stats.max_busy_ms) stats.max_busy_ms = age;
        if (age > 100U) fault = 1U;
        return;
    }
    if (!urgent_count && !count && !telemetry_ready) return;
    if (huart1.gState != HAL_UART_STATE_READY) return;
    if (urgent_count) {
        active = urgent[urgent_tail];
        urgent_tail = (uint8_t)((urgent_tail + 1U) % TX_URGENT_SIZE);
        urgent_count--;
    }
    else if (count) {
        active = queue[tail]; tail = (uint8_t)((tail + 1U) % TX_QUEUE_SIZE); count--;
    } else {
        while (!(telemetry_ready & (1U << telemetry_next)))
            telemetry_next = (uint8_t)((telemetry_next + 1U) % 3U);
        active = telemetry[telemetry_next];
        telemetry_ready &= (uint8_t)~(1U << telemetry_next);
        telemetry_next = (uint8_t)((telemetry_next + 1U) % 3U);
    }
    /* 标志先于 DMA 启动，避免完成中断早于 busy 赋值；临界区无等待。 */
    primask = __get_PRIMASK();
    __disable_irq();
    started = now;
    busy = 1U;
    result = HAL_UART_Transmit_DMA(&huart1, active.bytes, active.length);
    if (result != HAL_OK) { busy = 0U; fault = 1U; stats.dropped++; }
    __set_PRIMASK(primask);
}

void HostUartTx_Complete(void) { busy = 0U; stats.completed++; }
uint8_t HostUartTx_Busy(void) { return busy; }
uint8_t HostUartTx_ConsumeFault(void) { uint8_t value = fault; fault = 0U; return value; }
HostUartTxStats HostUartTx_GetStats(void) { return stats; }
