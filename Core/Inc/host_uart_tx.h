#ifndef HOST_UART_TX_H
#define HOST_UART_TX_H
#include <stdint.h>

typedef struct {
    uint32_t completed, dropped, replaced, max_busy_ms;
} HostUartTxStats;
int HostUartTx_Write(const char *text, int length);
void HostUartTx_Process(uint32_t now);
void HostUartTx_Complete(void);
void HostUartTx_Reset(void);
void HostUartTx_DiscardPending(void);
uint8_t HostUartTx_Busy(void);
uint8_t HostUartTx_ConsumeFault(void);
HostUartTxStats HostUartTx_GetStats(void);
#endif
