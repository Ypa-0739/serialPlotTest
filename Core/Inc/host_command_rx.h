#ifndef HOST_COMMAND_RX_H
#define HOST_COMMAND_RX_H

#include <stdint.h>

#define HOST_COMMAND_LENGTH 64U
#define HOST_COMMAND_QUEUE_CAPACITY 4U

/* ISR 是唯一生产者；主循环 Pop/Reset 时须保存并恢复中断屏蔽状态。 */
typedef struct {
    char line[HOST_COMMAND_LENGTH];
    char commands[HOST_COMMAND_QUEUE_CAPACITY][HOST_COMMAND_LENGTH];
    uint8_t length;
    uint8_t invalid_line;
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t stop_pending;
    uint32_t dropped;
    uint32_t invalid_lines;
} HostCommandRx;

void HostCommandRx_Reset(HostCommandRx *rx);
void HostCommandRx_Feed(HostCommandRx *rx, uint8_t byte);
uint8_t HostCommandRx_Pop(HostCommandRx *rx, char output[HOST_COMMAND_LENGTH]);

#endif
