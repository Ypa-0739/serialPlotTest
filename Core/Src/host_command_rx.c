#include "host_command_rx.h"
#include <string.h>

void HostCommandRx_Reset(HostCommandRx *rx)
{
    memset(rx, 0, sizeof(*rx));
}

void HostCommandRx_Feed(HostCommandRx *rx, uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (rx->invalid_line) {
            rx->invalid_lines++;
        } else if (rx->length != 0U) {
            rx->line[rx->length] = '\0';
            if (strcmp(rx->line, "STOP") == 0) {
                /* 满队列也不能吞 STOP；同时撤销停车前未执行命令。 */
                rx->dropped += rx->count;
                rx->head = rx->tail = rx->count = 0U;
                rx->stop_pending = 1U;
            } else if (rx->stop_pending || rx->count == HOST_COMMAND_QUEUE_CAPACITY) {
                rx->dropped++;
            } else {
                memcpy(rx->commands[rx->head], rx->line, rx->length + 1U);
                rx->head = (uint8_t)((rx->head + 1U) % HOST_COMMAND_QUEUE_CAPACITY);
                rx->count++;
            }
        }
        rx->length = 0U;
        rx->invalid_line = 0U;
        return;
    }
    /* 超长或含控制字符的行整行丢弃，绝不执行截断前缀。 */
    if (rx->invalid_line) return;
    if ((byte < 0x20U && byte != '\t') || byte > 0x7EU ||
        rx->length >= HOST_COMMAND_LENGTH - 1U) {
        rx->invalid_line = 1U;
        return;
    }
    rx->line[rx->length++] = (char)byte;
}

uint8_t HostCommandRx_Pop(HostCommandRx *rx, char output[HOST_COMMAND_LENGTH])
{
    if (rx->stop_pending) {
        rx->stop_pending = 0U;
        /* STOP 处理前已经开始接收的半行也属于旧批次。 */
        if (rx->length != 0U) rx->invalid_line = 1U;
        memcpy(output, "STOP", 5U);
        return 1U;
    }
    if (rx->count == 0U) return 0U;
    memcpy(output, rx->commands[rx->tail], HOST_COMMAND_LENGTH);
    rx->tail = (uint8_t)((rx->tail + 1U) % HOST_COMMAND_QUEUE_CAPACITY);
    rx->count--;
    return 1U;
}
