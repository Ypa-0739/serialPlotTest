#ifndef HOST_RX_ROUTER_H
#define HOST_RX_ROUTER_H
#include "host_command_rx.h"
#include "rpi_protocol.h"

/* 纯协议对象，不依赖 HAL。ISR 写入、主循环读取时由调用方提供临界区。 */
typedef struct {
    HostCommandRx text;
    RpiProtocolParser parser;
    RpiFrameQueue frames;
    uint8_t binary_candidate;
} HostRxRouter;

typedef struct {
    uint32_t text_dropped, text_invalid, binary_dropped, crc_errors;
} HostRxStats;

void HostRx_Init(HostRxRouter *rx);
/* 返回本字节是否完成一个合法二进制帧，供应用更新会话心跳。 */
uint8_t HostRx_Feed(HostRxRouter *rx, uint8_t byte, uint8_t binary_active);
uint8_t HostRx_PopText(HostRxRouter *rx, char *command);
uint8_t HostRx_PopFrame(HostRxRouter *rx, RpiFrame *frame);
void HostRx_ResetText(HostRxRouter *rx);
void HostRx_ResetFrames(HostRxRouter *rx);
void HostRx_ResetParser(HostRxRouter *rx);
HostRxStats HostRx_GetStats(const HostRxRouter *rx);
#endif
