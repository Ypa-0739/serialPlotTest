#include "host_rx_router.h"

static void OnFrame(const RpiFrame *frame, void *context)
{
    HostRxRouter *rx = context;
    uint8_t urgent = frame->message_type == RPI_MSG_COMMAND &&
                     frame->payload_length == 1U && frame->payload[0] == RPI_CMD_STOP_ALL;
    (void)RpiProtocol_QueuePush(&rx->frames, frame, urgent);
    rx->binary_candidate = 1U;
}

void HostRx_Init(HostRxRouter *rx)
{
    HostCommandRx_Reset(&rx->text);
    RpiProtocol_QueueReset(&rx->frames);
    RpiProtocol_ParserInit(&rx->parser, OnFrame, rx);
    rx->binary_candidate = 0U;
}

uint8_t HostRx_Feed(HostRxRouter *rx, uint8_t byte, uint8_t binary_active)
{
    uint32_t valid_before = rx->parser.valid_frames;
    if (binary_active || rx->binary_candidate ||
        (rx->text.length == 0U && !rx->text.invalid_line && byte == RPI_PROTOCOL_SOF_1)) {
        rx->binary_candidate = 1U;
        RpiProtocol_FeedByte(&rx->parser, byte);
        if (!binary_active && rx->parser.valid_frames == valid_before &&
            rx->parser.state == RPI_PARSE_SOF_1) rx->binary_candidate = 0U;
    } else {
        HostCommandRx_Feed(&rx->text, byte);
    }
    return rx->parser.valid_frames != valid_before;
}

uint8_t HostRx_PopText(HostRxRouter *rx, char *command)
{ return HostCommandRx_Pop(&rx->text, command); }
uint8_t HostRx_PopFrame(HostRxRouter *rx, RpiFrame *frame)
{ return RpiProtocol_QueuePop(&rx->frames, frame); }
void HostRx_ResetText(HostRxRouter *rx) { HostCommandRx_Reset(&rx->text); }
void HostRx_ResetFrames(HostRxRouter *rx) { RpiProtocol_QueueReset(&rx->frames); }
void HostRx_ResetParser(HostRxRouter *rx)
{
    rx->binary_candidate = 0U;
    RpiProtocol_ParserReset(&rx->parser);
}
HostRxStats HostRx_GetStats(const HostRxRouter *rx)
{
    HostRxStats stats = {rx->text.dropped, rx->text.invalid_lines,
                        rx->frames.dropped, rx->parser.crc_errors};
    return stats;
}
