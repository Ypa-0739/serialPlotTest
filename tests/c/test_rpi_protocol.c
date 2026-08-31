#include "rpi_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static RpiFrame received;
static uint8_t callback_count;

static void on_frame(const RpiFrame *frame, void *context)
{
    (void)context;
    received = *frame;
    ++callback_count;
}

int main(void)
{
    static const uint8_t vector[] = "123456789";
    uint8_t payload[] = {0x80U, 0x2AU, 0x00U, 0x00U, 0x00U};
    uint8_t wire[RPI_PROTOCOL_MAX_FRAME];
    uint16_t length;
    uint16_t index;
    RpiProtocolParser parser;

    assert(RpiProtocol_Crc16(vector, 9U) == 0x29B1U);
    length = RpiProtocol_Encode(RPI_MSG_COMMAND, 7U, payload, sizeof(payload),
                                wire, sizeof(wire));
    assert(length == 14U);

    RpiProtocol_ParserInit(&parser, on_frame, NULL);
    RpiProtocol_FeedByte(&parser, 0x00U);
    for (index = 0U; index < length; ++index) RpiProtocol_FeedByte(&parser, wire[index]);
    assert(callback_count == 1U);
    assert(received.message_type == RPI_MSG_COMMAND);
    assert(received.sequence == 7U);
    assert(received.payload_length == sizeof(payload));
    assert(memcmp(received.payload, payload, sizeof(payload)) == 0);

    wire[length - 1U] ^= 0x80U;
    for (index = 0U; index < length; ++index) RpiProtocol_FeedByte(&parser, wire[index]);
    assert(callback_count == 1U);
    assert(parser.crc_errors == 1U);
    return 0;
}
