#include "host_rx_router.h"
#include <assert.h>
#include <string.h>

static void text(HostRxRouter *rx, const char *value, uint8_t active)
{
    while (*value) assert(!HostRx_Feed(rx, (uint8_t)*value++, active));
}
static void binary(HostRxRouter *rx, uint8_t command, uint8_t sequence, uint8_t corrupt)
{
    uint8_t bytes[RPI_PROTOCOL_MAX_FRAME];
    uint16_t i, length = RpiProtocol_Encode(RPI_MSG_COMMAND, sequence, &command, 1,
                                            bytes, sizeof(bytes));
    unsigned complete = 0;
    if (corrupt) bytes[length - 1] ^= 1;
    for (i = 0; i < length; ++i) complete += HostRx_Feed(rx, bytes[i], 0);
    assert(complete == (corrupt ? 0U : 1U));
}
int main(void)
{
    HostRxRouter rx;
    char command[HOST_COMMAND_LENGTH];
    RpiFrame frame;
    unsigned i;
    HostRx_Init(&rx);
    text(&rx, "HOST LINK COM\r\n", 0);
    assert(HostRx_PopText(&rx, command) && !strcmp(command, "HOST LINK COM"));
    /* STOP must survive a full ordinary text queue. */
    for (i = 0; i < 8; ++i) text(&rx, "PING\n", 0);
    text(&rx, "STOP\nPOSE SET 1 2 3\n", 0);
    assert(HostRx_PopText(&rx, command) && !strcmp(command, "STOP"));
    assert(!HostRx_PopText(&rx, command));
    binary(&rx, RPI_CMD_PING, 1, 0);
    assert(HostRx_PopFrame(&rx, &frame) && frame.sequence == 1);
    for (i = 0; i < 8; ++i) binary(&rx, RPI_CMD_PING, (uint8_t)i, 0);
    binary(&rx, RPI_CMD_STOP_ALL, 42, 0);
    binary(&rx, RPI_CMD_PING, 43, 0);
    assert(HostRx_PopFrame(&rx, &frame) && frame.sequence == 42);
    assert(!HostRx_PopFrame(&rx, &frame));
    assert(HostRx_GetStats(&rx).binary_dropped > 0);
    assert(HostRx_GetStats(&rx).text_dropped > 0);
    /* CRC rejection can return to ASCII when no binary session owns the UART. */
    binary(&rx, RPI_CMD_PING, 44, 1);
    assert(!HostRx_PopFrame(&rx, &frame));
    assert(HostRx_GetStats(&rx).crc_errors == 1);
    text(&rx, "PING\n", 0);
    assert(HostRx_PopText(&rx, command) && !strcmp(command, "PING"));
    text(&rx, "STOP\n", 1);
    assert(!HostRx_PopText(&rx, command)); /* No ASCII side effects in binary session. */
    HostRx_ResetParser(&rx);
    text(&rx, "PING\n", 0);
    assert(HostRx_PopText(&rx, command));
    /* A new session/error reset cannot dispatch an old queued target. */
    binary(&rx, RPI_CMD_PING, 45, 0);
    HostRx_ResetFrames(&rx); HostRx_ResetParser(&rx); HostRx_ResetText(&rx);
    assert(!HostRx_PopFrame(&rx, &frame) && !HostRx_PopText(&rx, command));
    HostRx_Init(&rx);
    assert(HostRx_GetStats(&rx).crc_errors == 0);
    return 0;
}
