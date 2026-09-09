#include "host_command_rx.h"
#include <assert.h>
#include <string.h>

static void feed(HostCommandRx *rx, const char *text)
{
    while (*text) HostCommandRx_Feed(rx, (uint8_t)*text++);
}

int main(void)
{
    HostCommandRx rx;
    char command[HOST_COMMAND_LENGTH];
    unsigned int i;
    HostCommandRx_Reset(&rx);
    feed(&rx, "PING\r\nSTATUS\n");
    assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "PING") == 0);
    assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "STATUS") == 0);
    assert(!HostCommandRx_Pop(&rx, command));

    for (i = 0; i < HOST_COMMAND_QUEUE_CAPACITY + 1U; i++)
        feed(&rx, "POSE SET 100 0 0\n");
    assert(rx.dropped == 1U);
    feed(&rx, "STOP\r\nPOSE SET 200 0 0\n");
    assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "STOP") == 0);
    assert(!HostCommandRx_Pop(&rx, command));
    assert(rx.dropped == HOST_COMMAND_QUEUE_CAPACITY + 2U);
    feed(&rx, "POSE SET 300 0 0\n");
    assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "POSE SET 300 0 0") == 0);

    /* 超长命令的合法前缀及 NUL 前缀不得执行；下一完整 STOP 能恢复。 */
    feed(&rx, "STOP");
    for (i = 0; i < HOST_COMMAND_LENGTH; i++) feed(&rx, " ");
    feed(&rx, "\n");
    assert(!HostCommandRx_Pop(&rx, command));
    feed(&rx, "STOP");
    HostCommandRx_Feed(&rx, 0U);
    feed(&rx, "ignored\nSTOP\n");
    assert(rx.invalid_lines == 2U);
    assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "STOP") == 0);

    feed(&rx, "STOP\nPOSE SET ");
    assert(HostCommandRx_Pop(&rx, command));
    feed(&rx, "400 0 0\n");
    assert(!HostCommandRx_Pop(&rx, command));
    feed(&rx, "STOP\nSTOP\n");
    assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "STOP") == 0);
    assert(!HostCommandRx_Pop(&rx, command));

    /* 行长边界与循环队列回绕。 */
    for (i = 0; i < HOST_COMMAND_LENGTH - 1U; i++) feed(&rx, "A");
    feed(&rx, "\n");
    assert(HostCommandRx_Pop(&rx, command));
    assert(strlen(command) == HOST_COMMAND_LENGTH - 1U);
    for (i = 0; i < 20U; i++) {
        feed(&rx, "PING\n");
        assert(HostCommandRx_Pop(&rx, command) && strcmp(command, "PING") == 0);
    }
    return 0;
}
