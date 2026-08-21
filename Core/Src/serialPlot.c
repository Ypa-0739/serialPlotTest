#include "serialPlot.h"
#include "usart.h"
#include <string.h>

uint8_t SerialPlot_SendFloats(const float *data, uint8_t num_channels)
{
    uint8_t tx_buffer[2U + SERIAL_PLOT_MAX_CHANNELS * sizeof(float)];
    uint16_t total_len;

    if (data == NULL || num_channels == 0U ||
        num_channels > SERIAL_PLOT_MAX_CHANNELS) {
        return 0U;
    }

    tx_buffer[0] = SERIAL_PLOT_SYNC_BYTE_0;
    tx_buffer[1] = SERIAL_PLOT_SYNC_BYTE_1;
    memcpy(&tx_buffer[2], data, (size_t)num_channels * sizeof(float));
    total_len = 2U + (uint16_t)num_channels * (uint16_t)sizeof(float);

    /*
     * 原实现把局部栈数组交给 DMA 后立即返回，DMA 可能继续读取
     * 已失效的内存；串口忙时还会静默丢帧。绘图数据只有 20 Hz，
     * 这里用阻塞发送保证整帧生命周期和完整性，不会长时间占用串口。
     */
    return (HAL_UART_Transmit(&huart1, tx_buffer, total_len, 20U) == HAL_OK) ? 1U : 0U;
}
