#ifndef INC_SERIALPLOT_H_
#define INC_SERIALPLOT_H_

#include <stdint.h>

/* SerialPlot Framed 模式的同步字，后面紧跟 little-endian float 通道。 */
#define SERIAL_PLOT_SYNC_BYTE_0       0xAAU
#define SERIAL_PLOT_SYNC_BYTE_1       0xBBU
#define SERIAL_PLOT_MAX_CHANNELS      8U

/*
 * 向 USART1 发送一帧 SerialPlot 数据。
 * 返回 1 表示发送成功，0 表示参数非法或串口发送失败。
 */
uint8_t SerialPlot_SendFloats(const float *data, uint8_t num_channels);


#endif /* INC_SERIALPLOT_H_ */
