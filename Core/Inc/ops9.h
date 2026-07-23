/*
 * ops9.h
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */

#ifndef INC_OPS9_H_
#define INC_OPS9_H_

#include "main.h"

/*
 * 车辆位姿直接使用 OPS-9 原生坐标，不交换坐标轴，也不改变正负号：
 * robot_x/robot_y 单位为 mm，robot_yaw 为 OPS 输出的 Z 轴角度（degree）。
 * 这些变量由 USART2 中断更新，因此必须保留 volatile。
 */
extern volatile float robot_x;
extern volatile float robot_y;
extern volatile float robot_yaw;
extern volatile uint32_t ops9_frame_count;
extern volatile uint32_t ops9_invalid_frame_count;
extern volatile uint32_t ops9_rx_byte_count;
extern volatile uint32_t ops9_header_count;
extern volatile uint32_t ops9_format_error_count;
extern volatile uint32_t ops9_uart_error_count;
extern volatile uint32_t ops9_last_update_tick;
extern volatile uint32_t ops9_last_byte_tick;
extern volatile uint8_t ops9_last_raw_byte;

// OPS-9 串口接收中断处理函数声明
void OPS9_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void OPS9_Reset_Zero(void);


#endif /* INC_OPS9_H_ */
