/*
 * ops9.c
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */
#include "ops9.h"
#ifndef CONTROL_HOST_TEST
#include "usart.h" // 需要用到 huart2
#endif
#include <math.h>

/* OPS-9 每帧数据区包含 6 个小端 IEEE-754 float，下面是手册规定的索引。 */
#define OPS9_INDEX_Z_ANGLE   0U
#define OPS9_INDEX_POS_X     3U
#define OPS9_INDEX_POS_Y     4U

/* 车辆坐标与 OPS-9 原生坐标保持完全一致。 */
volatile float robot_x = 0.0f;   // OPS X，单位 mm
volatile float robot_y = 0.0f;   // OPS Y，单位 mm
volatile float robot_yaw = 0.0f; // OPS Z 轴角度，单位 degree
volatile uint32_t ops9_frame_count = 0U;
volatile uint32_t ops9_invalid_frame_count = 0U;
volatile uint32_t ops9_rx_byte_count = 0U;
volatile uint32_t ops9_header_count = 0U;
volatile uint32_t ops9_format_error_count = 0U;
volatile uint32_t ops9_uart_error_count = 0U;
volatile uint32_t ops9_last_update_tick = 0U;
volatile uint32_t ops9_last_byte_tick = 0U;
volatile uint8_t ops9_last_raw_byte = 0U;

// 用于 HAL 库单字节接收的缓存
uint8_t ops9_rx_byte;
static uint8_t count = 0; // 状态机步骤计数
static uint8_t i = 0;     // 数据数组索引

OPS9_Snapshot OPS9_GetSnapshot(void)
{
    OPS9_Snapshot snapshot;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot.x_mm = robot_x;
    snapshot.y_mm = robot_y;
    snapshot.yaw_deg = robot_yaw;
    snapshot.frame_count = ops9_frame_count;
    snapshot.last_update_tick = ops9_last_update_tick;
    __set_PRIMASK(primask);
    return snapshot;
}


// 利用共用体直接将24个字节转换为6个浮点数
static union {
    uint8_t data[24];
    float ActVal[6];
} posture;

/*
 * @brief OPS-9 串口单字节接收回调函数
 * @note  请在 HAL_UART_RxCpltCallback 中调用此函数
 */
void OPS9_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // 确认是 OPS-9 所在的串口2
    {

    	uint8_t ch = ops9_rx_byte;

        /* 第一层诊断：只要 USART2 收到过电平正确的字节，这些计数就会增长。 */
        ops9_rx_byte_count++;
        ops9_last_raw_byte = ch;
        ops9_last_byte_tick = HAL_GetTick();

        // 状态机解析 (参考官方手册附录)
        switch (count)
        {
            case 0: // 等待帧头第一个字节 0x0D
                if (ch == 0x0D)
                    count++;
                else
                    count = 0;
                break;

            case 1: // 等待帧头第二个字节 0x0A
                if (ch == 0x0A) {
                    i = 0;
                    ops9_header_count++;
                    count++;
                } else if (ch == 0x0D) {
                    ; // 保持状态
                } else {
                    ops9_format_error_count++;
                    count = 0;
                }
                break;

            case 2: // 接收 24 字节的数据区
                posture.data[i] = ch;
                i++;
                if (i >= 24) {
                    i = 0;
                    count++;
                }
                break;

            case 3: // 等待帧尾第一个字节 0x0A
                if (ch == 0x0A)
                    count++;
                else {
                    ops9_format_error_count++;
                    count = 0;
                }
                break;

            case 4: // 等待帧尾第二个字节 0x0D 并提取数据
                if (ch == 0x0D)
                {
                    float ops_zangle = posture.ActVal[OPS9_INDEX_Z_ANGLE];
                    float ops_pos_x  = posture.ActVal[OPS9_INDEX_POS_X];
                    float ops_pos_y  = posture.ActVal[OPS9_INDEX_POS_Y];

                    /*
                     * 直接映射：OPS X -> 车辆 X，OPS Y -> 车辆 Y，OPS角度 -> 车辆航向角。
                     * 不再进行旧注释中所说的 X/Y 交换或符号反转。
                     */
                    if (isfinite(ops_pos_x) && isfinite(ops_pos_y) && isfinite(ops_zangle)) {
                        robot_x = ops_pos_x;
                        robot_y = ops_pos_y;
                        robot_yaw = ops_zangle;
                        ops9_frame_count++;
                        ops9_last_update_tick = HAL_GetTick();
                    } else {
                        /* 数据损坏时保留上一帧有效坐标，避免 NaN 进入 PID。 */
                        ops9_invalid_frame_count++;
                    }
                } else {
                    ops9_format_error_count++;
                }
                count = 0;
                break;

            default:
                ops9_format_error_count++;
                count = 0;
                break;
        }

        // 重新开启下一次单字节中断接收
        HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
    }
}
uint8_t OPS9_Reset_Zero(void)
{
    // OPS-9 manual: zero command is "ACT0" (digit zero).
    static uint8_t zero_command[] = "ACT0";
    if (HAL_UART_Transmit_IT(&huart2, zero_command, 4U) != HAL_OK) {
        ops9_uart_error_count++;
        return 0U;
    }
    return 1U;
}
