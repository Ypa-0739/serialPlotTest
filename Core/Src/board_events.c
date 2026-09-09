#include "robot_app.h"
#include "usart.h"
#include "ops9.h"
#include "zdtCan.h"

/* HAL 回调只负责按外设路由。业务状态由应用/驱动模块各自持有。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) ZDT_CAN_RxFIFO0_Handler(hcan);
    else if (hcan->Instance == CAN2) RobotApp_Can2Rx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) OPS9_UART_RxCpltCallback(huart);
    else if (huart->Instance == USART1) RobotApp_HostRxComplete();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t ops9_rx_byte;
    if (huart->Instance == USART2) {
        ops9_uart_error_count++;
        HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1U);
    } else if (huart->Instance == USART1) RobotApp_HostError();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) RobotApp_HostTxComplete();
}

int _write(int file, char *text, int length)
{
    (void)file;
    return RobotApp_Write(text, length);
}
