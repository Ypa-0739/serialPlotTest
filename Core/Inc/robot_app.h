#ifndef ROBOT_APP_H
#define ROBOT_APP_H

/* 由 main 在 CubeMX 外设初始化完成后调用，单个主循环负责全部业务状态。 */
void RobotApp_Init(void);
void RobotApp_Process(void);

/* board_events.c 专用适配入口；RX/错误只组帧或锁存，不在 ISR 发起运动。 */
void RobotApp_HostRxComplete(void);
void RobotApp_HostError(void);
void RobotApp_HostTxComplete(void);
void RobotApp_Can2Rx(void);
int RobotApp_Write(char *text, int length);

#endif
