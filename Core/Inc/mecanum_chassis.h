/*
 * mecanum_chassis.h
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */

#ifndef INC_MECANUM_CHASSIS_H_
#define INC_MECANUM_CHASSIS_H_

#ifdef CONTROL_HOST_TEST
#include "control_test_hal.h"
#else
#include "main.h"
#endif
#include "motor_monitor.h"

/* 机器人底盘物理参数定义 */
#define WHEEL_DIAMETER  0.075f      // 轮子直径 75mm (单位:m)
#define ROBOT_W         0.229f      // 轮距 (左右轮距离) (单位:m)
#define ROBOT_H         0.255f      // 轴距 (前后轮距离) (单位:m)
#define MECANUM_MAX_WHEEL_SPEED_MPS 0.80f  // 单轮线速度硬保护，组合运动时按比例缩放

/*
 * 运动学输入是车体坐标速度：+Vx 向车体右侧，+Vy 向车头前方。
 * 车头与 OPS +Y 对齐时，车体 +Vx/+Vy 才分别等于 OPS +X/+Y；
 * 航向变化后，全局速度必须先旋转到车体坐标再传入本函数。
 * OPS位置单位为 mm/degree；这里的速度单位为 m/s 和 rad/s。
 */

/* 外部调用函数声明 */
void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br);
float MsToRpm(float v_ms);
uint8_t SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br);
void ReadAllMotorsSpeed(void);
uint8_t StopAllMotors(void);
uint8_t Mecanum_ConsumeCanTxFault(void);
void Mecanum_ReportCanTxResult(uint8_t result);
/* 后台速度轮询只做查询：入队失败属于瞬时拥塞，不能升级为会停车的运动故障。 */
void Mecanum_ReportPollResult(uint8_t result);
uint32_t Mecanum_GetPollFailures(void);
void Mecanum_ProcessFeedback(uint32_t now);
uint8_t Mecanum_FeedbackReady(uint8_t mask);
uint8_t Mecanum_SetRequiredMotorMask(uint8_t mask);
uint8_t Mecanum_GetRequiredMotorMask(void);
MotorStopMonitor Mecanum_GetStopStatus(void);
float Mecanum_GetAppliedScale(void);



#endif /* INC_MECANUM_CHASSIS_H_ */
