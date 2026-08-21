/*
 * mecanum_chassis.c
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */
/*
 * @brief  麦克纳姆轮车体速度逆解算
 * @param  Vx: 车体向右速度，单位 m/s
 * @param  Vy: 车体向前速度，单位 m/s
 * @param  Vz: 沿 OPS 航向角正方向的角速度，单位 rad/s
 * @retval 算出各轮目标线速度，存入指针
 */
#include "mecanum_chassis.h"
#include "zdtEmm.h"
#include <math.h>

static volatile uint8_t can_tx_fault_latched = 0U;

void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br) {
    float L = (ROBOT_H / 2.0f) + (ROBOT_W / 2.0f);

    *V_bl = -Vx + Vy - Vz * L;  // ID 1 左后
    *V_fl =  Vx + Vy - Vz * L;  // ID 2 左前
    *V_fr =  Vx - Vy - Vz * L;  // ID 3 右前
    *V_br = -Vx - Vy - Vz * L;  // ID 4 右后
}

/*
 * @brief  线速度 (m/s) 转 电机转速 (RPM)
 */
float MsToRpm(float v_ms) {
    if (WHEEL_DIAMETER <= 0.0f) return 0.0f;
    // V = RPM * π * D / 60  =>  RPM = V * 60 / (π * D)
    return v_ms * 60.0f / (3.1415926f * WHEEL_DIAMETER);
}

/*
 * @brief  设置 4 个轮子速度并下发至 CAN 节点
 */
uint8_t SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br) {
    float max_abs = fabsf(V_bl);
    float scale;
    uint8_t result = 0U;

    if (fabsf(V_fl) > max_abs) max_abs = fabsf(V_fl);
    if (fabsf(V_fr) > max_abs) max_abs = fabsf(V_fr);
    if (fabsf(V_br) > max_abs) max_abs = fabsf(V_br);
    if (max_abs > MECANUM_MAX_WHEEL_SPEED_MPS) {
        /* 四轮同时按比例缩放，保留期望的平移与旋转方向比例。 */
        scale = MECANUM_MAX_WHEEL_SPEED_MPS / max_abs;
        V_bl *= scale;
        V_fl *= scale;
        V_fr *= scale;
        V_br *= scale;
    }

    result |= ZDT_Emm_SetSpeedByID(1, MsToRpm(V_bl));  // ID 1: 左后
    result |= ZDT_Emm_SetSpeedByID(2, MsToRpm(V_fl));  // ID 2: 左前
    result |= ZDT_Emm_SetSpeedByID(3, MsToRpm(V_fr));  // ID 3: 右前
    result |= ZDT_Emm_SetSpeedByID(4, MsToRpm(V_br));  // ID 4: 右后
    if (result != 0U) can_tx_fault_latched = 1U;
    return result;
}

/*
 * @brief  向 4 个电机发送读取速度的指令
 */
void ReadAllMotorsSpeed(void) {
    ZDT_Emm_ReadSpeedByID(1);
    ZDT_Emm_ReadSpeedByID(2);
    ZDT_Emm_ReadSpeedByID(3);
    ZDT_Emm_ReadSpeedByID(4);
}

/*
 * @brief  紧急停止所有电机
 */
uint8_t StopAllMotors(void) {
    return SetAllMotorsSpeed(0.0f, 0.0f, 0.0f, 0.0f);
}

uint8_t Mecanum_ConsumeCanTxFault(void)
{
    uint8_t fault = can_tx_fault_latched;
    can_tx_fault_latched = 0U;
    return fault;
}

void Mecanum_ClearCanTxFault(void)
{
    can_tx_fault_latched = 0U;
}

void Mecanum_ReportCanTxResult(uint8_t result)
{
    if (result != 0U) can_tx_fault_latched = 1U;
}



