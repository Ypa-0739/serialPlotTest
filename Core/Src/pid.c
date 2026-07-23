/*
 * pid.c
 *
 *  Created on: Mar 9, 2026
 *      Author: steph
 */
#include "pid.h"

/**
 * @brief  初始化 PID 控制器
 * @param  pid          PID 结构体指针
 * @param  Kp           比例系数
 * @param  Ki           积分系数
 * @param  Kd           微分系数
 * @param  max_out      最大输出限幅 (例如电机的最大 RPM)
 * @param  max_integral 最大积分限幅 (防止长时间卡住导致积分爆表)
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float max_out, float max_integral)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->target = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;

    pid->max_out = max_out;
    pid->max_integral = max_integral;
}

/**
 * @brief  设置 PID 的目标值
 */
void PID_SetTarget(PID_Controller *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  重置 PID 状态 (清空历史误差和积分)
 * @note   通常在改变目标、重新发车时调用
 */
void PID_Reset(PID_Controller *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
}

/**
 * @brief  执行一次位置式 PID 计算
 * @param  pid          PID 结构体指针
 * @param  current_val  传感器当前真实值 (如 OPS-9 的当前 X 坐标)
 * @return float        PID 计算输出的控制量 (如电机的目标速度)
 */
float PID_CalcError(PID_Controller *pid, float error)
{
    float candidate_integral = pid->integral;
    float p_out;
    float d_out;
    float total_out;

    pid->error = error;
    p_out = pid->Kp * pid->error;
    d_out = pid->Kd * (pid->error - pid->last_error);

    if (pid->Ki != 0.0f) {
        candidate_integral += pid->error;
        if (candidate_integral > pid->max_integral) {
            candidate_integral = pid->max_integral;
        } else if (candidate_integral < -pid->max_integral) {
            candidate_integral = -pid->max_integral;
        }

        total_out = p_out + pid->Ki * candidate_integral + d_out;
        /* 输出已经饱和且误差仍推动同一方向时暂停积分，避免停车后积分继续释放。 */
        if (!((total_out > pid->max_out && pid->error > 0.0f) ||
              (total_out < -pid->max_out && pid->error < 0.0f))) {
            pid->integral = candidate_integral;
        }
    }

    total_out = p_out + pid->Ki * pid->integral + d_out;
    if (total_out > pid->max_out) {
        total_out = pid->max_out;
    } else if (total_out < -pid->max_out) {
        total_out = -pid->max_out;
    }
    pid->last_error = pid->error;
    return total_out;
}

float PID_Calc(PID_Controller *pid, float current_val)
{
    return PID_CalcError(pid, pid->target - current_val);
}



