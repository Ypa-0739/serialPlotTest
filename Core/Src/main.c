/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zdtCan.h"
#include "zdtEmm.h"
#include "serialPlot.h"
#include "zdtUart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mecanum_chassis.h"
#include "ops9.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LLM_TUNE_CONTROL_PERIOD_MS  20U
#define LLM_TUNE_DURATION_MS        5000U
#define LLM_TUNE_TARGET_MM          200.0f
#define LLM_TUNE_TARGET_YAW_DEG     30.0f
#define LLM_TUNE_MAX_SPEED_MPS      0.15f
#define LLM_TUNE_SPEED_HARD_MAX_MPS 0.30f
#define LLM_TUNE_MAX_ACCEL_MPS2     0.20f
#define LLM_TUNE_MAX_DECEL_MPS2     0.40f
#define LLM_TUNE_YAW_MAX_RADPS      0.30f
#define LLM_TUNE_YAW_HARD_MAX_RADPS 0.80f
#define LLM_TUNE_YAW_ACCEL_RADPS2   0.50f
#define LLM_TUNE_YAW_DECEL_RADPS2   0.80f
#define LLM_TUNE_OPS_TIMEOUT_MS     300U
#define LLM_TUNE_HOST_TIMEOUT_MS    1500U
#define LLM_TUNE_OVERTRAVEL_MM      50.0f
#define LLM_TUNE_WRONG_DIR_MM       25.0f
#define LLM_TUNE_MAX_YAW_ERROR_DEG  15.0f
#define LLM_TUNE_POSITION_TOL_MM    5.0f
#define LLM_TUNE_YAW_TOL_DEG        1.0f
#define LLM_TUNE_SETTLE_CYCLES      10U
#define LLM_POSE_POSITION_TOL_MM    2.0f
#define LLM_POSE_YAW_TOL_DEG        0.5f
#define LLM_POSE_SETTLE_CYCLES      15U
#define LLM_TUNE_KP_MAX             0.005f
#define LLM_TUNE_KI_MAX             0.00005f
#define LLM_TUNE_KD_MAX             0.002f
#define LLM_TUNE_YAW_KP_MAX         0.05f
#define LLM_TUNE_YAW_KI_MAX         0.00010f
#define LLM_TUNE_YAW_KD_MAX         0.02f
#define LLM_TUNE_HOLD_LINEAR_MPS    0.10f
#define LLM_TUNE_HOLD_YAW_RADPS     0.15f
#define OPS_CENTER_OFFSET_X_MM      0.0f
#define OPS_CENTER_OFFSET_Y_MM      25.0f
#define LLM_TUNE_CROSS_TRACK_MM     50.0f
#define LLM_TUNE_YAW_TRANSLATION_MM 50.0f
#define DEBUG_MOTOR_MAX_RPM         300.0f
#define DEBUG_MOTOR_DEFAULT_MS      2000UL
#define DEBUG_MOTOR_MAX_MS          10000UL
#define DEBUG_MOVE_DEFAULT_MPS      0.04f
#define DEBUG_MOVE_MAX_MPS          0.08f
#define DEBUG_MOVE_DEFAULT_MS       1000UL
#define DEBUG_MOVE_MAX_MS           3000UL
#define DEBUG_TURN_DEFAULT_RADPS     0.15f
#define DEBUG_TURN_MAX_RADPS         0.30f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ZDT_Motor_t motors[4];

uint32_t last_odom_tick = 0;
uint8_t move_state = 0; // 0:向前，1:停，2:向后，3:停

// 4个电机速度缓存
float motor_target_speed[4] = {0};  // 目标速度
float motor_actual_speed[4] = {0};  // 实际速度
PID_Controller pid_x;
PID_Controller pid_y;
PID_Controller pid_yaw;
// === LLM 自动调参状态机专属变量 ===
uint8_t pc_rx_byte;                       // PC 串口单字节接收
char pc_rx_buf[64];                       // ISR 正在拼接的命令
char pc_command_buf[64];                  // 主循环待处理的完整命令
volatile uint8_t pc_rx_idx = 0;
volatile uint8_t pc_command_ready = 0;

typedef enum {
    TUNE_STATE_WAIT = 0,     // 等待大模型参数状态
    TUNE_STATE_RUN           // 正在运行测试状态
} TuneState_t;

typedef enum {
    TUNE_AXIS_Y = 0,
    TUNE_AXIS_X,
    TUNE_AXIS_YAW
} TuneAxis_t;

TuneState_t current_tune_state = TUNE_STATE_WAIT;
TuneAxis_t current_tune_axis = TUNE_AXIS_Y;
uint32_t tune_start_time = 0;
uint32_t last_control_time = 0;
float start_x_pos = 0.0f;
float start_y_pos = 0.0f;
float start_yaw_deg = 0.0f;
float tune_direction = 1.0f;             // 每轮往返，避免一直驶离测试区域
float tune_output = 0.0f;
uint32_t tune_round_count = 0;
uint8_t debug_motor_active = 0U;
uint8_t debug_motor_id = 0U;
uint32_t debug_motor_stop_tick = 0U;
uint8_t debug_chassis_active = 0U;
uint32_t debug_chassis_stop_tick = 0U;
uint8_t ops_monitor_enabled = 0U;
uint32_t ops_monitor_last_tick = 0U;
uint32_t last_host_command_tick = 0U;
uint16_t tune_settle_cycles = 0U;
uint8_t pose_control_active = 0U;
float pose_target_x_mm = 0.0f;
float pose_target_y_mm = 0.0f;
float pose_target_yaw_deg = 0.0f;
float pose_target_center_x_mm = 0.0f;
float pose_target_center_y_mm = 0.0f;
float pose_output_vx = 0.0f;
float pose_output_vy = 0.0f;
float pose_output_vz = 0.0f;
uint16_t pose_settle_cycles = 0U;
uint32_t pose_last_control_time = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LLM_ProcessCommand(void);
static void LLM_StartTuneRound(void);
static void LLM_StopTuneRound(const char *reason);
static uint8_t LLM_ProcessDebugCommand(const char *command);
static void LLM_ProcessMotorFeedback(void);
static void LLM_PrintHelp(void);
static void LLM_PrintOpsStatus(void);
static float LLM_AngleErrorDeg(float current_deg, float reference_deg);
static void LLM_OpsToChassisCenter(float ops_x_mm, float ops_y_mm, float yaw_deg,
                                   float *center_x_mm, float *center_y_mm);
static const char *LLM_TuneAxisName(TuneAxis_t axis);
static PID_Controller *LLM_GetPidForAxis(TuneAxis_t axis);
static PID_Controller *LLM_GetTunePid(void);
static void LLM_ProcessPoseControl(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t LLM_IsValidMotorId(unsigned int id)
{
    return (id >= 1U && id <= 4U) ? 1U : 0U;
}

static float LLM_AngleErrorDeg(float current_deg, float reference_deg)
{
    float error = fmodf(current_deg - reference_deg + 180.0f, 360.0f);
    if (error < 0.0f) {
        error += 360.0f;
    }
    return error - 180.0f;
}

static void LLM_OpsToChassisCenter(float ops_x_mm, float ops_y_mm, float yaw_deg,
                                   float *center_x_mm, float *center_y_mm)
{
    float yaw_rad = yaw_deg * (3.1415926f / 180.0f);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    /*
     * OPS9 安装点位于车体几何中心前方 25 mm（车体 +Y）。
     * 先把车体偏置旋转到 OPS 全局坐标，再从传感器坐标中扣除，
     * 得到不会因原地旋转而沿圆弧移动的车体中心坐标。
     */
    *center_x_mm = ops_x_mm -
                   (cos_yaw * OPS_CENTER_OFFSET_X_MM -
                    sin_yaw * OPS_CENTER_OFFSET_Y_MM);
    *center_y_mm = ops_y_mm -
                   (sin_yaw * OPS_CENTER_OFFSET_X_MM +
                    cos_yaw * OPS_CENTER_OFFSET_Y_MM);
}

static const char *LLM_TuneAxisName(TuneAxis_t axis)
{
    if (axis == TUNE_AXIS_X) return "X";
    if (axis == TUNE_AXIS_YAW) return "YAW";
    return "Y";
}

static PID_Controller *LLM_GetTunePid(void)
{
    return LLM_GetPidForAxis(current_tune_axis);
}

static PID_Controller *LLM_GetPidForAxis(TuneAxis_t axis)
{
    if (axis == TUNE_AXIS_X) return &pid_x;
    if (axis == TUNE_AXIS_YAW) return &pid_yaw;
    return &pid_y;
}

static float LLM_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float LLM_Slew(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static float LLM_BrakeLimitLinear(float error_mm, float tolerance_mm)
{
    float remaining_m = (fabsf(error_mm) - tolerance_mm) / 1000.0f;
    if (remaining_m <= 0.0f) return 0.0f;
    return sqrtf(2.0f * LLM_TUNE_MAX_DECEL_MPS2 * remaining_m);
}

static float LLM_BrakeLimitYaw(float error_deg, float tolerance_deg)
{
    float remaining_rad = (fabsf(error_deg) - tolerance_deg) *
                          (3.1415926f / 180.0f);
    if (remaining_rad <= 0.0f) return 0.0f;
    return sqrtf(2.0f * LLM_TUNE_YAW_DECEL_RADPS2 * remaining_rad);
}

static void LLM_PrintHelp(void)
{
    printf("# HELP STATUS | PING | STOP | RESET | OPS STATUS | OPS MONITOR ON|OFF | OPS ZERO\r\n");
    printf("# HELP PROTO EMM|X | CAN STATUS | MOTOR EN|DIS <id>\r\n");
    printf("# HELP MOTOR RUN <id> <signed_rpm> [ms] | MOTOR STOP <id>|ALL | MOTOR GET <id>\r\n");
    printf("# HELP MOVE FWD|BACK|LEFT|RIGHT [mps] [ms] | TURN CW|CCW [radps] [ms] | MOVE STOP\r\n");
    printf("# HELP POSE SET <x_mm> <y_mm> <yaw_deg> | POSE STOP | POSE STATUS\r\n");
    printf("# HELP TUNE AXIS X|Y|YAW | TUNE LIMIT <mps_or_radps>\r\n");
    printf("# HELP PID SET X|Y|YAW <p> <i> <d> | PID LIMIT X|Y|YAW <value> | PID STATUS ALL\r\n");
    printf("# LIMIT motor id=1..4 rpm=+/-%.0f duration=100..%lu ms\r\n",
           DEBUG_MOTOR_MAX_RPM, DEBUG_MOTOR_MAX_MS);
    printf("# LIMIT move speed=0..%.2f mps duration=100..%lu ms; FWD=+Y LEFT=-X\r\n",
           DEBUG_MOVE_MAX_MPS, DEBUG_MOVE_MAX_MS);
    printf("# LIMIT turn speed=0..%.2f radps duration=100..%lu ms\r\n",
           DEBUG_TURN_MAX_RADPS, DEBUG_MOVE_MAX_MS);
}

static void LLM_PrintOpsStatus(void)
{
    /* 先读取中断更新的时间戳，再读取当前时间，避免无符号减法下溢。 */
    uint32_t last_byte_tick = ops9_last_byte_tick;
    uint32_t last_frame_tick = ops9_last_update_tick;
    uint32_t now = HAL_GetTick();
    uint32_t byte_age = (ops9_rx_byte_count == 0U) ? 0xFFFFFFFFUL : now - last_byte_tick;
    uint32_t frame_age = (ops9_frame_count == 0U) ? 0xFFFFFFFFUL : now - last_frame_tick;
    float center_x;
    float center_y;
    const char *link;

    /*
     * NO_DATA: USART2 一个字节都没收到，优先查供电、TX/RX、共地和RS-232模块。
     * BYTES_NO_FRAME: 有字节但不符合OPS帧，优先查波特率、模块方向和电平转换。
     * STALE: 曾经有完整帧，但最近停止更新。
     * OK: 最近100ms内收到过完整、有效的OPS坐标帧。
     */
    if (ops9_rx_byte_count == 0U) link = "NO_DATA";
    else if (ops9_frame_count == 0U) link = "BYTES_NO_FRAME";
    else if (frame_age > LLM_TUNE_OPS_TIMEOUT_MS) link = "STALE";
    else link = "OK";

    LLM_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
    printf("# OPS LINK=%s X=%.2f Y=%.2f YAW=%.2f CENTER_X=%.2f CENTER_Y=%.2f "
           "OFFSET_X=%.2f OFFSET_Y=%.2f BYTES=%lu HEADERS=%lu "
           "FRAMES=%lu INVALID=%lu FORMAT_ERR=%lu UART_ERR=%lu "
           "BYTE_AGE=%lu FRAME_AGE=%lu LAST=0x%02X\r\n",
           link, robot_x, robot_y, robot_yaw, center_x, center_y,
           OPS_CENTER_OFFSET_X_MM, OPS_CENTER_OFFSET_Y_MM,
           (unsigned long)ops9_rx_byte_count,
           (unsigned long)ops9_header_count,
           (unsigned long)ops9_frame_count,
           (unsigned long)ops9_invalid_frame_count,
           (unsigned long)ops9_format_error_count,
           (unsigned long)ops9_uart_error_count,
           (unsigned long)byte_age, (unsigned long)frame_age,
           ops9_last_raw_byte);
}

static void LLM_ProcessMotorFeedback(void)
{
    ZDT_MotorEvent_t event;
    while (ZDT_Emm_PollEvent(&event)) {
        if (event.function_code == 0x35U) {
            printf("# MOTOR SPEED ID=%u RPM=%.1f\r\n", event.motor_id, event.speed_rpm);
        } else if (event.function_code == 0x3AU) {
            printf("# MOTOR STATE ID=%u EN=%u REACHED=%u STALL=%u PROTECT=%u RAW=0x%02X\r\n",
                   event.motor_id,
                   (event.value & 0x01U) ? 1U : 0U,
                   (event.value & 0x02U) ? 1U : 0U,
                   (event.value & 0x04U) ? 1U : 0U,
                   (event.value & 0x08U) ? 1U : 0U,
                   event.value);
        } else {
            const char *result = "OTHER";
            /*
             * 速度闭环每 20 ms 给四个电机发送一次 0xF6，正常 ACK 会形成约
             * 200 行/秒的无效串口流量。仅静默正常 0x02 ACK，条件/格式等异常仍输出。
             */
            if (event.function_code == 0xF6U && event.value == 0x02U) {
                continue;
            }
            if (event.value == 0x02U) result = "OK";
            else if (event.value == 0xE2U) result = "CONDITION";
            else if (event.value == 0xEEU) result = "FORMAT";
            else if (event.value == 0x9FU) result = "DONE";
            printf("# MOTOR ACK ID=%u CMD=0x%02X CODE=0x%02X %s\r\n",
                   event.motor_id, event.function_code, event.value, result);
        }
    }
}

static uint8_t LLM_ProcessDebugCommand(const char *command)
{
    unsigned int id;
    float rpm;
    unsigned long duration_ms = DEBUG_MOTOR_DEFAULT_MS;
    unsigned long move_duration_ms = DEBUG_MOVE_DEFAULT_MS;
    char move_direction[8];
    float move_speed = DEBUG_MOVE_DEFAULT_MPS;
    float turn_speed = DEBUG_TURN_DEFAULT_RADPS;
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float v1, v2, v3, v4;
    float pose_x, pose_y, pose_yaw;
    float center_x, center_y;
    int fields;
    uint8_t result_a;
    uint8_t result_b;

    if (strcmp(command, "PING") == 0) {
        printf("# PONG\r\n");
        return 1U;
    }

    if (strcmp(command, "HELP") == 0) {
        LLM_PrintHelp();
        return 1U;
    }

    if (strcmp(command, "PROTO EMM") == 0 || strcmp(command, "PROTO X") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        current_tune_state = TUNE_STATE_WAIT;
        ZDT_Emm_SetProtocol((strcmp(command, "PROTO X") == 0) ? ZDT_PROTOCOL_X : ZDT_PROTOCOL_EMM);
        StopAllMotors();
        printf("# PROTOCOL %s\r\n", ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM");
        return 1U;
    }

    if (strcmp(command, "CAN STATUS") == 0) {
        ZDT_CAN_Stats_t stats;
        ZDT_CAN_GetStats(&stats);
        printf("# CAN STATE=%u ERROR=0x%08lX FREE=%lu TX_OK=%lu TX_ERR=%lu RX=%lu LAST=%u\r\n",
               (unsigned int)HAL_CAN_GetState(&hcan1),
               (unsigned long)HAL_CAN_GetError(&hcan1),
               (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
               (unsigned long)stats.tx_ok, (unsigned long)stats.tx_error,
               (unsigned long)stats.rx_count, stats.last_tx_result);
        return 1U;
    }

    if (strcmp(command, "OPS STATUS") == 0) {
        LLM_PrintOpsStatus();
        return 1U;
    }

    if (strcmp(command, "OPS MONITOR ON") == 0) {
        ops_monitor_enabled = 1U;
        ops_monitor_last_tick = 0U;
        printf("# OPS MONITOR ON PERIOD=1000MS\r\n");
        return 1U;
    }

    if (strcmp(command, "OPS MONITOR OFF") == 0) {
        ops_monitor_enabled = 0U;
        printf("# OPS MONITOR OFF\r\n");
        return 1U;
    }

    if (strcmp(command, "OPS ZERO") == 0) {
        OPS9_Reset_Zero();
        printf("# OPS ZERO SENT ACT0\r\n");
        return 1U;
    }

    if (strcmp(command, "POSE STOP") == 0) {
        pose_control_active = 0U;
        pose_output_vx = 0.0f;
        pose_output_vy = 0.0f;
        pose_output_vz = 0.0f;
        StopAllMotors();
        printf("# POSE STOP\r\n");
        return 1U;
    }

    if (strcmp(command, "POSE STATUS") == 0) {
        LLM_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
        printf("# POSE ACTIVE=%u TARGET_X=%.2f TARGET_Y=%.2f TARGET_YAW=%.2f "
               "TARGET_CENTER_X=%.2f TARGET_CENTER_Y=%.2f "
               "X=%.2f Y=%.2f YAW=%.2f CENTER_X=%.2f CENTER_Y=%.2f\r\n",
               pose_control_active, pose_target_x_mm, pose_target_y_mm,
               pose_target_yaw_deg, pose_target_center_x_mm, pose_target_center_y_mm,
               robot_x, robot_y, robot_yaw, center_x, center_y);
        return 1U;
    }

    if (sscanf(command, "POSE SET %f %f %f", &pose_x, &pose_y, &pose_yaw) == 3) {
        uint32_t last_ops_tick = ops9_last_update_tick;
        uint32_t now = HAL_GetTick();
        if (!isfinite(pose_x) || !isfinite(pose_y) || !isfinite(pose_yaw)) {
            printf("# ERROR POSE VALUE\r\n");
        } else if (ops9_frame_count == 0U ||
                   (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS) {
            StopAllMotors();
            printf("# ERROR OPS NOT READY\r\n");
        } else {
            debug_motor_active = 0U;
            debug_chassis_active = 0U;
            current_tune_state = TUNE_STATE_WAIT;
            StopAllMotors();
            PID_Reset(&pid_x);
            PID_Reset(&pid_y);
            PID_Reset(&pid_yaw);
            /*
             * POSE SET 继续接收 OPS 原始目标坐标，保证现有上位机命令兼容；
             * 内部按目标航向换算成车体中心目标，再用中心坐标闭环。
             */
            LLM_OpsToChassisCenter(pose_x, pose_y, pose_yaw,
                                   &pose_target_center_x_mm,
                                   &pose_target_center_y_mm);
            PID_SetTarget(&pid_x, pose_target_center_x_mm);
            PID_SetTarget(&pid_y, pose_target_center_y_mm);
            pose_target_x_mm = pose_x;
            pose_target_y_mm = pose_y;
            pose_target_yaw_deg = pose_yaw;
            pose_output_vx = 0.0f;
            pose_output_vy = 0.0f;
            pose_output_vz = 0.0f;
            pose_settle_cycles = 0U;
            pose_last_control_time = now;
            pose_control_active = 1U;
            printf("# POSE START X=%.2f Y=%.2f YAW=%.2f "
                   "CENTER_X=%.2f CENTER_Y=%.2f TOL_MM=%.2f TOL_YAW=%.2f\r\n",
                   pose_x, pose_y, pose_yaw,
                   pose_target_center_x_mm, pose_target_center_y_mm,
                   LLM_POSE_POSITION_TOL_MM, LLM_POSE_YAW_TOL_DEG);
        }
        return 1U;
    }

    if (strcmp(command, "MOTOR STOP ALL") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        current_tune_state = TUNE_STATE_WAIT;
        printf("# MOTOR STOP ALL\r\n");
        return 1U;
    }

    if (strcmp(command, "MOVE STOP") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        current_tune_state = TUNE_STATE_WAIT;
        printf("# MOVE STOP\r\n");
        return 1U;
    }

    fields = sscanf(command, "TURN %7s %f %lu",
                    move_direction, &turn_speed, &move_duration_ms);
    if (fields >= 1) {
        if (!isfinite(turn_speed) || turn_speed <= 0.0f ||
            turn_speed > DEBUG_TURN_MAX_RADPS) {
            printf("# ERROR TURN SPEED 0..%.2f RADPS\r\n", DEBUG_TURN_MAX_RADPS);
            return 1U;
        }
        if (move_duration_ms < 100UL || move_duration_ms > DEBUG_MOVE_MAX_MS) {
            printf("# ERROR TURN DURATION 100..%lu MS\r\n", DEBUG_MOVE_MAX_MS);
            return 1U;
        }

        if (strcmp(move_direction, "CCW") == 0)      vz =  turn_speed;
        else if (strcmp(move_direction, "CW") == 0) vz = -turn_speed;
        else {
            printf("# ERROR TURN DIR CW|CCW\r\n");
            return 1U;
        }

        /* 旋转测试只给 Vz，低速且定时自动停止，用于检查四轮旋转组合和OPS航向。 */
        StopAllMotors();
        debug_motor_active = 0U;
        pose_control_active = 0U;
        current_tune_state = TUNE_STATE_WAIT;
        Mecanum_Kinematics(0.0f, 0.0f, vz, &v1, &v2, &v3, &v4);
        SetAllMotorsSpeed(v1, v2, v3, v4);
        debug_chassis_active = 1U;
        debug_chassis_stop_tick = HAL_GetTick() + (uint32_t)move_duration_ms;
        printf("# TURN %s SPEED=%.3f RADPS MS=%lu VZ=%.3f\r\n",
               move_direction, turn_speed, move_duration_ms, vz);
        return 1U;
    }

    fields = sscanf(command, "MOVE %7s %f %lu",
                    move_direction, &move_speed, &move_duration_ms);
    if (fields >= 1) {
        if (!isfinite(move_speed) || move_speed <= 0.0f ||
            move_speed > DEBUG_MOVE_MAX_MPS) {
            printf("# ERROR MOVE SPEED 0..%.2f MPS\r\n", DEBUG_MOVE_MAX_MPS);
            return 1U;
        }
        if (move_duration_ms < 100UL || move_duration_ms > DEBUG_MOVE_MAX_MS) {
            printf("# ERROR MOVE DURATION 100..%lu MS\r\n", DEBUG_MOVE_MAX_MS);
            return 1U;
        }

        /* 调试命令使用车体方向；仅在车头对齐 OPS +Y 时与 OPS 全局轴重合。 */
        if (strcmp(move_direction, "FWD") == 0)       vy =  move_speed;
        else if (strcmp(move_direction, "BACK") == 0) vy = -move_speed;
        else if (strcmp(move_direction, "LEFT") == 0) vx = -move_speed;
        else if (strcmp(move_direction, "RIGHT") == 0) vx = move_speed;
        else {
            printf("# ERROR MOVE DIR FWD|BACK|LEFT|RIGHT\r\n");
            return 1U;
        }

        StopAllMotors();
        debug_motor_active = 0U;
        pose_control_active = 0U;
        current_tune_state = TUNE_STATE_WAIT;
        Mecanum_Kinematics(vx, vy, 0.0f, &v1, &v2, &v3, &v4);
        SetAllMotorsSpeed(v1, v2, v3, v4);
        debug_chassis_active = 1U;
        debug_chassis_stop_tick = HAL_GetTick() + (uint32_t)move_duration_ms;
        printf("# MOVE %s SPEED=%.3f MPS MS=%lu VX=%.3f VY=%.3f\r\n",
               move_direction, move_speed, move_duration_ms, vx, vy);
        return 1U;
    }

    if (sscanf(command, "MOTOR EN %u", &id) == 1) {
        if (!LLM_IsValidMotorId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_EnableSingleMotor((uint8_t)id, 1U);
            printf("# MOTOR EN ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR DIS %u", &id) == 1) {
        if (!LLM_IsValidMotorId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            ZDT_Emm_SetSingleMotorSpeed((uint8_t)id, 0.0f);
            result_a = ZDT_Emm_EnableSingleMotor((uint8_t)id, 0U);
            if (debug_motor_active && debug_motor_id == (uint8_t)id) debug_motor_active = 0U;
            printf("# MOTOR DIS ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR STOP %u", &id) == 1) {
        if (!LLM_IsValidMotorId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_SetSingleMotorSpeed((uint8_t)id, 0.0f);
            if (debug_motor_active && debug_motor_id == (uint8_t)id) debug_motor_active = 0U;
            printf("# MOTOR STOP ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR GET %u", &id) == 1) {
        if (!LLM_IsValidMotorId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_ReadSingleMotorSpeed((uint8_t)id);
            HAL_Delay(2);
            result_b = ZDT_Emm_ReadStatusByID((uint8_t)id);
            printf("# MOTOR GET ID=%u TX_SPEED=%u TX_STATE=%u\r\n", id, result_a, result_b);
        }
        return 1U;
    }

    fields = sscanf(command, "MOTOR RUN %u %f %lu", &id, &rpm, &duration_ms);
    if (fields >= 2) {
        if (!LLM_IsValidMotorId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else if (!isfinite(rpm) || rpm == 0.0f || fabsf(rpm) > DEBUG_MOTOR_MAX_RPM) {
            printf("# ERROR RPM RANGE +/-%.0f NONZERO\r\n", DEBUG_MOTOR_MAX_RPM);
        } else if (duration_ms < 100UL || duration_ms > DEBUG_MOTOR_MAX_MS) {
            printf("# ERROR DURATION 100..%lu MS\r\n", DEBUG_MOTOR_MAX_MS);
        } else {
            StopAllMotors();
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            current_tune_state = TUNE_STATE_WAIT;
            result_a = ZDT_Emm_EnableSingleMotor((uint8_t)id, 1U);
            HAL_Delay(5);
            result_b = ZDT_Emm_SetSingleMotorSpeed((uint8_t)id, rpm);
            if (result_a == 0U && result_b == 0U) {
                debug_motor_active = 1U;
                debug_motor_id = (uint8_t)id;
                debug_motor_stop_tick = HAL_GetTick() + (uint32_t)duration_ms;
            }
            printf("# MOTOR RUN ID=%u RPM=%.1f MS=%lu PROTO=%s TX_EN=%u TX_RUN=%u\r\n",
                   id, rpm, duration_ms,
                   ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM",
                   result_a, result_b);
        }
        return 1U;
    }

    return 0U;
}

static void LLM_ProcessPoseControl(uint32_t now)
{
    float current_ops_x;
    float current_ops_y;
    float current_x;
    float current_y;
    float current_yaw;
    float error_x;
    float error_y;
    float error_yaw;
    float distance_mm;
    float world_vx;
    float world_vy;
    float world_speed;
    float linear_limit;
    float heading_rad;
    float body_vx;
    float body_vy;
    float desired_vz;
    float yaw_limit;
    float step_linear;
    float step_yaw;
    float delta_vx;
    float delta_vy;
    float delta_v;
    float target_linear_speed;
    float current_linear_speed;
    float v1, v2, v3, v4;
    uint32_t last_ops_tick;

    if (!pose_control_active ||
        (uint32_t)(now - pose_last_control_time) < LLM_TUNE_CONTROL_PERIOD_MS) {
        return;
    }

    last_ops_tick = ops9_last_update_tick;
    now = HAL_GetTick();
    pose_last_control_time += LLM_TUNE_CONTROL_PERIOD_MS;
    current_ops_x = robot_x;
    current_ops_y = robot_y;
    current_yaw = robot_yaw;

    if (!isfinite(current_ops_x) || !isfinite(current_ops_y) || !isfinite(current_yaw) ||
        ops9_frame_count == 0U ||
        (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS ||
        (uint32_t)(now - last_host_command_tick) > LLM_TUNE_HOST_TIMEOUT_MS) {
        pose_control_active = 0U;
        StopAllMotors();
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    LLM_OpsToChassisCenter(current_ops_x, current_ops_y, current_yaw,
                           &current_x, &current_y);
    error_x = pose_target_center_x_mm - current_x;
    error_y = pose_target_center_y_mm - current_y;
    error_yaw = LLM_AngleErrorDeg(pose_target_yaw_deg, current_yaw);
    distance_mm = sqrtf(error_x * error_x + error_y * error_y);

    /* X/Y PID先给出全局速度，再旋转到车体坐标，供树莓派直接下发全局目标位姿。 */
    world_vx = PID_Calc(&pid_x, current_x);
    world_vy = PID_Calc(&pid_y, current_y);
    world_speed = sqrtf(world_vx * world_vx + world_vy * world_vy);
    linear_limit = LLM_BrakeLimitLinear(distance_mm, LLM_POSE_POSITION_TOL_MM);
    if (linear_limit > pid_y.max_out) linear_limit = pid_y.max_out;
    if (world_speed > linear_limit && world_speed > 0.0001f) {
        float scale = linear_limit / world_speed;
        world_vx *= scale;
        world_vy *= scale;
    }

    heading_rad = current_yaw * (3.1415926f / 180.0f);
    body_vx = cosf(heading_rad) * world_vx + sinf(heading_rad) * world_vy;
    body_vy = -sinf(heading_rad) * world_vx + cosf(heading_rad) * world_vy;

    desired_vz = PID_CalcError(&pid_yaw, error_yaw);
    yaw_limit = LLM_BrakeLimitYaw(error_yaw, LLM_POSE_YAW_TOL_DEG);
    desired_vz = LLM_ClampFloat(desired_vz, -yaw_limit, yaw_limit);

    target_linear_speed = sqrtf(body_vx * body_vx + body_vy * body_vy);
    current_linear_speed = sqrtf(pose_output_vx * pose_output_vx +
                                 pose_output_vy * pose_output_vy);
    step_linear = (target_linear_speed < current_linear_speed ?
                   LLM_TUNE_MAX_DECEL_MPS2 : LLM_TUNE_MAX_ACCEL_MPS2) *
                  ((float)LLM_TUNE_CONTROL_PERIOD_MS / 1000.0f);
    step_yaw = (fabsf(desired_vz) < fabsf(pose_output_vz) ?
                LLM_TUNE_YAW_DECEL_RADPS2 : LLM_TUNE_YAW_ACCEL_RADPS2) *
               ((float)LLM_TUNE_CONTROL_PERIOD_MS / 1000.0f);
    delta_vx = body_vx - pose_output_vx;
    delta_vy = body_vy - pose_output_vy;
    delta_v = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
    if (delta_v > step_linear && delta_v > 0.0001f) {
        pose_output_vx += delta_vx * step_linear / delta_v;
        pose_output_vy += delta_vy * step_linear / delta_v;
    } else {
        pose_output_vx = body_vx;
        pose_output_vy = body_vy;
    }
    pose_output_vz = LLM_Slew(pose_output_vz, desired_vz, step_yaw);

    if (distance_mm <= LLM_POSE_POSITION_TOL_MM &&
        fabsf(error_yaw) <= LLM_POSE_YAW_TOL_DEG) {
        pose_settle_cycles++;
        if (pose_settle_cycles >= LLM_POSE_SETTLE_CYCLES) {
            pose_control_active = 0U;
            StopAllMotors();
            printf("# POSE TARGET X=%.2f Y=%.2f YAW=%.2f "
                   "CENTER_X=%.2f CENTER_Y=%.2f ERROR_MM=%.2f ERROR_YAW=%.2f\r\n",
                   current_ops_x, current_ops_y, current_yaw,
                   current_x, current_y, distance_mm, error_yaw);
            return;
        }
    } else {
        pose_settle_cycles = 0U;
    }

    Mecanum_Kinematics(pose_output_vx, pose_output_vy, pose_output_vz,
                       &v1, &v2, &v3, &v4);
    SetAllMotorsSpeed(v1, v2, v3, v4);
}

static void LLM_StartTuneRound(void)
{
    uint32_t last_ops_tick = ops9_last_update_tick;
    uint32_t now = HAL_GetTick();
    PID_Controller *pid = LLM_GetTunePid();
    float center_x;
    float center_y;

    debug_chassis_active = 0U;
    pose_control_active = 0U;

    /* 没有新鲜的 OPS 坐标时拒绝启动，避免位置环退化成开环运动。 */
    if (ops9_frame_count == 0U || (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS) {
        StopAllMotors();
        current_tune_state = TUNE_STATE_WAIT;
        printf("# ERROR OPS NOT READY\r\n");
        return;
    }

    if (tune_round_count > 0U) {
        tune_direction = -tune_direction;
    }
    tune_round_count++;

    StopAllMotors();
    /*
     * 线性轴调参同时使用另外两个 PID 保持正交位移和起始航向。
     * 每轮开始必须清空三轴历史，避免上一轮积分/微分状态串入本轮。
     */
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    PID_Reset(&pid_yaw);
    LLM_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
    start_x_pos = center_x;
    start_y_pos = center_y;
    start_yaw_deg = robot_yaw;
    /* 调参数据统一归一化为从0向正目标运动，便于正反轮次直接比较。 */
    PID_SetTarget(pid, current_tune_axis == TUNE_AXIS_YAW ?
                       LLM_TUNE_TARGET_YAW_DEG : LLM_TUNE_TARGET_MM);
    tune_output = 0.0f;
    tune_settle_cycles = 0U;
    tune_start_time = now;
    last_control_time = tune_start_time;
    current_tune_state = TUNE_STATE_RUN;
    printf("# ROUND START %lu AXIS=%s DIR %.0f X=%.2f Y=%.2f YAW=%.2f "
           "CENTER_X=%.2f CENTER_Y=%.2f\r\n",
           (unsigned long)tune_round_count, LLM_TuneAxisName(current_tune_axis), tune_direction,
           robot_x, robot_y, robot_yaw, center_x, center_y);
}

static void LLM_StopTuneRound(const char *reason)
{
    float center_x;
    float center_y;

    StopAllMotors();
    tune_output = 0.0f;
    current_tune_state = TUNE_STATE_WAIT;
    LLM_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
    printf("# ROUND STOP %s AXIS=%s X=%.2f Y=%.2f YAW=%.2f "
           "CENTER_X=%.2f CENTER_Y=%.2f\r\n",
           reason, LLM_TuneAxisName(current_tune_axis), robot_x, robot_y, robot_yaw,
           center_x, center_y);
}

static void LLM_ProcessCommand(void)
{
    char command[64];
    uint8_t i;
    float p_val, i_val, d_val;
    float speed_limit_val;
    char axis_name[8];
    PID_Controller *pid;
    TuneAxis_t requested_axis;
    float kp_max, ki_max, kd_max, output_hard_max;

    if (!pc_command_ready) {
        return;
    }

    __disable_irq();
    for (i = 0; i < sizeof(command); i++) {
        command[i] = pc_command_buf[i];
        if (command[i] == '\0') {
            break;
        }
    }
    command[sizeof(command) - 1U] = '\0';
    pc_command_ready = 0U;
    last_host_command_tick = HAL_GetTick();
    __enable_irq();

    if (LLM_ProcessDebugCommand(command)) {
        return;
    }

    if (strcmp(command, "PID STATUS ALL") == 0) {
        printf("# PID ALL X=%.7f,%.8f,%.7f Y=%.7f,%.8f,%.7f "
               "YAW=%.7f,%.8f,%.7f\r\n",
               pid_x.Kp, pid_x.Ki, pid_x.Kd,
               pid_y.Kp, pid_y.Ki, pid_y.Kd,
               pid_yaw.Kp, pid_yaw.Ki, pid_yaw.Kd);
    } else if (sscanf(command, "PID LIMIT %7s %f", axis_name, &speed_limit_val) == 2) {
        if (strcmp(axis_name, "X") == 0) requested_axis = TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = TUNE_AXIS_YAW;
        else {
            printf("# ERROR PID AXIS X|Y|YAW\r\n");
            return;
        }
        output_hard_max = requested_axis == TUNE_AXIS_YAW ?
                          LLM_TUNE_YAW_HARD_MAX_RADPS : LLM_TUNE_SPEED_HARD_MAX_MPS;
        if (isfinite(speed_limit_val) && speed_limit_val >= 0.02f &&
            speed_limit_val <= output_hard_max) {
            pid = LLM_GetPidForAxis(requested_axis);
            pid->max_out = speed_limit_val;
            printf("# PID LIMIT AXIS=%s OUTPUT=%.3f UNIT=%s\r\n",
                   LLM_TuneAxisName(requested_axis), speed_limit_val,
                   requested_axis == TUNE_AXIS_YAW ? "RADPS" : "MPS");
        } else {
            printf("# ERROR PID LIMIT OUTPUT 0.02..%.2f\r\n", output_hard_max);
        }
    } else if (sscanf(command, "PID SET %7s %f %f %f",
                      axis_name, &p_val, &i_val, &d_val) == 4) {
        if (strcmp(axis_name, "X") == 0) requested_axis = TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = TUNE_AXIS_YAW;
        else {
            printf("# ERROR PID AXIS X|Y|YAW\r\n");
            return;
        }
        pid = LLM_GetPidForAxis(requested_axis);
        kp_max = requested_axis == TUNE_AXIS_YAW ? LLM_TUNE_YAW_KP_MAX : LLM_TUNE_KP_MAX;
        ki_max = requested_axis == TUNE_AXIS_YAW ? LLM_TUNE_YAW_KI_MAX : LLM_TUNE_KI_MAX;
        kd_max = requested_axis == TUNE_AXIS_YAW ? LLM_TUNE_YAW_KD_MAX : LLM_TUNE_KD_MAX;
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= kp_max &&
            i_val >= 0.0f && i_val <= ki_max &&
            d_val >= 0.0f && d_val <= kd_max) {
            pid->Kp = p_val;
            pid->Ki = i_val;
            pid->Kd = d_val;
            PID_Reset(pid);
            printf("# PID LOADED AXIS=%s P=%.7f I=%.8f D=%.7f\r\n",
                   LLM_TuneAxisName(requested_axis), p_val, i_val, d_val);
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   kp_max, ki_max, kd_max);
        }
    } else if (sscanf(command, "TUNE AXIS %7s", axis_name) == 1) {
        if (strcmp(axis_name, "X") == 0) current_tune_axis = TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) current_tune_axis = TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) current_tune_axis = TUNE_AXIS_YAW;
        else {
            printf("# ERROR TUNE AXIS X|Y|YAW\r\n");
            return;
        }
        pose_control_active = 0U;
        LLM_StopTuneRound("AXIS");
        tune_round_count = 0U;
        tune_direction = 1.0f;
        printf("# TUNE AXIS %s UNIT_IN=%s UNIT_OUT=%s\r\n",
               LLM_TuneAxisName(current_tune_axis),
               current_tune_axis == TUNE_AXIS_YAW ? "DEG" : "MM",
               current_tune_axis == TUNE_AXIS_YAW ? "RADPS" : "MPS");
    } else if (sscanf(command, "TUNE LIMIT %f", &speed_limit_val) == 1) {
        pid = LLM_GetTunePid();
        output_hard_max = current_tune_axis == TUNE_AXIS_YAW ?
                          LLM_TUNE_YAW_HARD_MAX_RADPS : LLM_TUNE_SPEED_HARD_MAX_MPS;
        /* 工作速度上限由上位机统一配置；这里保留独立硬上限作为最终安全边界。 */
        if (isfinite(speed_limit_val) && speed_limit_val >= 0.02f &&
            speed_limit_val <= output_hard_max) {
            pid->max_out = speed_limit_val;
            printf("# TUNE LIMIT AXIS=%s OUTPUT=%.3f UNIT=%s HARD_MAX=%.3f\r\n",
                   LLM_TuneAxisName(current_tune_axis), pid->max_out,
                   current_tune_axis == TUNE_AXIS_YAW ? "RADPS" : "MPS",
                   output_hard_max);
        } else {
            printf("# ERROR TUNE LIMIT 0.02..%.2f\r\n", output_hard_max);
        }
    } else if ((sscanf(command, "SET P:%f I:%f D:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "SET KP:%f KI:%f KD:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "PID %f %f %f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "P:%f,I:%f,D:%f", &p_val, &i_val, &d_val) == 3)) {
        pid = LLM_GetTunePid();
        kp_max = current_tune_axis == TUNE_AXIS_YAW ? LLM_TUNE_YAW_KP_MAX : LLM_TUNE_KP_MAX;
        ki_max = current_tune_axis == TUNE_AXIS_YAW ? LLM_TUNE_YAW_KI_MAX : LLM_TUNE_KI_MAX;
        kd_max = current_tune_axis == TUNE_AXIS_YAW ? LLM_TUNE_YAW_KD_MAX : LLM_TUNE_KD_MAX;
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= kp_max &&
            i_val >= 0.0f && i_val <= ki_max &&
            d_val >= 0.0f && d_val <= kd_max) {
            pid->Kp = p_val;
            pid->Ki = i_val;
            pid->Kd = d_val;
            printf("# PID UPDATED AXIS=%s P=%.7f I=%.8f D=%.7f\r\n",
                   LLM_TuneAxisName(current_tune_axis), p_val, i_val, d_val);
            LLM_StartTuneRound();
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   kp_max, ki_max, kd_max);
        }
    } else if (strcmp(command, "STATUS") == 0) {
        pid = LLM_GetTunePid();
        printf("# STATUS AXIS=%s P=%.7f I=%.8f D=%.7f MAX_OUT=%.3f STATE=%u PROTO=%s OPS_FRAMES=%lu\r\n",
               LLM_TuneAxisName(current_tune_axis), pid->Kp, pid->Ki, pid->Kd, pid->max_out,
               (unsigned int)current_tune_state,
               ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM",
               (unsigned long)ops9_frame_count);
    } else if (strcmp(command, "RESET") == 0) {
        PID_Reset(LLM_GetTunePid());
        pose_control_active = 0U;
        tune_round_count = 0U;
        tune_direction = 1.0f;
        LLM_StopTuneRound("RESET");
    } else if (strcmp(command, "STOP") == 0) {
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        LLM_StopTuneRound("HOST");
    } else {
        printf("# ERROR UNKNOWN COMMAND\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // 声明外部的接收缓存变量
  extern uint8_t ops9_rx_byte;
  // 开启 USART2 单字节中断接收
  HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
  // 开启 USART1 单字节中断接收 (接收 LLM 发来的参数)
    HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1);
  // 1. 初始化 CAN 和过滤器
  ZDT_CAN_ConfigFilter();

  // 2. 注册回调
  ZDT_CAN_RegisterCallback(ZDT_Emm_RxHandler);

  // 3. 初始化 4 个电机
  ZDT_Emm_InitAll();

  // 4. 使能所有电机（必须使能才能响应速度命令）
  // 依据：P48 5.3.2 电机使能控制
  HAL_Delay(100);
  ZDT_Emm_EnableByID(1);
  HAL_Delay(10);
  ZDT_Emm_EnableByID(2);
  HAL_Delay(10);
  ZDT_Emm_EnableByID(3);
  HAL_Delay(10);
  ZDT_Emm_EnableByID(4);
  HAL_Delay(100);  // 等待使能完成

  // 5.启动定时器（用于定时读取速度）
  HAL_TIM_Base_Start_IT(&htim3);

  // 6. 初始化里程计计时器
    last_odom_tick = HAL_GetTick();
    last_host_command_tick = HAL_GetTick();

  //7.初始化PID参数
  // 注意：坐标单位是 mm，误差 1000mm 时，乘以 Kp=0.001，算出的速度正好是 1.0 m/s
    PID_Init(&pid_x,   0.001f, 0.0f, 0.0f, LLM_TUNE_MAX_SPEED_MPS, 5000.0f);
    PID_Init(&pid_y,   0.001f, 0.0f, 0.0f, LLM_TUNE_MAX_SPEED_MPS, 5000.0f);
    PID_Init(&pid_yaw, 0.01f,  0.0f, 0.0f, LLM_TUNE_YAW_MAX_RADPS, 1000.0f);

    StopAllMotors();
    printf("# STM32F407 MECANUM X/Y/YAW PID CONTROLLER READY\r\n");
    printf("# CSV timestamp,setpoint,input,output,error,p,i,d,ops_x_mm,ops_y_mm,yaw_deg,"
           "cross_mm,yaw_delta_deg,hold_cross,hold_yaw,center_x_mm,center_y_mm;"
           " UNIT BY AXIS\r\n");
    printf("# MOTOR PROTOCOL DEFAULT EMM; SEND HELP FOR DEBUG COMMANDS\r\n");
    printf("# SEND OPS STATUS OR OPS MONITOR ON TO CHECK OPS-9 LINK\r\n");
    printf("# TUNE SPEED SET BY HOST; HARD LIMIT %.2fMPS; OPS/HOST LOSS STOPS MOTORS\r\n",
           LLM_TUNE_SPEED_HARD_MAX_MPS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();
      LLM_ProcessCommand();
      LLM_ProcessMotorFeedback();

      /* 命令处理和串口中断可能更新时间戳，超时判断前必须刷新当前时间。 */
      now = HAL_GetTick();
      LLM_ProcessPoseControl(now);

      if (ops_monitor_enabled && (uint32_t)(now - ops_monitor_last_tick) >= 1000U)
      {
          ops_monitor_last_tick = now;
          LLM_PrintOpsStatus();
      }

      if (debug_motor_active && (int32_t)(now - debug_motor_stop_tick) >= 0)
      {
          ZDT_Emm_SetSingleMotorSpeed(debug_motor_id, 0.0f);
          printf("# MOTOR AUTO STOP ID=%u\r\n", debug_motor_id);
          debug_motor_active = 0U;
      }

      if (debug_chassis_active && (int32_t)(now - debug_chassis_stop_tick) >= 0)
      {
          StopAllMotors();
          printf("# MOVE AUTO STOP\r\n");
          debug_chassis_active = 0U;
      }

      if (current_tune_state == TUNE_STATE_RUN &&
          (uint32_t)(now - last_control_time) >= LLM_TUNE_CONTROL_PERIOD_MS)
      {
          float current_ops_x = robot_x;
          float current_ops_y = robot_y;
          float current_x;
          float current_y;
          float current_yaw = robot_yaw;
          float dx;
          float dy;
          float start_heading_rad;
          float body_right_mm;
          float body_forward_mm;
          float yaw_delta_deg;
          float normalized_input;
          float normalized_error;
          float yaw_error;
          float cross_track_mm;
          float target_value;
          float tolerance;
          float brake_limit;
          float desired_output;
          float hold_cross_output = 0.0f;
          float hold_yaw_output = 0.0f;
          float max_output_step;
          float command_vx = 0.0f;
          float command_vy = 0.0f;
          float command_vz = 0.0f;
          float V1, V2, V3, V4;
          PID_Controller *pid = LLM_GetTunePid();
          uint32_t last_ops_tick;

          /* OPS时间戳由中断更新：先快照它，再读取当前时间。 */
          last_ops_tick = ops9_last_update_tick;
          now = HAL_GetTick();
          last_control_time += LLM_TUNE_CONTROL_PERIOD_MS;
          if (!isfinite(current_ops_x) || !isfinite(current_ops_y) ||
              !isfinite(current_yaw) ||
              ops9_frame_count == 0U ||
              (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS) {
              LLM_StopTuneRound("OPS LOST");
              continue;
          }
          if ((uint32_t)(now - last_host_command_tick) > LLM_TUNE_HOST_TIMEOUT_MS) {
              LLM_StopTuneRound("HOST LOST");
              continue;
          }
          if ((uint32_t)(now - tune_start_time) >= LLM_TUNE_DURATION_MS) {
              LLM_StopTuneRound("TIMEOUT");
              continue;
          }

          LLM_OpsToChassisCenter(current_ops_x, current_ops_y, current_yaw,
                                 &current_x, &current_y);
          dx = current_x - start_x_pos;
          dy = current_y - start_y_pos;
          start_heading_rad = start_yaw_deg * (3.1415926f / 180.0f);
          body_right_mm = cosf(start_heading_rad) * dx + sinf(start_heading_rad) * dy;
          body_forward_mm = -sinf(start_heading_rad) * dx + cosf(start_heading_rad) * dy;
          yaw_delta_deg = LLM_AngleErrorDeg(current_yaw, start_yaw_deg);
          yaw_error = LLM_AngleErrorDeg(current_yaw, start_yaw_deg);

          if (current_tune_axis == TUNE_AXIS_X) {
              normalized_input = tune_direction * body_right_mm;
              cross_track_mm = body_forward_mm;
              target_value = LLM_TUNE_TARGET_MM;
              tolerance = LLM_TUNE_POSITION_TOL_MM;
          } else if (current_tune_axis == TUNE_AXIS_YAW) {
              normalized_input = tune_direction * yaw_delta_deg;
              cross_track_mm = sqrtf(dx * dx + dy * dy);
              target_value = LLM_TUNE_TARGET_YAW_DEG;
              tolerance = LLM_TUNE_YAW_TOL_DEG;
          } else {
              normalized_input = tune_direction * body_forward_mm;
              cross_track_mm = body_right_mm;
              target_value = LLM_TUNE_TARGET_MM;
              tolerance = LLM_TUNE_POSITION_TOL_MM;
          }
          normalized_error = target_value - normalized_input;

          /* 运动方向明显相反通常意味着轮序/极性错误，不能继续靠 PID 拉回。 */
          if (normalized_input < -(current_tune_axis == TUNE_AXIS_YAW ?
                                   3.0f : LLM_TUNE_WRONG_DIR_MM)) {
              LLM_StopTuneRound("WRONG DIR");
              continue;
          }

          /* 航向大幅偏离优先按底盘故障处理，避免用位置环硬压机械问题。 */
          if (current_tune_axis != TUNE_AXIS_YAW &&
              fabsf(yaw_error) > LLM_TUNE_MAX_YAW_ERROR_DEG) {
              LLM_StopTuneRound("YAW LIMIT");
              continue;
          }

          if (current_tune_axis == TUNE_AXIS_YAW) {
              if (cross_track_mm > LLM_TUNE_YAW_TRANSLATION_MM) {
                  LLM_StopTuneRound("TRANSLATION LIMIT");
                  continue;
              }
          } else if (fabsf(cross_track_mm) > LLM_TUNE_CROSS_TRACK_MM) {
              LLM_StopTuneRound("CROSS TRACK");
              continue;
          }

          if (normalized_input > target_value +
              (current_tune_axis == TUNE_AXIS_YAW ? 10.0f : LLM_TUNE_OVERTRAVEL_MM)) {
              LLM_StopTuneRound("OVERTRAVEL");
              continue;
          }

          if (fabsf(normalized_error) <= tolerance) {
              tune_settle_cycles++;
              if (tune_settle_cycles >= LLM_TUNE_SETTLE_CYCLES) {
                  LLM_StopTuneRound("TARGET");
                  continue;
              }
          } else {
              tune_settle_cycles = 0U;
          }

          desired_output = PID_Calc(pid, normalized_input);
          if (current_tune_axis == TUNE_AXIS_YAW) {
              brake_limit = LLM_BrakeLimitYaw(normalized_error,
                                              LLM_TUNE_YAW_TOL_DEG);
          } else {
              brake_limit = LLM_BrakeLimitLinear(normalized_error,
                                                 LLM_TUNE_POSITION_TOL_MM);
          }
          if (brake_limit < pid->max_out) {
              desired_output = LLM_ClampFloat(desired_output, -brake_limit, brake_limit);
          }
          desired_output *= tune_direction;

          /* 软件斜坡限制每个20ms周期的速度变化，避免电机突然起停。 */
          max_output_step = (current_tune_axis == TUNE_AXIS_YAW ?
                            (fabsf(desired_output) < fabsf(tune_output) ?
                             LLM_TUNE_YAW_DECEL_RADPS2 : LLM_TUNE_YAW_ACCEL_RADPS2) :
                            (fabsf(desired_output) < fabsf(tune_output) ?
                             LLM_TUNE_MAX_DECEL_MPS2 : LLM_TUNE_MAX_ACCEL_MPS2)) *
                            ((float)LLM_TUNE_CONTROL_PERIOD_MS / 1000.0f);
          tune_output = LLM_Slew(tune_output, desired_output, max_output_step);

          if (current_tune_axis == TUNE_AXIS_X) {
              command_vx = tune_output;
              /*
               * X 横移时，Y 环把相对起点的车体前向位移压回 0，
               * YAW 环保持起始航向；保持量单独限幅，避免抢占主轴控制权。
               */
              hold_cross_output = PID_CalcError(&pid_y, -body_forward_mm);
              hold_yaw_output = PID_CalcError(&pid_yaw, -yaw_delta_deg);
              command_vy = LLM_ClampFloat(hold_cross_output,
                                          -LLM_TUNE_HOLD_LINEAR_MPS,
                                          LLM_TUNE_HOLD_LINEAR_MPS);
              command_vz = LLM_ClampFloat(hold_yaw_output,
                                          -LLM_TUNE_HOLD_YAW_RADPS,
                                          LLM_TUNE_HOLD_YAW_RADPS);
          } else if (current_tune_axis == TUNE_AXIS_Y) {
              command_vy = tune_output;
              /* Y 前进时对称地保持车体横向位移和起始航向。 */
              hold_cross_output = PID_CalcError(&pid_x, -body_right_mm);
              hold_yaw_output = PID_CalcError(&pid_yaw, -yaw_delta_deg);
              command_vx = LLM_ClampFloat(hold_cross_output,
                                          -LLM_TUNE_HOLD_LINEAR_MPS,
                                          LLM_TUNE_HOLD_LINEAR_MPS);
              command_vz = LLM_ClampFloat(hold_yaw_output,
                                          -LLM_TUNE_HOLD_YAW_RADPS,
                                          LLM_TUNE_HOLD_YAW_RADPS);
          } else {
              /*
               * 原地旋转时不启用 X/Y 保持：OPS 若偏离几何旋转中心会测到弧线位移，
               * 此时强行消除“平移”反而会给底盘注入真实的平移指令。
               */
              command_vz = tune_output;
          }
          Mecanum_Kinematics(command_vx, command_vy, command_vz, &V1, &V2, &V3, &V4);
          SetAllMotorsSpeed(V1, V2, V3, V4);
          printf("%lu,%.2f,%.2f,%.4f,%.2f,%.7f,%.8f,%.7f,"
                 "%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f\r\n",
                 (unsigned long)(now - tune_start_time),
                 target_value, normalized_input,
                 tune_direction * tune_output, normalized_error,
                 pid->Kp, pid->Ki, pid->Kd,
                 current_ops_x, current_ops_y, current_yaw,
                 cross_track_mm, yaw_delta_deg,
                 current_tune_axis == TUNE_AXIS_Y ? command_vx : command_vy,
                 command_vz, current_x, current_y);
      }

      HAL_Delay(1);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// CAN 接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    ZDT_CAN_RxFIFO0_Handler(hcan);
}
int _write(int file, char *ptr, int len)
{
    // 注意：假设你连接电脑的串口是 USART1。如果是其他串口，请修改 &huart1
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
// 定时器中断回调 (10ms 一次)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // 如果需要绘图，可以在这里置标志位
        // flag_plot_10ms = 1;
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 1. 处理 OPS-9 传感器数据 (USART2)
	    if (huart->Instance == USART2)
	    {
	        OPS9_UART_RxCpltCallback(huart);
	    }
	    // 2. 处理 PC 端大模型发来的指令 (USART1)
	    else if (huart->Instance == USART1)
	    {
	        // ISR 只组帧；浮点解析和状态切换放到主循环执行。
	        if (pc_rx_byte == '\n' || pc_rx_byte == '\r')
	        {
	            if (pc_rx_idx > 0U && !pc_command_ready)
	            {
	                uint8_t i;
	                pc_rx_buf[pc_rx_idx] = '\0';
	                for (i = 0U; i <= pc_rx_idx; i++) {
	                    pc_command_buf[i] = pc_rx_buf[i];
	                }
	                pc_command_ready = 1U;
	            }
	            pc_rx_idx = 0U;
	        }
	        else
	        {
	            if (!pc_command_ready && pc_rx_idx < sizeof(pc_rx_buf) - 1U)
	            {
	                pc_rx_buf[pc_rx_idx++] = pc_rx_byte;
	            }
	        }
	        // 必须重新开启中断，等待下一个字节
	        HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1);
	    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	extern uint8_t ops9_rx_byte;
    if (huart->Instance == USART2)
    {
        ops9_uart_error_count++;
        // 一旦检测到 USART2 报错（如 ORE 溢出），强行重新开启接收！
        HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
    }
    else if (huart->Instance == USART1)
    {
        pc_rx_idx = 0U;
        HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
