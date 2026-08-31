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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mecanum_chassis.h"
#include "ops9.h"
#include "pid.h"
#include "llm_tuner.h"
#include "dm_g6220.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POSE_POSITION_TOL_MM    2.0f
#define POSE_YAW_TOL_DEG        0.5f
#define POSE_SETTLE_CYCLES      25U
#define POSE_CONTROL_PERIOD_MS  20U
#define POSE_DT_MAX_MS          100U
#define POSE_TUNE_TIMEOUT_MS    15000U
#define POSE_WORK_TIMEOUT_MS    35000U
#define POSE_STOP_LINEAR_EPS_MPS 0.005f
#define POSE_STOP_YAW_EPS_RADPS  0.010f
/* POSE 平移规划参数：速度 m/s，加/减速度 m/s^2。 */
#define POSE_SPEED_DEFAULT_MPS       0.15f
#define POSE_SPEED_MIN_MPS           0.02f
#define POSE_SPEED_HARD_MAX_MPS      0.30f
#define POSE_ACCEL_DEFAULT_MPS2      0.20f
#define POSE_ACCEL_MIN_MPS2          0.05f
#define POSE_ACCEL_HARD_MAX_MPS2     0.80f
#define POSE_DECEL_DEFAULT_MPS2      0.40f
#define POSE_DECEL_MIN_MPS2          0.05f
#define POSE_DECEL_HARD_MAX_MPS2     1.20f
/* POSE 航向规划参数：速度 rad/s，加/减速度 rad/s^2。 */
#define POSE_YAW_SPEED_DEFAULT_RADPS     0.30f
#define POSE_YAW_SPEED_MIN_RADPS         0.02f
#define POSE_YAW_SPEED_HARD_MAX_RADPS    0.80f
#define POSE_YAW_ACCEL_DEFAULT_RADPS2    0.50f
#define POSE_YAW_ACCEL_MIN_RADPS2        0.10f
#define POSE_YAW_ACCEL_HARD_MAX_RADPS2   2.00f
#define POSE_YAW_DECEL_DEFAULT_RADPS2    0.80f
#define POSE_YAW_DECEL_MIN_RADPS2        0.10f
#define POSE_YAW_DECEL_HARD_MAX_RADPS2   3.00f
/* 单个 POSE 航点相对当前车体中心的硬行程边界，防止错误坐标导致长距离失控。 */
#define POSE_TARGET_DISTANCE_HARD_MAX_MM 10000.0f
#define POSE_RUNTIME_ERROR_HARD_MAX_MM   10500.0f
#define OPS_CENTER_OFFSET_X_MM      0.0f
#define OPS_CENTER_OFFSET_Y_MM      25.0f
#define DEBUG_MOTOR_MAX_RPM         300.0f
#define DEBUG_MOTOR_DEFAULT_MS      2000UL
#define DEBUG_MOTOR_MAX_MS          10000UL
#define DEBUG_MOVE_DEFAULT_MPS      0.04f
#define DEBUG_MOVE_MAX_MPS          0.08f
#define DEBUG_MOVE_DEFAULT_MS       1000UL
#define DEBUG_MOVE_MAX_MS           3000UL
#define DEBUG_TURN_DEFAULT_RADPS     0.15f
#define DEBUG_TURN_MAX_RADPS         0.30f
#define TELEMETRY_PERIOD_MS          50U
#define TELEMETRY_STAGGER_MS         25U
#define TELEMETRY_MASK_WHEEL         0x01U
#define TELEMETRY_MASK_POSE          0x02U
#define TELEMETRY_MASK_BOTH          (TELEMETRY_MASK_WHEEL | TELEMETRY_MASK_POSE)
#define MOTOR_FEEDBACK_POLL_MS       10U
#define HOST_PROTOCOL_VERSION        3U
#define HOST_UART_TX_TIMEOUT_MS      20U
#define HOST_WAIT_TIMEOUT_MS         60000U
#define G6220_CAN_ID                 0x01U
#define G6220_MASTER_ID              0x00U
#define G6220_CAN2_FILTER_BANK       14U
#define G6220_SLAVE_FILTER_START     14U
#define G6220_STARTUP_DELAY_MS       1000U
#define G6220_COMMAND_DELAY_MS       50U

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
// === 主机串口命令接收状态 ===
uint8_t pc_rx_byte;                       // 主机串口单字节接收
char pc_rx_buf[64];                       // ISR 正在拼接的命令
char pc_command_buf[64];                  // 主循环待处理的完整命令
volatile uint8_t pc_rx_idx = 0;
volatile uint8_t pc_command_ready = 0;

typedef enum {
    ROBOT_MODE_WORK = 0,
    ROBOT_MODE_TUNE,
    ROBOT_MODE_PLOT
} RobotMode_t;

typedef enum {
    HOST_LINK_NONE = 0,
    HOST_LINK_COM,
    HOST_LINK_RPI
} HostLink_t;

typedef enum {
    POSE_PHASE_IDLE = 0,
    POSE_PHASE_TRANSLATE,
    POSE_PHASE_ROTATE
} PoseControlPhase_t;

typedef enum {
    CHASSIS_MOTION_NONE = 0,
    CHASSIS_MOTION_POSE,
    CHASSIS_MOTION_TUNE_ROUND,
    CHASSIS_MOTION_DEBUG_CHASSIS,
    CHASSIS_MOTION_SINGLE_MOTOR
} ChassisMotionType_t;

typedef struct {
    float linear_accel_mps2;
    float linear_decel_mps2;
    float yaw_accel_radps2;
    float yaw_decel_radps2;
} PoseMotionProfile_t;

RobotMode_t current_robot_mode = ROBOT_MODE_WORK;
uint8_t debug_motor_active = 0U;
uint8_t debug_motor_id = 0U;
uint32_t debug_motor_stop_tick = 0U;
uint8_t debug_chassis_active = 0U;
uint32_t debug_chassis_stop_tick = 0U;
uint8_t ops_monitor_enabled = 0U;
uint32_t ops_monitor_last_tick = 0U;
uint32_t last_host_command_tick = 0U;
HostLink_t active_host_link = HOST_LINK_NONE;
uint32_t host_wait_start_tick = 0U;
uint8_t host_wait_timeout_reported = 0U;
uint8_t chassis_motors_enabled = 0U;
uint8_t pose_control_active = 0U;
PoseControlPhase_t pose_control_phase = POSE_PHASE_IDLE;
float pose_target_x_mm = 0.0f;
float pose_target_y_mm = 0.0f;
float pose_target_yaw_deg = 0.0f;
float pose_translation_yaw_deg = 0.0f;
float pose_target_center_x_mm = 0.0f;
float pose_target_center_y_mm = 0.0f;
float pose_output_vx = 0.0f;
float pose_output_vy = 0.0f;
float pose_output_vz = 0.0f;
float pose_target_vx = 0.0f;
float pose_target_vy = 0.0f;
float pose_target_vz = 0.0f;
uint16_t pose_settle_cycles = 0U;
uint32_t pose_last_control_time = 0U;
uint32_t pose_start_time = 0U;
PoseMotionProfile_t pose_motion_profile;
uint8_t telemetry_mask = 0U;
uint8_t telemetry_next_group = TELEMETRY_MASK_WHEEL;
uint16_t telemetry_wheel_sequence = 0U;
uint16_t telemetry_pose_sequence = 0U;
uint32_t telemetry_last_tick = 0U;
uint32_t telemetry_tx_ok = 0U;
uint32_t telemetry_tx_error = 0U;
uint8_t motor_feedback_poll_id = 1U;
uint32_t motor_feedback_poll_tick = 0U;
volatile uint32_t host_uart_tx_ok = 0U;
volatile uint32_t host_uart_tx_error = 0U;
DM_G6220_Motor_t g6220_motor;
uint8_t g6220_initialized = 0U;
uint8_t g6220_enable_requested = 0U;
DM_G6220_Result_t g6220_last_result = DM_G6220_ERROR_PARAM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Host_ProcessCommand(void);
static uint8_t Host_ProcessOperationalCommand(const char *command);
static void Motor_ProcessFeedback(void);
static void Host_PrintHelp(void);
static void Ops_PrintStatus(void);
static float Pose_AngleErrorDeg(float current_deg, float reference_deg);
static void Pose_OpsToChassisCenter(float ops_x_mm, float ops_y_mm, float yaw_deg,
                                    float *center_x_mm, float *center_y_mm);
static void Pose_ProcessControl(uint32_t now);
static void Telemetry_Process(uint32_t now);
static void Motor_ProcessFeedbackPolling(uint32_t now);
static void ChassisSafety_Process(uint32_t now);
static const char *RobotMode_Name(RobotMode_t mode);
static DM_G6220_Result_t G6220_SetEnabled(uint8_t enable);
static void Pose_InitMotionProfile(void);
static void Pose_ResetPlanner(void);
static void Robot_StopAllMotion(void);
static void Robot_SetMode(RobotMode_t mode);
static const char *HostLink_Name(HostLink_t link);
static uint8_t HostLink_ProcessCommand(const char *command);
static void HostLink_ProcessWait(uint32_t now);
static void HostLink_SetChassisEnabled(uint8_t enable);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t Motor_IsValidId(unsigned int id)
{
    return (id >= 1U && id <= 4U) ? 1U : 0U;
}

static const char *RobotMode_Name(RobotMode_t mode)
{
    if (mode == ROBOT_MODE_TUNE) return "TUNE";
    if (mode == ROBOT_MODE_PLOT) return "PLOT";
    return "WORK";
}

static const char *HostLink_Name(HostLink_t link)
{
    if (link == HOST_LINK_COM) return "COM";
    if (link == HOST_LINK_RPI) return "RPI";
    return "NONE";
}

static DM_G6220_Result_t G6220_SetEnabled(uint8_t enable)
{
    if (!g6220_initialized) {
        g6220_last_result = DM_G6220_ERROR_PARAM;
        return g6220_last_result;
    }

    g6220_last_result = DM_G6220_SendCommand(
        &g6220_motor,
        enable ? DM_G6220_CMD_ENABLE : DM_G6220_CMD_DISABLE);
    if (g6220_last_result == DM_G6220_OK) {
        g6220_enable_requested = enable ? 1U : 0U;
    }
    return g6220_last_result;
}

static void Robot_StopAllMotion(void)
{
    (void)StopAllMotors();
    debug_motor_active = 0U;
    debug_chassis_active = 0U;
    pose_control_active = 0U;
    Pose_ResetPlanner();
    LLM_TunerAbort();
    (void)G6220_SetEnabled(0U);
    /* PID_Reset 只清运行历史，不修改已经调好的 Kp/Ki/Kd 和输出限幅。 */
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    PID_Reset(&pid_yaw);
}

static void HostLink_SetChassisEnabled(uint8_t enable)
{
    uint8_t id;
    uint8_t all_ok = 1U;
    uint8_t result;

    /* 四轮闭环驱动器按顺序切换使能状态；使能且零速时提供静止保持力矩。 */
    for (id = 1U; id <= 4U; id++) {
        result = ZDT_Emm_EnableSingleMotor(id, enable);
        Mecanum_ReportCanTxResult(result);
        if (result != 0U) all_ok = 0U;
        HAL_Delay(10U);
    }
    chassis_motors_enabled = (enable && all_ok) ? 1U : 0U;
}

static uint8_t HostLink_ProcessCommand(const char *command)
{
    HostLink_t requested = HOST_LINK_NONE;

    if (strcmp(command, "HOST LINK COM") == 0) requested = HOST_LINK_COM;
    else if (strcmp(command, "HOST LINK RPI") == 0) requested = HOST_LINK_RPI;
    else if (strncmp(command, "HOST LINK ", 10U) == 0) {
        printf("# ERROR HOST LINK COM|RPI\r\n");
        return 1U;
    }
    else if (strcmp(command, "HOST STATUS") == 0) {
        printf("# HOST STATUS STATE=%s OWNER=%s MOTOR_EN=%u HEARTBEAT=%s WAIT_MS=%lu TIMEOUT_MS=%lu\r\n",
               active_host_link == HOST_LINK_NONE ? "WAITING" : "LINKED",
               HostLink_Name(active_host_link),
               chassis_motors_enabled,
               active_host_link == HOST_LINK_RPI ? "REQUIRED" : "OFF",
               (unsigned long)(HAL_GetTick() - host_wait_start_tick),
               (unsigned long)HOST_WAIT_TIMEOUT_MS);
        return 1U;
    } else {
        return 0U;
    }

    /* 每次声明或切换主机都先停车、清旧状态，绝不恢复上一个主机的目标。 */
    Robot_StopAllMotion();
    LLM_TunerResetSession();
    telemetry_mask = 0U;
    current_robot_mode = ROBOT_MODE_WORK;
    active_host_link = requested;
    last_host_command_tick = HAL_GetTick();
    if (!chassis_motors_enabled) HostLink_SetChassisEnabled(1U);
    printf("# HOST LINK %s OK HEARTBEAT=%s\r\n",
           HostLink_Name(active_host_link),
           active_host_link == HOST_LINK_RPI ? "REQUIRED" : "OFF");
    return 1U;
}

static void HostLink_ProcessWait(uint32_t now)
{
    if (active_host_link != HOST_LINK_NONE || host_wait_timeout_reported) return;
    if ((uint32_t)(now - host_wait_start_tick) >= HOST_WAIT_TIMEOUT_MS) {
        host_wait_timeout_reported = 1U;
        printf("# HOST WAIT TIMEOUT STATE=WAITING\r\n");
    }
}

static void Robot_SetMode(RobotMode_t mode)
{
    if (current_robot_mode == mode) {
        if (mode == ROBOT_MODE_PLOT) telemetry_mask = TELEMETRY_MASK_BOTH;
        if (mode == ROBOT_MODE_WORK) {
            (void)G6220_SetEnabled(1U);
        }
        printf("# MODE %s PLOT=%u CHANGED=0\r\n",
               RobotMode_Name(current_robot_mode), telemetry_mask != 0U);
        return;
    }

    /* 模式切换是安全边界：先停掉旧模式的一切运动，再启用新模式输出。 */
    Robot_StopAllMotion();
    LLM_TunerResetSession();
    current_robot_mode = mode;

    if (mode == ROBOT_MODE_PLOT) {
        telemetry_mask = TELEMETRY_MASK_BOTH;
        telemetry_last_tick = HAL_GetTick();
    } else if (mode == ROBOT_MODE_WORK) {
        (void)G6220_SetEnabled(1U);
    }

    printf("# MODE %s PLOT=%u CHANGED=1 MOTION=STOPPED\r\n",
           RobotMode_Name(current_robot_mode), telemetry_mask != 0U);
}

static ChassisMotionType_t ChassisSafety_GetActiveMotion(void)
{
    if (LLM_TunerIsRunning()) return CHASSIS_MOTION_TUNE_ROUND;
    if (pose_control_active) return CHASSIS_MOTION_POSE;
    if (debug_chassis_active) return CHASSIS_MOTION_DEBUG_CHASSIS;
    if (debug_motor_active) return CHASSIS_MOTION_SINGLE_MOTOR;
    return CHASSIS_MOTION_NONE;
}

static const char *ChassisSafety_MotionName(ChassisMotionType_t motion)
{
    if (motion == CHASSIS_MOTION_POSE) return "POSE";
    if (motion == CHASSIS_MOTION_TUNE_ROUND) return "TUNE";
    if (motion == CHASSIS_MOTION_DEBUG_CHASSIS) return "DEBUG_CHASSIS";
    if (motion == CHASSIS_MOTION_SINGLE_MOTOR) return "SINGLE_MOTOR";
    return "NONE";
}

static uint8_t ChassisSafety_CanReady(void)
{
    return (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING &&
            HAL_CAN_GetError(&hcan1) == HAL_CAN_ERROR_NONE) ? 1U : 0U;
}

static uint8_t ChassisSafety_OpsReady(uint32_t now)
{
    uint32_t last_ops_tick = ops9_last_update_tick;
    return (isfinite(robot_x) && isfinite(robot_y) && isfinite(robot_yaw) &&
            ops9_frame_count > 0U &&
            (uint32_t)(now - last_ops_tick) <= LLM_TUNE_OPS_TIMEOUT_MS) ? 1U : 0U;
}

static void ChassisSafety_Stop(ChassisMotionType_t motion, const char *reason)
{
    debug_motor_active = 0U;
    debug_chassis_active = 0U;
    pose_control_active = 0U;
    Pose_ResetPlanner();
    (void)G6220_SetEnabled(0U);
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    PID_Reset(&pid_yaw);

    if (motion == CHASSIS_MOTION_TUNE_ROUND && LLM_TunerIsRunning()) {
        LLM_TunerStopRound(reason);
        return;
    }

    LLM_TunerAbort();
    (void)StopAllMotors();
    if (motion == CHASSIS_MOTION_POSE) {
        printf("# POSE STOP SAFETY REASON=%s\r\n", reason);
    } else {
        printf("# MOTION STOP SAFETY TYPE=%s REASON=%s\r\n",
               ChassisSafety_MotionName(motion), reason);
    }
}

static void ChassisSafety_Process(uint32_t now)
{
    ChassisMotionType_t motion = ChassisSafety_GetActiveMotion();
    uint8_t needs_ops;
    uint8_t needs_host;

    if (motion == CHASSIS_MOTION_NONE) return;

    /* CAN状态和四轮实际发送结果对所有运动类型都是共同的硬安全边界。 */
    if (!ChassisSafety_CanReady() ||
        Mecanum_ConsumeCanTxFault()) {
        ChassisSafety_Stop(motion, "CAN FAULT");
        return;
    }

    /* 单电机悬空台架测试不依赖OPS；其余底盘运动都要求位姿链路有效。 */
    needs_ops = (motion != CHASSIS_MOTION_SINGLE_MOTOR) ? 1U : 0U;
    if (needs_ops && !ChassisSafety_OpsReady(now)) {
        ChassisSafety_Stop(motion, "OPS LOST");
        return;
    }

    /* 只有 RPI 在 WORK/PLOT 运动时要求心跳；COM 调试由操作者直接看护。 */
    needs_host = (active_host_link == HOST_LINK_RPI &&
                  current_robot_mode != ROBOT_MODE_TUNE) ? 1U : 0U;
    if (needs_host &&
        (uint32_t)(now - last_host_command_tick) > LLM_TUNE_HOST_TIMEOUT_MS) {
        ChassisSafety_Stop(motion, "HOST LOST");
        return;
    }

    if (motion == CHASSIS_MOTION_POSE) {
        uint32_t timeout_ms = (current_robot_mode == ROBOT_MODE_TUNE) ?
                              POSE_TUNE_TIMEOUT_MS : POSE_WORK_TIMEOUT_MS;
        if ((uint32_t)(now - pose_start_time) > timeout_ms) {
            ChassisSafety_Stop(motion, "TIMEOUT");
        }
    }
}

static float Pose_AngleErrorDeg(float current_deg, float reference_deg)
{
    float error = fmodf(current_deg - reference_deg + 180.0f, 360.0f);
    if (error < 0.0f) {
        error += 360.0f;
    }
    return error - 180.0f;
}

static void Pose_OpsToChassisCenter(float ops_x_mm, float ops_y_mm, float yaw_deg,
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

static float Math_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static void Pose_InitMotionProfile(void)
{
    /* 默认值也经过硬边界裁剪，避免以后改宏时越过机械安全范围。 */
    pose_motion_profile.linear_accel_mps2 =
        Math_ClampFloat(POSE_ACCEL_DEFAULT_MPS2,
                        POSE_ACCEL_MIN_MPS2,
                        POSE_ACCEL_HARD_MAX_MPS2);
    pose_motion_profile.linear_decel_mps2 =
        Math_ClampFloat(POSE_DECEL_DEFAULT_MPS2,
                        POSE_DECEL_MIN_MPS2,
                        POSE_DECEL_HARD_MAX_MPS2);
    pose_motion_profile.yaw_accel_radps2 =
        Math_ClampFloat(POSE_YAW_ACCEL_DEFAULT_RADPS2,
                        POSE_YAW_ACCEL_MIN_RADPS2,
                        POSE_YAW_ACCEL_HARD_MAX_RADPS2);
    pose_motion_profile.yaw_decel_radps2 =
        Math_ClampFloat(POSE_YAW_DECEL_DEFAULT_RADPS2,
                        POSE_YAW_DECEL_MIN_RADPS2,
                        POSE_YAW_DECEL_HARD_MAX_RADPS2);
}

static void Pose_ResetPlanner(void)
{
    pose_control_phase = POSE_PHASE_IDLE;
    pose_target_vx = 0.0f;
    pose_target_vy = 0.0f;
    pose_target_vz = 0.0f;
    pose_output_vx = 0.0f;
    pose_output_vy = 0.0f;
    pose_output_vz = 0.0f;
    pose_settle_cycles = 0U;
    pose_last_control_time = HAL_GetTick();
}

static float Pose_SlewScalar(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static void Pose_SlewVector2D(float current_x, float current_y,
                              float target_x, float target_y,
                              float accel_mps2, float decel_mps2, float dt_s,
                              float *output_x, float *output_y)
{
    float current_speed = sqrtf(current_x * current_x + current_y * current_y);
    float target_projection = 0.0f;
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;
    float delta_speed = sqrtf(delta_x * delta_x + delta_y * delta_y);
    float rate = accel_mps2;
    float max_delta;

    if (current_speed > 0.0001f) {
        target_projection = (current_x * target_x + current_y * target_y) /
                            current_speed;
        /* 反向、转弯或目标在当前速度方向上的投影变小，都按减速度约束。 */
        if (target_projection < current_speed) {
            rate = decel_mps2;
        }
    }

    max_delta = rate * dt_s;
    if (delta_speed > max_delta && delta_speed > 0.0001f) {
        *output_x = current_x + delta_x * max_delta / delta_speed;
        *output_y = current_y + delta_y * max_delta / delta_speed;
    } else {
        *output_x = target_x;
        *output_y = target_y;
    }
}

static float Pose_BrakeLimitLinear(float distance_mm, float tolerance_mm)
{
    float remaining_m = (distance_mm - tolerance_mm) / 1000.0f;
    if (remaining_m <= 0.0f) return 0.0f;
    return sqrtf(2.0f * pose_motion_profile.linear_decel_mps2 * remaining_m);
}

static float Pose_BrakeLimitYaw(float error_deg, float tolerance_deg)
{
    float remaining_rad = (fabsf(error_deg) - tolerance_deg) *
                          (3.1415926f / 180.0f);
    if (remaining_rad <= 0.0f) return 0.0f;
    return sqrtf(2.0f * pose_motion_profile.yaw_decel_radps2 * remaining_rad);
}

static void Host_PrintHelp(void)
{
    printf("# HELP HOST LINK COM|RPI | HOST STATUS (COM no heartbeat; RPI heartbeat required)\r\n");
    printf("# HELP PROTO VERSION | MODE WORK|TUNE|PLOT | MODE STATUS (default WORK; switching stops motion)\r\n");
    printf("# HELP STATUS | PING | STOP | RESET | OPS STATUS | OPS MONITOR ON|OFF | OPS ZERO\r\n");
    printf("# HELP PROTO EMM|X | CAN STATUS | MOTOR EN|DIS <id>\r\n");
    printf("# HELP MOTOR RUN <id> <signed_rpm> [ms] | MOTOR STOP <id>|ALL | MOTOR GET <id>\r\n");
    printf("# HELP MOVE FWD|BACK|LEFT|RIGHT [mps] [ms] | TURN CW|CCW [radps] [ms] | MOVE STOP\r\n");
    printf("# HELP POSE SET <x_mm> <y_mm> <yaw_deg> | POSE STOP | POSE STATUS\r\n");
    printf("# HELP TUNE AXIS X|Y|YAW | TUNE LIMIT <mps_or_radps> | NO PING ROUND=5S POSE=15S ROUNDS=20\r\n");
    printf("# HELP PID SET X|Y|YAW <p> <i> <d> | PID LIMIT X|Y|YAW <value> | PID STATUS ALL\r\n");
    printf("# HELP G6220 STATUS | G6220 ENABLE | G6220 DISABLE\r\n");
    printf("# HELP TELEM OFF|WHEEL|POSE|BOTH|STATUS (two tagged 8-channel groups, 20Hz)\r\n");
    printf("# HELP PLOT ON|OFF|STATUS (legacy alias for MODE PLOT + TELEM BOTH)\r\n");
    printf("# LIMIT motor id=1..4 rpm=+/-%.0f duration=100..%lu ms\r\n",
           DEBUG_MOTOR_MAX_RPM, DEBUG_MOTOR_MAX_MS);
    printf("# LIMIT move speed=0..%.2f mps duration=100..%lu ms; FWD=+Y LEFT=-X\r\n",
           DEBUG_MOVE_MAX_MPS, DEBUG_MOVE_MAX_MS);
    printf("# LIMIT turn speed=0..%.2f radps duration=100..%lu ms\r\n",
           DEBUG_TURN_MAX_RADPS, DEBUG_MOVE_MAX_MS);
}

static void Motor_ProcessFeedbackPolling(uint32_t now)
{
    uint8_t result;

    if ((telemetry_mask & TELEMETRY_MASK_WHEEL) == 0U ||
        (uint32_t)(now - motor_feedback_poll_tick) < MOTOR_FEEDBACK_POLL_MS) {
        return;
    }

    motor_feedback_poll_tick = now;
    result = ZDT_Emm_ReadSpeedByID(motor_feedback_poll_id);
    Mecanum_ReportCanTxResult(result);
    motor_feedback_poll_id++;
    if (motor_feedback_poll_id > 4U) motor_feedback_poll_id = 1U;
}

static void Telemetry_Process(uint32_t now)
{
    uint8_t group;
    uint32_t period_ms;
    int written;

    if (telemetry_mask == 0U) return;
    period_ms = (telemetry_mask == TELEMETRY_MASK_BOTH) ?
                TELEMETRY_STAGGER_MS : TELEMETRY_PERIOD_MS;
    if ((uint32_t)(now - telemetry_last_tick) < period_ms) return;
    telemetry_last_tick = now;

    if (telemetry_mask == TELEMETRY_MASK_BOTH) {
        group = telemetry_next_group;
        telemetry_next_group = (group == TELEMETRY_MASK_WHEEL) ?
                               TELEMETRY_MASK_POSE : TELEMETRY_MASK_WHEEL;
    } else {
        group = telemetry_mask;
    }

    if (group == TELEMETRY_MASK_WHEEL) {
        written = printf("@W,1,%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                         (unsigned long)now, telemetry_wheel_sequence++,
                         motors[0].target_speed, motors[1].target_speed,
                         motors[2].target_speed, motors[3].target_speed,
                         motors[0].actual_speed, motors[1].actual_speed,
                         motors[2].actual_speed, motors[3].actual_speed);
    } else {
        float center_x;
        float center_y;
        Pose_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
        written = printf("@P,1,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\r\n",
                         (unsigned long)now, telemetry_pose_sequence++,
                         robot_x, robot_y, robot_yaw, center_x, center_y,
                         pose_control_active ? pose_output_vx : 0.0f,
                         pose_control_active ? pose_output_vy : 0.0f,
                         pose_control_active ? pose_output_vz : 0.0f);
    }

    if (written > 0) telemetry_tx_ok++;
    else telemetry_tx_error++;
}

static void Ops_PrintStatus(void)
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

    Pose_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
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

static void Motor_ProcessFeedback(void)
{
    ZDT_MotorEvent_t event;
    while (ZDT_Emm_PollEvent(&event)) {
        if (event.function_code == 0x35U) {
            /* 周期轮询由@W遥测承载；仅在未开四轮遥测时打印人工MOTOR GET回复。 */
            if ((telemetry_mask & TELEMETRY_MASK_WHEEL) == 0U) {
                printf("# MOTOR SPEED ID=%u RPM=%.1f\r\n",
                       event.motor_id, event.speed_rpm);
            }
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

static uint8_t Host_ProcessOperationalCommand(const char *command)
{
    unsigned int id;
    float rpm = 0.0f;
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
    float target_distance_mm;
    int fields;
    uint8_t result_a;
    uint8_t result_b;
    DM_G6220_Feedback_t g6220_feedback;
    DM_G6220_Result_t g6220_result;

    if (strcmp(command, "PING") == 0) {
        printf("# PONG\r\n");
        return 1U;
    }

    if (strcmp(command, "HELP") == 0) {
        Host_PrintHelp();
        return 1U;
    }

    if (strcmp(command, "PROTO VERSION") == 0) {
        printf("# PROTO VERSION=%u MODES=WORK,TUNE,PLOT LEGACY_POSE=1 HOST_LINK=REQUIRED\r\n",
               HOST_PROTOCOL_VERSION);
        return 1U;
    }

    if (strcmp(command, "MODE STATUS") == 0) {
        printf("# MODE %s PLOT=%u TUNE_STATE=%u POSE_ACTIVE=%u\r\n",
               RobotMode_Name(current_robot_mode), telemetry_mask != 0U,
               (unsigned int)LLM_TunerGetState(), pose_control_active);
        return 1U;
    }

    if (strcmp(command, "MODE WORK") == 0) {
        Robot_SetMode(ROBOT_MODE_WORK);
        return 1U;
    }

    if (strcmp(command, "MODE TUNE") == 0) {
        Robot_SetMode(ROBOT_MODE_TUNE);
        return 1U;
    }

    if (strcmp(command, "MODE PLOT") == 0) {
        Robot_SetMode(ROBOT_MODE_PLOT);
        printf("# PLOT ON LEGACY=1 TELEM=BOTH FORMAT=TAGGED_ASCII GROUPS=W8,P8 PERIOD=50MS\r\n");
        return 1U;
    }

    if (strcmp(command, "PLOT ON") == 0) {
        /* 兼容旧命令：PLOT ON 等价于安全切换到独立 PLOT 模式。 */
        Robot_SetMode(ROBOT_MODE_PLOT);
        printf("# PLOT ON LEGACY=1 TELEM=BOTH FORMAT=TAGGED_ASCII GROUPS=W8,P8 PERIOD=50MS\r\n");
        return 1U;
    }

    if (strcmp(command, "PLOT OFF") == 0) {
        uint32_t tx_ok = telemetry_tx_ok;
        uint32_t tx_error = telemetry_tx_error;
        telemetry_mask = 0U;
        Robot_SetMode(ROBOT_MODE_WORK);
        printf("# PLOT OFF TX_OK=%lu TX_ERR=%lu\r\n",
               (unsigned long)tx_ok, (unsigned long)tx_error);
        return 1U;
    }

    if (strcmp(command, "PLOT STATUS") == 0) {
        printf("# PLOT ENABLED=%u MODE=%s LEGACY=1 TELEM_MASK=%u "
               "FORMAT=TAGGED_ASCII GROUPS=W8,P8 PERIOD=50MS TX_OK=%lu TX_ERR=%lu\r\n",
               telemetry_mask != 0U, RobotMode_Name(current_robot_mode),
               telemetry_mask, (unsigned long)telemetry_tx_ok,
               (unsigned long)telemetry_tx_error);
        return 1U;
    }

    if (strcmp(command, "TELEM STATUS") == 0) {
        printf("# TELEM MASK=%u WHEEL=%u POSE=%u PERIOD=50MS FORMAT=TAGGED_ASCII "
               "W_SEQ=%u P_SEQ=%u TX_OK=%lu TX_ERR=%lu\r\n",
               telemetry_mask,
               (telemetry_mask & TELEMETRY_MASK_WHEEL) != 0U,
               (telemetry_mask & TELEMETRY_MASK_POSE) != 0U,
               telemetry_wheel_sequence, telemetry_pose_sequence,
               (unsigned long)telemetry_tx_ok,
               (unsigned long)telemetry_tx_error);
        return 1U;
    }

    if (strcmp(command, "TELEM OFF") == 0 ||
        strcmp(command, "TELEM WHEEL") == 0 ||
        strcmp(command, "TELEM POSE") == 0 ||
        strcmp(command, "TELEM BOTH") == 0) {
        if (strcmp(command, "TELEM OFF") == 0) telemetry_mask = 0U;
        else if (strcmp(command, "TELEM WHEEL") == 0) telemetry_mask = TELEMETRY_MASK_WHEEL;
        else if (strcmp(command, "TELEM POSE") == 0) telemetry_mask = TELEMETRY_MASK_POSE;
        else telemetry_mask = TELEMETRY_MASK_BOTH;
        telemetry_last_tick = HAL_GetTick();
        telemetry_next_group = TELEMETRY_MASK_WHEEL;
        motor_feedback_poll_tick = telemetry_last_tick;
        printf("# TELEM MASK=%u WHEEL=%u POSE=%u FORMAT=TAGGED_ASCII GROUPS=W8,P8\r\n",
               telemetry_mask,
               (telemetry_mask & TELEMETRY_MASK_WHEEL) != 0U,
               (telemetry_mask & TELEMETRY_MASK_POSE) != 0U);
        return 1U;
    }

    if (strcmp(command, "HELP") == 0) {
        Host_PrintHelp();
        return 1U;
    }

    if (strcmp(command, "PROTO EMM") == 0 || strcmp(command, "PROTO X") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
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

    if (strcmp(command, "G6220 STATUS") == 0) {
        (void)DM_G6220_GetFeedback(&g6220_motor, &g6220_feedback);
        printf("# G6220 INIT=%u ENABLE_REQ=%u CAN_STATE=%u CAN_ERR=0x%08lX "
               "TX_OK=%lu TX_ERR=%lu RX_OK=%lu RX_IGN=%lu LAST=%u ",
               g6220_initialized, g6220_enable_requested,
               (unsigned int)HAL_CAN_GetState(&hcan2),
               (unsigned long)HAL_CAN_GetError(&hcan2),
               (unsigned long)g6220_motor.tx_ok,
               (unsigned long)g6220_motor.tx_error,
               (unsigned long)g6220_motor.rx_ok,
               (unsigned long)g6220_motor.rx_ignored,
               (unsigned int)g6220_last_result);
        if (g6220_motor.rx_ok > 0U) {
            printf("STATE=%u POS=%.4f VEL=%.4f TORQUE=%.3f MOS=%.1f ROTOR=%.1f AGE=%luMS\r\n",
                   (unsigned int)g6220_feedback.state,
                   g6220_feedback.position_rad,
                   g6220_feedback.velocity_radps,
                   g6220_feedback.torque_nm,
                   g6220_feedback.mos_temperature_c,
                   g6220_feedback.rotor_temperature_c,
                   (unsigned long)(HAL_GetTick() - g6220_feedback.update_tick_ms));
        } else {
            printf("STATE=NO_FEEDBACK\r\n");
        }
        return 1U;
    }

    if (strcmp(command, "G6220 ENABLE") == 0) {
        if (current_robot_mode != ROBOT_MODE_WORK) {
            printf("# ERROR G6220 ENABLE REQUIRES MODE WORK\r\n");
        } else {
            g6220_result = G6220_SetEnabled(1U);
            printf("# G6220 ENABLE RESULT=%u\r\n", (unsigned int)g6220_result);
        }
        return 1U;
    }

    if (strcmp(command, "G6220 DISABLE") == 0) {
        g6220_result = G6220_SetEnabled(0U);
        printf("# G6220 DISABLE RESULT=%u\r\n", (unsigned int)g6220_result);
        return 1U;
    }

    if (strcmp(command, "OPS STATUS") == 0) {
        Ops_PrintStatus();
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
        Pose_ResetPlanner();
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        /* 保持现有取消协议语义：收到 POSE STOP 后立即确认已经零速。 */
        StopAllMotors();
        printf("# POSE STOP\r\n");
        return 1U;
    }

    if (strcmp(command, "POSE STATUS") == 0) {
        Pose_OpsToChassisCenter(robot_x, robot_y, robot_yaw, &center_x, &center_y);
        printf("# POSE ACTIVE=%u TARGET_X=%.2f TARGET_Y=%.2f TARGET_YAW=%.2f "
               "TARGET_CENTER_X=%.2f TARGET_CENTER_Y=%.2f "
               "X=%.2f Y=%.2f YAW=%.2f CENTER_X=%.2f CENTER_Y=%.2f "
               "CMD_VX=%.3f CMD_VY=%.3f CMD_VZ=%.3f "
               "PLAN_VX=%.3f PLAN_VY=%.3f PLAN_VZ=%.3f\r\n",
               pose_control_active, pose_target_x_mm, pose_target_y_mm,
               pose_target_yaw_deg, pose_target_center_x_mm, pose_target_center_y_mm,
               robot_x, robot_y, robot_yaw, center_x, center_y,
               pose_target_vx, pose_target_vy, pose_target_vz,
               pose_output_vx, pose_output_vy, pose_output_vz);
        return 1U;
    }

    if (sscanf(command, "POSE SET %f %f %f", &pose_x, &pose_y, &pose_yaw) == 3) {
        uint32_t last_ops_tick = ops9_last_update_tick;
        uint32_t now = HAL_GetTick();
        if (!isfinite(pose_x) || !isfinite(pose_y) || !isfinite(pose_yaw)) {
            pose_control_active = 0U;
            Pose_ResetPlanner();
            StopAllMotors();
            printf("# ERROR POSE VALUE\r\n");
        } else if (ops9_frame_count == 0U ||
                   (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS) {
            pose_control_active = 0U;
            Pose_ResetPlanner();
            StopAllMotors();
            printf("# ERROR OPS NOT READY\r\n");
        } else {
            debug_motor_active = 0U;
            debug_chassis_active = 0U;
            LLM_TunerAbort();
            StopAllMotors();
            /* 只清积分和误差历史，调好的 Kp/Ki/Kd 会原样保留。 */
            PID_Reset(&pid_x);
            PID_Reset(&pid_y);
            PID_Reset(&pid_yaw);
            /*
             * POSE SET 继续接收 OPS 原始目标坐标，保证现有上位机命令兼容；
             * 内部按目标航向换算成车体中心目标，再用中心坐标闭环。
             */
            Pose_OpsToChassisCenter(pose_x, pose_y, pose_yaw,
                                    &pose_target_center_x_mm,
                                    &pose_target_center_y_mm);
            Pose_OpsToChassisCenter(robot_x, robot_y, robot_yaw,
                                    &center_x, &center_y);
            target_distance_mm = sqrtf(
                (pose_target_center_x_mm - center_x) *
                (pose_target_center_x_mm - center_x) +
                (pose_target_center_y_mm - center_y) *
                (pose_target_center_y_mm - center_y));
            if (!isfinite(target_distance_mm) ||
                target_distance_mm > POSE_TARGET_DISTANCE_HARD_MAX_MM) {
                pose_control_active = 0U;
                Pose_ResetPlanner();
                StopAllMotors();
                printf("# ERROR POSE OUT OF BOUNDS DISTANCE_MM=%.2f MAX_MM=%.2f\r\n",
                       target_distance_mm, POSE_TARGET_DISTANCE_HARD_MAX_MM);
                return 1U;
            }
            PID_SetTarget(&pid_x, pose_target_center_x_mm);
            PID_SetTarget(&pid_y, pose_target_center_y_mm);
            pose_target_x_mm = pose_x;
            pose_target_y_mm = pose_y;
            pose_target_yaw_deg = pose_yaw;
            pose_translation_yaw_deg = robot_yaw;
            Mecanum_ClearCanTxFault();
            Pose_ResetPlanner();
            pose_last_control_time = now;
            pose_start_time = now;
            pose_control_phase = POSE_PHASE_TRANSLATE;
            pose_control_active = 1U;
            printf("# POSE START X=%.2f Y=%.2f YAW=%.2f "
                   "CENTER_X=%.2f CENTER_Y=%.2f TOL_MM=%.2f TOL_YAW=%.2f\r\n",
                   pose_x, pose_y, pose_yaw,
                   pose_target_center_x_mm, pose_target_center_y_mm,
                   POSE_POSITION_TOL_MM, POSE_YAW_TOL_DEG);
        }
        return 1U;
    }

    if (strcmp(command, "MOTOR STOP ALL") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
        printf("# MOTOR STOP ALL\r\n");
        return 1U;
    }

    if (strcmp(command, "MOVE STOP") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
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

        /* 调试运动仅限 TUNE/PLOT；WORK 模式下唯一运动源是 POSE SET（受心跳与安全检查保护）。 */
        if (current_robot_mode == ROBOT_MODE_WORK) {
            printf("# ERROR MODE REQUIRED=TUNE|PLOT CURRENT=WORK\r\n");
            return 1U;
        }
        if (!ChassisSafety_CanReady() || !ChassisSafety_OpsReady(HAL_GetTick())) {
            printf("# ERROR DEBUG CHASSIS SAFETY CAN=%u OPS=%u\r\n",
                   ChassisSafety_CanReady(), ChassisSafety_OpsReady(HAL_GetTick()));
            return 1U;
        }

        /* 旋转测试只给 Vz，低速且定时自动停止，用于检查四轮旋转组合和OPS航向。 */
        StopAllMotors();
        debug_motor_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
        Mecanum_ClearCanTxFault();
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

        /* 调试运动仅限 TUNE/PLOT；WORK 模式下唯一运动源是 POSE SET（受心跳与安全检查保护）。 */
        if (current_robot_mode == ROBOT_MODE_WORK) {
            printf("# ERROR MODE REQUIRED=TUNE|PLOT CURRENT=WORK\r\n");
            return 1U;
        }
        if (!ChassisSafety_CanReady() || !ChassisSafety_OpsReady(HAL_GetTick())) {
            printf("# ERROR DEBUG CHASSIS SAFETY CAN=%u OPS=%u\r\n",
                   ChassisSafety_CanReady(), ChassisSafety_OpsReady(HAL_GetTick()));
            return 1U;
        }

        StopAllMotors();
        debug_motor_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
        Mecanum_ClearCanTxFault();
        Mecanum_Kinematics(vx, vy, 0.0f, &v1, &v2, &v3, &v4);
        SetAllMotorsSpeed(v1, v2, v3, v4);
        debug_chassis_active = 1U;
        debug_chassis_stop_tick = HAL_GetTick() + (uint32_t)move_duration_ms;
        printf("# MOVE %s SPEED=%.3f MPS MS=%lu VX=%.3f VY=%.3f\r\n",
               move_direction, move_speed, move_duration_ms, vx, vy);
        return 1U;
    }

    if (sscanf(command, "MOTOR EN %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_EnableSingleMotor((uint8_t)id, 1U);
            printf("# MOTOR EN ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR DIS %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
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
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_SetSingleMotorSpeed((uint8_t)id, 0.0f);
            if (debug_motor_active && debug_motor_id == (uint8_t)id) debug_motor_active = 0U;
            printf("# MOTOR STOP ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR GET %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
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
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else if (!isfinite(rpm) || rpm == 0.0f || fabsf(rpm) > DEBUG_MOTOR_MAX_RPM) {
            printf("# ERROR RPM RANGE +/-%.0f NONZERO\r\n", DEBUG_MOTOR_MAX_RPM);
        } else if (duration_ms < 100UL || duration_ms > DEBUG_MOTOR_MAX_MS) {
            printf("# ERROR DURATION 100..%lu MS\r\n", DEBUG_MOTOR_MAX_MS);
        } else {
            /* 调试运动仅限 TUNE/PLOT；WORK 模式下唯一运动源是 POSE SET（受心跳与安全检查保护）。 */
            if (current_robot_mode == ROBOT_MODE_WORK) {
                printf("# ERROR MODE REQUIRED=TUNE|PLOT CURRENT=WORK\r\n");
                return 1U;
            }
            if (!ChassisSafety_CanReady()) {
                printf("# ERROR MOTOR RUN CAN NOT READY\r\n");
                return 1U;
            }
            StopAllMotors();
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            Pose_ResetPlanner();
            LLM_TunerAbort();
            Mecanum_ClearCanTxFault();
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

static void Pose_ProcessControl(uint32_t now)
{
    float current_ops_x;
    float current_ops_y;
    float current_x;
    float current_y;
    float current_yaw;
    float error_x;
    float error_y;
    float error_yaw;
    float yaw_reference_deg;
    float distance_mm;
    float linear_speed;
    float world_vx;
    float world_vy;
    float world_speed;
    float linear_limit;
    float heading_rad;
    float body_vx;
    float body_vy;
    float desired_vz;
    float yaw_limit;
    float step_yaw;
    float yaw_rate;
    float dt_s;
    float v1, v2, v3, v4;
    uint32_t last_ops_tick;
    uint32_t elapsed_ms;
    uint32_t can_error;
    HAL_CAN_StateTypeDef can_state;

    if (!pose_control_active) {
        return;
    }

    last_ops_tick = ops9_last_update_tick;
    now = HAL_GetTick();
    current_ops_x = robot_x;
    current_ops_y = robot_y;
    current_yaw = robot_yaw;
    can_state = HAL_CAN_GetState(&hcan1);
    can_error = HAL_CAN_GetError(&hcan1);

    if (!isfinite(current_ops_x) || !isfinite(current_ops_y) || !isfinite(current_yaw) ||
        ops9_frame_count == 0U ||
        (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS ||
        (active_host_link == HOST_LINK_RPI &&
         current_robot_mode != ROBOT_MODE_TUNE &&
         (uint32_t)(now - last_host_command_tick) > LLM_TUNE_HOST_TIMEOUT_MS) ||
        (current_robot_mode == ROBOT_MODE_TUNE &&
         (uint32_t)(now - pose_start_time) > POSE_TUNE_TIMEOUT_MS) ||
        can_state != HAL_CAN_STATE_LISTENING || can_error != HAL_CAN_ERROR_NONE) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        StopAllMotors();
        (void)G6220_SetEnabled(0U);
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    Pose_OpsToChassisCenter(current_ops_x, current_ops_y, current_yaw,
                            &current_x, &current_y);
    error_x = pose_target_center_x_mm - current_x;
    error_y = pose_target_center_y_mm - current_y;
    yaw_reference_deg = (pose_control_phase == POSE_PHASE_TRANSLATE) ?
                        pose_translation_yaw_deg : pose_target_yaw_deg;
    error_yaw = Pose_AngleErrorDeg(yaw_reference_deg, current_yaw);
    distance_mm = sqrtf(error_x * error_x + error_y * error_y);

    if (!isfinite(current_x) || !isfinite(current_y) ||
        !isfinite(error_x) || !isfinite(error_y) || !isfinite(error_yaw) ||
        !isfinite(distance_mm) ||
        distance_mm > POSE_RUNTIME_ERROR_HARD_MAX_MM) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        StopAllMotors();
        (void)G6220_SetEnabled(0U);
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    /* 安全条件每次主循环都检查；只有正常规划计算受 20 ms 控制周期限制。 */
    elapsed_ms = (uint32_t)(now - pose_last_control_time);
    if (elapsed_ms < POSE_CONTROL_PERIOD_MS) return;
    if (elapsed_ms > POSE_DT_MAX_MS) elapsed_ms = POSE_DT_MAX_MS;
    dt_s = (float)elapsed_ms / 1000.0f;
    pose_last_control_time = now;

    /* X/Y PID先给出全局速度，再旋转到车体坐标，供树莓派直接下发全局目标位姿。 */
    world_vx = PID_Calc(&pid_x, current_x);
    world_vy = PID_Calc(&pid_y, current_y);
    world_speed = sqrtf(world_vx * world_vx + world_vy * world_vy);
    linear_limit = Pose_BrakeLimitLinear(distance_mm, POSE_POSITION_TOL_MM);
    /* X/Y 的 PID LIMIT 最终解释为二维平移速度矢量的模长上限。 */
    if (linear_limit > pid_x.max_out) linear_limit = pid_x.max_out;
    if (linear_limit > pid_y.max_out) linear_limit = pid_y.max_out;
    if (linear_limit > POSE_SPEED_HARD_MAX_MPS) {
        linear_limit = POSE_SPEED_HARD_MAX_MPS;
    }
    if (world_speed > linear_limit && world_speed > 0.0001f) {
        float scale = linear_limit / world_speed;
        world_vx *= scale;
        world_vy *= scale;
    }

    heading_rad = current_yaw * (3.1415926f / 180.0f);
    body_vx = cosf(heading_rad) * world_vx + sinf(heading_rad) * world_vy;
    body_vy = -sinf(heading_rad) * world_vx + cosf(heading_rad) * world_vy;
    pose_target_vx = body_vx;
    pose_target_vy = body_vy;

    desired_vz = PID_CalcError(&pid_yaw, error_yaw);
    yaw_limit = Pose_BrakeLimitYaw(error_yaw, POSE_YAW_TOL_DEG);
    if (yaw_limit > pid_yaw.max_out) yaw_limit = pid_yaw.max_out;
    if (yaw_limit > POSE_YAW_SPEED_HARD_MAX_RADPS) {
        yaw_limit = POSE_YAW_SPEED_HARD_MAX_RADPS;
    }
    desired_vz = Math_ClampFloat(desired_vz, -yaw_limit, yaw_limit);
    pose_target_vz = desired_vz;

    /* 对整个 Vx/Vy 差矢量限幅，保留 PID 给出的平移方向，避免逐轴斜坡扭曲轨迹。 */
    Pose_SlewVector2D(pose_output_vx, pose_output_vy,
                      body_vx, body_vy,
                      pose_motion_profile.linear_accel_mps2,
                      pose_motion_profile.linear_decel_mps2,
                      dt_s, &pose_output_vx, &pose_output_vy);
    yaw_rate = ((fabsf(pose_output_vz) > 0.0001f &&
                 pose_output_vz * desired_vz <= 0.0f) ||
                fabsf(desired_vz) < fabsf(pose_output_vz)) ?
               pose_motion_profile.yaw_decel_radps2 :
               pose_motion_profile.yaw_accel_radps2;
    step_yaw = yaw_rate * dt_s;
    pose_output_vz = Pose_SlewScalar(pose_output_vz, desired_vz, step_yaw);
    linear_speed = sqrtf(pose_output_vx * pose_output_vx +
                         pose_output_vy * pose_output_vy);

    if (!isfinite(pose_output_vx) || !isfinite(pose_output_vy) ||
        !isfinite(pose_output_vz)) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        StopAllMotors();
        (void)G6220_SetEnabled(0U);
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    if (pose_control_phase == POSE_PHASE_TRANSLATE &&
        distance_mm <= POSE_POSITION_TOL_MM &&
        linear_speed <= POSE_STOP_LINEAR_EPS_MPS &&
        fabsf(error_yaw) <= POSE_YAW_TOL_DEG &&
        fabsf(pose_output_vz) <= POSE_STOP_YAW_EPS_RADPS) {
        /*
         * 平移阶段保持起始航向；到点且速度接近零后明确停车，再切换到
         * 原地转向阶段。协议仍只在最终位姿稳定后报告 POSE TARGET。
         */
        StopAllMotors();
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        Pose_ResetPlanner();
        pose_control_phase = POSE_PHASE_ROTATE;
        pose_last_control_time = now;
        return;
    }

    if (pose_control_phase == POSE_PHASE_ROTATE &&
        distance_mm <= POSE_POSITION_TOL_MM &&
        fabsf(error_yaw) <= POSE_YAW_TOL_DEG &&
        linear_speed <= POSE_STOP_LINEAR_EPS_MPS &&
        fabsf(pose_output_vz) <= POSE_STOP_YAW_EPS_RADPS) {
        pose_settle_cycles++;
        if (pose_settle_cycles >= POSE_SETTLE_CYCLES) {
            pose_control_active = 0U;
            Pose_ResetPlanner();
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

static void Host_ProcessCommand(void)
{
    char command[64];
    uint8_t i;
    float p_val, i_val, d_val;
    float speed_limit_val;
    char axis_name[8];
    PID_Controller *pid;
    LLM_TuneAxis_t requested_axis;
    float kp_max, ki_max, kd_max, output_min, output_hard_max;

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
    __enable_irq();

    /* HOST LINK 在所有业务命令之前处理；抢占失败不能刷新当前主机心跳。 */
    if (HostLink_ProcessCommand(command)) {
        return;
    }

    if (active_host_link == HOST_LINK_NONE) {
        /* 等待期间只开放无运动副作用的探测和停车命令。 */
        if (strcmp(command, "PING") == 0 ||
            strcmp(command, "PROTO VERSION") == 0 ||
            strcmp(command, "HELP") == 0) {
            (void)Host_ProcessOperationalCommand(command);
        } else if (strcmp(command, "STOP") == 0) {
            Robot_StopAllMotion();
            printf("# STOP MODE=%s HOST=WAITING\r\n", RobotMode_Name(current_robot_mode));
        } else {
            printf("# ERROR HOST NOT LINKED\r\n");
        }
        return;
    }

    last_host_command_tick = HAL_GetTick();

    if (Host_ProcessOperationalCommand(command)) {
        return;
    }

    if (strcmp(command, "PID STATUS ALL") == 0) {
        printf("# PID ALL X=%.7f,%.8f,%.7f Y=%.7f,%.8f,%.7f "
               "YAW=%.7f,%.8f,%.7f\r\n",
               pid_x.Kp, pid_x.Ki, pid_x.Kd,
               pid_y.Kp, pid_y.Ki, pid_y.Kd,
               pid_yaw.Kp, pid_yaw.Ki, pid_yaw.Kd);
    } else if (sscanf(command, "PID LIMIT %7s %f", axis_name, &speed_limit_val) == 2) {
        if (strcmp(axis_name, "X") == 0) requested_axis = LLM_TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = LLM_TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = LLM_TUNE_AXIS_YAW;
        else {
            printf("# ERROR PID AXIS X|Y|YAW\r\n");
            return;
        }
        output_min = requested_axis == LLM_TUNE_AXIS_YAW ?
                     POSE_YAW_SPEED_MIN_RADPS : POSE_SPEED_MIN_MPS;
        output_hard_max = requested_axis == LLM_TUNE_AXIS_YAW ?
                          POSE_YAW_SPEED_HARD_MAX_RADPS :
                          POSE_SPEED_HARD_MAX_MPS;
        if (isfinite(speed_limit_val) && speed_limit_val >= output_min &&
            speed_limit_val <= output_hard_max) {
            pid = LLM_TunerGetPidForAxis(requested_axis);
            pid->max_out = speed_limit_val;
            printf("# PID LIMIT AXIS=%s OUTPUT=%.3f UNIT=%s\r\n",
                   LLM_TunerAxisName(requested_axis), speed_limit_val,
                   requested_axis == LLM_TUNE_AXIS_YAW ? "RADPS" : "MPS");
        } else {
            printf("# ERROR PID LIMIT OUTPUT %.2f..%.2f\r\n",
                   output_min, output_hard_max);
        }
    } else if (sscanf(command, "PID SET %7s %f %f %f",
                      axis_name, &p_val, &i_val, &d_val) == 4) {
        if (strcmp(axis_name, "X") == 0) requested_axis = LLM_TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = LLM_TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = LLM_TUNE_AXIS_YAW;
        else {
            printf("# ERROR PID AXIS X|Y|YAW\r\n");
            return;
        }
        pid = LLM_TunerGetPidForAxis(requested_axis);
        kp_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KP_MAX : LLM_TUNE_KP_MAX;
        ki_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KI_MAX : LLM_TUNE_KI_MAX;
        kd_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KD_MAX : LLM_TUNE_KD_MAX;
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= kp_max &&
            i_val >= 0.0f && i_val <= ki_max &&
            d_val >= 0.0f && d_val <= kd_max) {
            pid->Kp = p_val;
            pid->Ki = i_val;
            pid->Kd = d_val;
            PID_Reset(pid);
            printf("# PID LOADED AXIS=%s P=%.7f I=%.8f D=%.7f\r\n",
                   LLM_TunerAxisName(requested_axis), p_val, i_val, d_val);
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   kp_max, ki_max, kd_max);
        }
    } else if (sscanf(command, "TUNE AXIS %7s", axis_name) == 1) {
        if (strcmp(axis_name, "X") == 0) requested_axis = LLM_TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = LLM_TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = LLM_TUNE_AXIS_YAW;
        else {
            printf("# ERROR TUNE AXIS X|Y|YAW\r\n");
            return;
        }
        /* 兼容现有调参器：TUNE AXIS 同时作为进入独立 TUNE 模式的入口。 */
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            Robot_SetMode(ROBOT_MODE_TUNE);
        }
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerSetAxis(requested_axis);
        LLM_TunerStopRound("AXIS");
        LLM_TunerResetSession();
        printf("# TUNE AXIS %s UNIT_IN=%s UNIT_OUT=%s\r\n",
               LLM_TunerAxisName(requested_axis),
               requested_axis == LLM_TUNE_AXIS_YAW ? "DEG" : "MM",
               requested_axis == LLM_TUNE_AXIS_YAW ? "RADPS" : "MPS");
    } else if (sscanf(command, "TUNE LIMIT %f", &speed_limit_val) == 1) {
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            printf("# ERROR MODE REQUIRED=TUNE CURRENT=%s\r\n",
                   RobotMode_Name(current_robot_mode));
            return;
        }
        requested_axis = LLM_TunerGetAxis();
        pid = LLM_TunerGetPid();
        output_hard_max = requested_axis == LLM_TUNE_AXIS_YAW ?
                          LLM_TUNE_YAW_HARD_MAX_RADPS : LLM_TUNE_SPEED_HARD_MAX_MPS;
        /* 工作速度上限由上位机统一配置；这里保留独立硬上限作为最终安全边界。 */
        if (isfinite(speed_limit_val) && speed_limit_val >= 0.02f &&
            speed_limit_val <= output_hard_max) {
            pid->max_out = speed_limit_val;
            printf("# TUNE LIMIT AXIS=%s OUTPUT=%.3f UNIT=%s HARD_MAX=%.3f\r\n",
                   LLM_TunerAxisName(requested_axis), pid->max_out,
                   requested_axis == LLM_TUNE_AXIS_YAW ? "RADPS" : "MPS",
                   output_hard_max);
        } else {
            printf("# ERROR TUNE LIMIT 0.02..%.2f\r\n", output_hard_max);
        }
    } else if ((sscanf(command, "SET P:%f I:%f D:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "SET KP:%f KI:%f KD:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "PID %f %f %f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "P:%f,I:%f,D:%f", &p_val, &i_val, &d_val) == 3)) {
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            printf("# ERROR MODE REQUIRED=TUNE CURRENT=%s\r\n",
                   RobotMode_Name(current_robot_mode));
            return;
        }
        requested_axis = LLM_TunerGetAxis();
        pid = LLM_TunerGetPid();
        kp_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KP_MAX : LLM_TUNE_KP_MAX;
        ki_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KI_MAX : LLM_TUNE_KI_MAX;
        kd_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KD_MAX : LLM_TUNE_KD_MAX;
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= kp_max &&
            i_val >= 0.0f && i_val <= ki_max &&
            d_val >= 0.0f && d_val <= kd_max) {
            pid->Kp = p_val;
            pid->Ki = i_val;
            pid->Kd = d_val;
            printf("# PID UPDATED AXIS=%s P=%.7f I=%.8f D=%.7f\r\n",
                   LLM_TunerAxisName(requested_axis), p_val, i_val, d_val);
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            Pose_ResetPlanner();
            Mecanum_ClearCanTxFault();
            LLM_TunerStartRound();
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   kp_max, ki_max, kd_max);
        }
    } else if (strcmp(command, "STATUS") == 0) {
        requested_axis = LLM_TunerGetAxis();
        pid = LLM_TunerGetPid();
        printf("# STATUS MODE=%s HOST_PROTO=%u HOST=%s AXIS=%s P=%.7f I=%.8f D=%.7f MAX_OUT=%.3f "
               "STATE=%u PLOT=%u MOTOR_PROTO=%s OPS_FRAMES=%lu UART_TX_OK=%lu UART_TX_ERR=%lu\r\n",
               RobotMode_Name(current_robot_mode), HOST_PROTOCOL_VERSION,
               HostLink_Name(active_host_link),
               LLM_TunerAxisName(requested_axis),
               pid->Kp, pid->Ki, pid->Kd, pid->max_out,
               (unsigned int)LLM_TunerGetState(),
               telemetry_mask != 0U,
               ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM",
               (unsigned long)ops9_frame_count,
               (unsigned long)host_uart_tx_ok,
               (unsigned long)host_uart_tx_error);
    } else if (strcmp(command, "RESET") == 0) {
        PID_Reset(LLM_TunerGetPid());
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerResetSession();
        LLM_TunerStopRound("RESET");
    } else if (strcmp(command, "STOP") == 0) {
        if (current_robot_mode == ROBOT_MODE_TUNE && LLM_TunerIsRunning()) {
            debug_motor_active = 0U;
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            Pose_ResetPlanner();
            (void)G6220_SetEnabled(0U);
            LLM_TunerStopRound("HOST");
        } else {
            Robot_StopAllMotion();
            printf("# STOP MODE=%s\r\n", RobotMode_Name(current_robot_mode));
        }
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
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  // 声明外部的接收缓存变量
  extern uint8_t ops9_rx_byte;
  DM_G6220_Result_t g6220_result;
  // 开启 USART2 单字节中断接收
  HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
  // 开启 USART1 单字节中断接收（接收树莓派或 PC 发来的主机命令）
    HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1);
  // 1. 初始化 CAN 和过滤器
  ZDT_CAN_ConfigFilter();

  // 2. 注册回调
  ZDT_CAN_RegisterCallback(ZDT_Emm_RxHandler);

  // G6220 独占 CAN2 (1 Mbit/s)：初始化软件对象、过滤器和 FIFO0 中断。
  g6220_result = DM_G6220_Init(&g6220_motor, &hcan2,
                               G6220_CAN_ID, G6220_MASTER_ID);
  if (g6220_result == DM_G6220_OK) {
      g6220_result = DM_G6220_StartCan(&g6220_motor,
                                      G6220_CAN2_FILTER_BANK,
                                      G6220_SLAVE_FILTER_START);
  }
  g6220_last_result = g6220_result;
  g6220_initialized = (g6220_result == DM_G6220_OK) ? 1U : 0U;

  // 3. 初始化 4 个电机
  ZDT_Emm_InitAll();

  // 4. 上电等待阶段四轮保持零速使能，用闭环保持力矩防止外力造成车体偏移。
  HAL_Delay(100);
  StopAllMotors();
  HostLink_SetChassisEnabled(1U);
  HAL_Delay(100);

  // 5. TIM3/TIM4 当前未使用，不启动定时器。

  // 6. 初始化里程计计时器
    last_odom_tick = HAL_GetTick();
    last_host_command_tick = HAL_GetTick();

  //7.初始化PID参数
  // 注意：坐标单位是 mm，误差 1000mm 时，乘以 Kp=0.001，算出的速度正好是 1.0 m/s
    Pose_InitMotionProfile();
    PID_Init(&pid_x,   0.001f, 0.0f, 0.0f, POSE_SPEED_DEFAULT_MPS, 5000.0f);
    PID_Init(&pid_y,   0.001f, 0.0f, 0.0f, POSE_SPEED_DEFAULT_MPS, 5000.0f);
    PID_Init(&pid_yaw, 0.01f,  0.0f, 0.0f, POSE_YAW_SPEED_DEFAULT_RADPS, 1000.0f);
    LLM_TunerInit(&pid_x, &pid_y, &pid_yaw);

    StopAllMotors();
    if (g6220_initialized) {
        /* 厂商建议 CAN 初始化后等待约 1 秒；等待态仍保持 G6220 失能。 */
        HAL_Delay(G6220_STARTUP_DELAY_MS);
        g6220_result = G6220_SetEnabled(0U);
        HAL_Delay(G6220_COMMAND_DELAY_MS);
    }
    host_wait_start_tick = HAL_GetTick();
    last_host_command_tick = host_wait_start_tick;
    printf("# STM32F407 MECANUM X/Y/YAW PID CONTROLLER HOST WAIT\r\n");
    printf("# PROTO VERSION=%u MODES=WORK,TUNE,PLOT LEGACY_POSE=1 HOST_LINK=REQUIRED\r\n",
           HOST_PROTOCOL_VERSION);
    printf("# HOST WAIT STATE=WAITING TIMEOUT_MS=%lu ACCEPT=COM,RPI\r\n",
           (unsigned long)HOST_WAIT_TIMEOUT_MS);
    printf("# MODE WORK PLOT=0 MOTION=STOPPED HOST=WAITING\r\n");
    printf("# G6220 INIT=%u ENABLE_REQ=%u RESULT=%u CAN_ID=0x%02X MASTER_ID=0x%03X\r\n",
           g6220_initialized, g6220_enable_requested,
           (unsigned int)g6220_last_result,
           (unsigned int)G6220_CAN_ID, (unsigned int)G6220_MASTER_ID);
    printf("# CSV timestamp,setpoint,input,output,error,p,i,d,ops_x_mm,ops_y_mm,yaw_deg,"
           "cross_mm,yaw_delta_deg,hold_cross,hold_yaw,center_x_mm,center_y_mm;"
           " UNIT BY AXIS\r\n");
    printf("# MOTOR PROTOCOL DEFAULT EMM; SEND HELP FOR DEBUG COMMANDS\r\n");
    printf("# SEND OPS STATUS OR OPS MONITOR ON TO CHECK OPS-9 LINK\r\n");
    printf("# TUNE NO PING ROUND=5S POSE=15S ROUNDS=%lu; HARD LIMIT %.2fMPS; SAFETY FAULT STOPS MOTORS\r\n",
           (unsigned long)LLM_TUNE_MAX_SESSION_ROUNDS,
           LLM_TUNE_SPEED_HARD_MAX_MPS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();
      Host_ProcessCommand();
      Motor_ProcessFeedback();

      /* 命令处理和串口中断可能更新时间戳，超时判断前必须刷新当前时间。 */
      now = HAL_GetTick();
      HostLink_ProcessWait(now);
      ChassisSafety_Process(now);
      Pose_ProcessControl(now);

      /* 鎺у埗鍜岃秴鏃朵紭鍏堬紱涓插彛杞涓庨仴娴嬫斁鍦ㄦ湰杞湯灏俱€?*/
      if (current_robot_mode == ROBOT_MODE_TUNE) {
          LLM_TunerProcess(now);
      }

      if (debug_motor_active && (int32_t)(now - debug_motor_stop_tick) >= 0)
      {
          Mecanum_ReportCanTxResult(
              ZDT_Emm_SetSingleMotorSpeed(debug_motor_id, 0.0f));
          printf("# MOTOR AUTO STOP ID=%u\r\n", debug_motor_id);
          debug_motor_active = 0U;
      }

      if (debug_chassis_active && (int32_t)(now - debug_chassis_stop_tick) >= 0)
      {
          (void)StopAllMotors();
          printf("# MOVE AUTO STOP\r\n");
          debug_chassis_active = 0U;
      }

      if (ops_monitor_enabled && (uint32_t)(now - ops_monitor_last_tick) >= 1000U)
      {
          ops_monitor_last_tick = now;
          Ops_PrintStatus();
      }

      if (active_host_link != HOST_LINK_NONE) {
          Motor_ProcessFeedbackPolling(now);
          Telemetry_Process(now);
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
    if (hcan->Instance == CAN1) {
        ZDT_CAN_RxFIFO0_Handler(hcan);
    } else if (hcan->Instance == CAN2) {
        DM_G6220_RxFIFO0_Handler(&g6220_motor);
    }
}
int _write(int file, char *ptr, int len)
{
    HAL_StatusTypeDef status;
    (void)file;

    if (ptr == NULL || len <= 0 || len > 0xFFFF) {
        host_uart_tx_error++;
        return -1;
    }

    /*
     * 115200 8-N-1 下150字节约需13 ms。20 ms足够发送正常CSV行，
     * 同时避免USART异常时HAL_MAX_DELAY永久卡死主循环和安全停车逻辑。
     * 此处不能printf报告失败，否则会递归进入_write。
     */
    status = HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len,
                               HOST_UART_TX_TIMEOUT_MS);
    if (status != HAL_OK) {
        host_uart_tx_error++;
        return -1;
    }
    host_uart_tx_ok++;
    return len;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 1. 处理 OPS-9 传感器数据 (USART2)
	    if (huart->Instance == USART2)
	    {
	        OPS9_UART_RxCpltCallback(huart);
	    }
	    // 2. 处理树莓派或 PC 发来的主机指令 (USART1)
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
