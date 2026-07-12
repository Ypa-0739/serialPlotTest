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
#define LLM_TUNE_DURATION_MS        4000U
#define LLM_TUNE_TARGET_MM          300.0f
#define LLM_TUNE_MAX_SPEED_MPS      0.25f
#define LLM_TUNE_KP_MAX             0.005f
#define LLM_TUNE_KI_MAX             0.00005f
#define LLM_TUNE_KD_MAX             0.002f

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

TuneState_t current_tune_state = TUNE_STATE_WAIT;
uint32_t tune_start_time = 0;
uint32_t last_control_time = 0;
float start_y_pos = 0.0f;
float tune_direction = 1.0f;             // 每轮往返，避免一直驶离测试区域
float tune_output = 0.0f;
uint32_t tune_round_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LLM_ProcessCommand(void);
static void LLM_StartTuneRound(void);
static void LLM_StopTuneRound(const char *reason);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LLM_StartTuneRound(void)
{
    if (tune_round_count > 0U) {
        tune_direction = -tune_direction;
    }
    tune_round_count++;

    StopAllMotors();
    PID_Reset(&pid_y);
    start_y_pos = robot_y;
    PID_SetTarget(&pid_y, start_y_pos + tune_direction * LLM_TUNE_TARGET_MM);
    tune_output = 0.0f;
    tune_start_time = HAL_GetTick();
    last_control_time = tune_start_time;
    current_tune_state = TUNE_STATE_RUN;
    printf("# ROUND START %lu DIR %.0f\r\n",
           (unsigned long)tune_round_count, tune_direction);
}

static void LLM_StopTuneRound(const char *reason)
{
    StopAllMotors();
    tune_output = 0.0f;
    current_tune_state = TUNE_STATE_WAIT;
    printf("# ROUND STOP %s\r\n", reason);
}

static void LLM_ProcessCommand(void)
{
    char command[64];
    uint8_t i;
    float p_val, i_val, d_val;

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

    if ((sscanf(command, "SET P:%f I:%f D:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "SET KP:%f KI:%f KD:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "PID %f %f %f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "P:%f,I:%f,D:%f", &p_val, &i_val, &d_val) == 3)) {
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= LLM_TUNE_KP_MAX &&
            i_val >= 0.0f && i_val <= LLM_TUNE_KI_MAX &&
            d_val >= 0.0f && d_val <= LLM_TUNE_KD_MAX) {
            pid_y.Kp = p_val;
            pid_y.Ki = i_val;
            pid_y.Kd = d_val;
            printf("# PID UPDATED P=%.7f I=%.8f D=%.7f\r\n", p_val, i_val, d_val);
            LLM_StartTuneRound();
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   LLM_TUNE_KP_MAX, LLM_TUNE_KI_MAX, LLM_TUNE_KD_MAX);
        }
    } else if (strcmp(command, "STATUS") == 0) {
        printf("# STATUS P=%.7f I=%.8f D=%.7f STATE=%u\r\n",
               pid_y.Kp, pid_y.Ki, pid_y.Kd, (unsigned int)current_tune_state);
    } else if (strcmp(command, "RESET") == 0) {
        PID_Reset(&pid_y);
        tune_round_count = 0U;
        tune_direction = 1.0f;
        LLM_StopTuneRound("RESET");
    } else if (strcmp(command, "STOP") == 0) {
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

  //7.初始化PID参数
  // 注意：坐标单位是 mm，误差 1000mm 时，乘以 Kp=0.001，算出的速度正好是 1.0 m/s
    PID_Init(&pid_x,   0.002f, 0.0f, 0.0f, 0.3f, 0.1f);  // X轴纠偏：限速 0.3 m/s
    PID_Init(&pid_y,   0.001f, 0.0f, 0.0f, LLM_TUNE_MAX_SPEED_MPS, 5000.0f);
    PID_Init(&pid_yaw, 0.02f,  0.0f, 0.0f, 0.5f, 0.2f);  // 角度纠偏：限速 0.5 rad/s

    StopAllMotors();
    printf("# STM32F407 MECANUM Y-AXIS PID TUNER READY\r\n");
    printf("# CSV timestamp_ms,setpoint_mm,input_mm,output_mps,error_mm,p,i,d\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();
      LLM_ProcessCommand();

      if (current_tune_state == TUNE_STATE_RUN &&
          (uint32_t)(now - last_control_time) >= LLM_TUNE_CONTROL_PERIOD_MS)
      {
          float current_y = robot_y;
          float moved_distance;
          float normalized_input;
          float normalized_error;
          float V1, V2, V3, V4;

          last_control_time += LLM_TUNE_CONTROL_PERIOD_MS;
          if (!isfinite(current_y)) {
              LLM_StopTuneRound("INVALID OPS9");
              continue;
          }
          if ((uint32_t)(now - tune_start_time) >= LLM_TUNE_DURATION_MS) {
              LLM_StopTuneRound("TIMEOUT");
              continue;
          }

          tune_output = PID_Calc(&pid_y, current_y);
          Mecanum_Kinematics(0.0f, tune_output, 0.0f, &V1, &V2, &V3, &V4);
          SetAllMotorsSpeed(V1, V2, V3, V4);

          moved_distance = current_y - start_y_pos;
          normalized_input = tune_direction * moved_distance;
          normalized_error = LLM_TUNE_TARGET_MM - normalized_input;
          printf("%lu,%.2f,%.2f,%.4f,%.2f,%.7f,%.8f,%.7f\r\n",
                 (unsigned long)(now - tune_start_time),
                 LLM_TUNE_TARGET_MM, normalized_input,
                 tune_direction * tune_output, normalized_error,
                 pid_y.Kp, pid_y.Ki, pid_y.Kd);
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
