#ifndef INC_DM_G6220_H_
#define INC_DM_G6220_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * G6220 的反馈帧使用整数表示位置、速度和力矩。下面三个量程用于
 * 把反馈值还原为物理单位，必须与达妙上位机中电机当前的
 * P_MAX / V_MAX / T_MAX 一致。
 */
#define DM_G6220_DEFAULT_POSITION_RANGE_RAD    12.5f
#define DM_G6220_DEFAULT_VELOCITY_RANGE_RADPS  45.0f
#define DM_G6220_DEFAULT_TORQUE_RANGE_NM       10.0f
#define DM_G6220_MAX_CAN_ID              0x0FU
#define DM_G6220_CAN_DLC                 8U

typedef enum {
    DM_G6220_OK = 0,
    DM_G6220_ERROR_PARAM,
    DM_G6220_ERROR_CAN_BUSY,
    DM_G6220_ERROR_CAN_TX,
    DM_G6220_ERROR_CAN_FILTER,
    DM_G6220_ERROR_CAN_START,
    DM_G6220_ERROR_CAN_NOTIFICATION
} DM_G6220_Result_t;

typedef enum {
    DM_G6220_CMD_ENABLE = 0,
    DM_G6220_CMD_DISABLE,
    DM_G6220_CMD_SAVE_ZERO,
    DM_G6220_CMD_CLEAR_ERROR
} DM_G6220_Command_t;

typedef enum {
    DM_G6220_STATE_DISABLED = 0x0,
    DM_G6220_STATE_ENABLED = 0x1,
    DM_G6220_STATE_OVER_VOLTAGE = 0x8,
    DM_G6220_STATE_UNDER_VOLTAGE = 0x9,
    DM_G6220_STATE_OVER_CURRENT = 0xA,
    DM_G6220_STATE_MOS_OVER_TEMPERATURE = 0xB,
    DM_G6220_STATE_COIL_OVER_TEMPERATURE = 0xC,
    DM_G6220_STATE_COMMUNICATION_LOST = 0xD,
    DM_G6220_STATE_OVERLOAD = 0xE
} DM_G6220_State_t;

typedef struct {
    uint8_t can_id;
    DM_G6220_State_t state;
    float position_rad;
    float velocity_radps;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
    uint32_t update_tick_ms;
} DM_G6220_Feedback_t;

typedef struct {
    CAN_HandleTypeDef *hcan;
    uint16_t can_id;
    uint16_t master_id;
    float feedback_position_range_rad;
    float feedback_velocity_range_radps;
    float feedback_torque_range_nm;
    DM_G6220_Feedback_t feedback;
    volatile uint32_t tx_ok;
    volatile uint32_t tx_error;
    volatile uint32_t rx_ok;
    volatile uint32_t rx_ignored;
} DM_G6220_Motor_t;

/* 初始化软件对象，不会启动 CAN，也不会使能或驱动电机。 */
DM_G6220_Result_t DM_G6220_Init(DM_G6220_Motor_t *motor,
                                CAN_HandleTypeDef *hcan,
                                uint16_t can_id,
                                uint16_t master_id);

/* 配置此电机所在的独立 bxCAN 总线并开启 FIFO0 接收中断。 */
DM_G6220_Result_t DM_G6220_StartCan(DM_G6220_Motor_t *motor,
                                    uint32_t filter_bank,
                                    uint32_t slave_start_filter_bank);

/* 修改反馈解析量程；三个值必须大于 0，并与电机参数完全一致。 */
DM_G6220_Result_t DM_G6220_SetFeedbackRanges(
    DM_G6220_Motor_t *motor,
    float position_range_rad,
    float velocity_range_radps,
    float torque_range_nm);

DM_G6220_Result_t DM_G6220_SendCommand(DM_G6220_Motor_t *motor,
                                       DM_G6220_Command_t command);

/* 位置-速度模式：位置和最大运行速度均为 little-endian float。 */
DM_G6220_Result_t DM_G6220_SendPositionVelocity(DM_G6220_Motor_t *motor,
                                                float position_rad,
                                                float velocity_radps);

/*
 * 在 CAN FIFO 回调取得标准帧后调用。返回 1 表示该帧属于此电机且已解析，
 * 返回 0 表示 ID、长度或数据不匹配。反馈帧的 StdId 应等于 Master_ID。
 */
uint8_t DM_G6220_ParseFeedback(DM_G6220_Motor_t *motor,
                               uint32_t std_id,
                               const uint8_t *data,
                               uint8_t len);

/* 在对应 CAN 的 HAL FIFO0 回调中调用，读取并解析积压的反馈帧。 */
void DM_G6220_RxFIFO0_Handler(DM_G6220_Motor_t *motor);

/* 从中断更新的反馈结构中取得一致快照。 */
uint8_t DM_G6220_GetFeedback(DM_G6220_Motor_t *motor,
                             DM_G6220_Feedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif /* INC_DM_G6220_H_ */
