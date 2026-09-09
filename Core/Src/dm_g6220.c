#include "dm_g6220.h"

#include <math.h>
#include <string.h>

#define DM_G6220_POSITION_VELOCITY_ID_OFFSET  0x100U

static float DM_G6220_UintToFloat(uint32_t value,
                                  float min_value,
                                  float max_value,
                                  uint8_t bits)
{
    const uint32_t max_integer = (1UL << bits) - 1UL;
    return ((float)value * (max_value - min_value) /
            (float)max_integer) + min_value;
}

static DM_G6220_Result_t DM_G6220_SendFrame(DM_G6220_Motor_t *motor,
                                            uint16_t std_id,
                                            const uint8_t data[DM_G6220_CAN_DLC])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    if (motor == NULL || motor->hcan == NULL || data == NULL ||
        std_id > 0x7FFU) {
        return DM_G6220_ERROR_PARAM;
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = DM_G6220_CAN_DLC;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 控制层不能因 CAN 邮箱堵塞而长期阻塞；由上层决定重试或安全停车。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(motor->hcan) == 0U) {
        motor->tx_error++;
        return DM_G6220_ERROR_CAN_BUSY;
    }
    if (HAL_CAN_AddTxMessage(motor->hcan, &tx_header,
                             (uint8_t *)data, &tx_mailbox) != HAL_OK) {
        motor->tx_error++;
        return DM_G6220_ERROR_CAN_TX;
    }

    motor->tx_ok++;
    return DM_G6220_OK;
}

DM_G6220_Result_t DM_G6220_Init(DM_G6220_Motor_t *motor,
                                CAN_HandleTypeDef *hcan,
                                uint16_t can_id,
                                uint16_t master_id)
{
    if (motor == NULL || hcan == NULL || can_id == 0U ||
        can_id > DM_G6220_MAX_CAN_ID || master_id > 0x7FFU) {
        return DM_G6220_ERROR_PARAM;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->can_id = can_id;
    motor->master_id = master_id;
    motor->feedback_position_range_rad =
        DM_G6220_DEFAULT_POSITION_RANGE_RAD;
    motor->feedback_velocity_range_radps =
        DM_G6220_DEFAULT_VELOCITY_RANGE_RADPS;
    motor->feedback_torque_range_nm =
        DM_G6220_DEFAULT_TORQUE_RANGE_NM;
    motor->feedback.can_id = (uint8_t)can_id;
    return DM_G6220_OK;
}

DM_G6220_Result_t DM_G6220_StartCan(DM_G6220_Motor_t *motor,
                                    uint32_t filter_bank,
                                    uint32_t slave_start_filter_bank)
{
    CAN_FilterTypeDef filter = {0};

    if (motor == NULL || motor->hcan == NULL || filter_bank > 27U ||
        slave_start_filter_bank > 27U) {
        return DM_G6220_ERROR_PARAM;
    }

    /*
     * CAN1/CAN2 共用过滤器组。本项目 CAN1 使用 bank 0，CAN2 从 bank 14
     * 开始。G6220 独占 CAN2 物理总线，先接收全部标准帧，再由反馈中的
     * Master_ID 和 CAN_ID 做第二层匹配，便于后续同总线增加机械臂电机。
     */
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = slave_start_filter_bank;

    if (HAL_CAN_ConfigFilter(motor->hcan, &filter) != HAL_OK) {
        return DM_G6220_ERROR_CAN_FILTER;
    }
    if (HAL_CAN_Start(motor->hcan) != HAL_OK) {
        return DM_G6220_ERROR_CAN_START;
    }
    if (HAL_CAN_ActivateNotification(
            motor->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return DM_G6220_ERROR_CAN_NOTIFICATION;
    }
    return DM_G6220_OK;
}

DM_G6220_Result_t DM_G6220_SetFeedbackRanges(
    DM_G6220_Motor_t *motor,
    float position_range_rad,
    float velocity_range_radps,
    float torque_range_nm)
{
    if (motor == NULL || !isfinite(position_range_rad) ||
        !isfinite(velocity_range_radps) || !isfinite(torque_range_nm) ||
        position_range_rad <= 0.0f || velocity_range_radps <= 0.0f ||
        torque_range_nm <= 0.0f) {
        return DM_G6220_ERROR_PARAM;
    }

    motor->feedback_position_range_rad = position_range_rad;
    motor->feedback_velocity_range_radps = velocity_range_radps;
    motor->feedback_torque_range_nm = torque_range_nm;
    return DM_G6220_OK;
}

DM_G6220_Result_t DM_G6220_SendCommand(DM_G6220_Motor_t *motor,
                                       DM_G6220_Command_t command)
{
    uint8_t data[DM_G6220_CAN_DLC];

    if (motor == NULL || command > DM_G6220_CMD_CLEAR_ERROR) {
        return DM_G6220_ERROR_PARAM;
    }

    memset(data, 0xFF, sizeof(data));
    if (command == DM_G6220_CMD_ENABLE) {
        data[7] = 0xFCU;
    } else if (command == DM_G6220_CMD_DISABLE) {
        data[7] = 0xFDU;
    } else if (command == DM_G6220_CMD_SAVE_ZERO) {
        data[7] = 0xFEU;
    } else {
        data[7] = 0xFBU;
    }

    return DM_G6220_SendFrame(motor, motor->can_id, data);
}

DM_G6220_Result_t DM_G6220_SendPositionVelocity(DM_G6220_Motor_t *motor,
                                                float position_rad,
                                                float velocity_radps)
{
    uint8_t data[DM_G6220_CAN_DLC];

    if (motor == NULL || !isfinite(position_rad) ||
        !isfinite(velocity_radps)) {
        return DM_G6220_ERROR_PARAM;
    }

    /* STM32F407 和 G6220 都使用 little-endian IEEE-754 float。 */
    memcpy(&data[0], &position_rad, sizeof(position_rad));
    memcpy(&data[4], &velocity_radps, sizeof(velocity_radps));
    return DM_G6220_SendFrame(motor,
                              (uint16_t)(motor->can_id +
                              DM_G6220_POSITION_VELOCITY_ID_OFFSET), data);
}

uint8_t DM_G6220_ParseFeedback(DM_G6220_Motor_t *motor,
                               uint32_t std_id,
                               const uint8_t *data,
                               uint8_t len)
{
    uint8_t feedback_id;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;

    if (motor == NULL || data == NULL || len != DM_G6220_CAN_DLC ||
        std_id != motor->master_id) {
        if (motor != NULL) motor->rx_ignored++;
        return 0U;
    }

    feedback_id = data[0] & 0x0FU;
    if (feedback_id != (uint8_t)motor->can_id) {
        motor->rx_ignored++;
        return 0U;
    }

    position = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    velocity = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    torque = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

    motor->feedback.can_id = feedback_id;
    motor->feedback.state = (DM_G6220_State_t)(data[0] >> 4);
    motor->feedback.position_rad = DM_G6220_UintToFloat(
        position, -motor->feedback_position_range_rad,
        motor->feedback_position_range_rad, 16U);
    motor->feedback.velocity_radps = DM_G6220_UintToFloat(
        velocity, -motor->feedback_velocity_range_radps,
        motor->feedback_velocity_range_radps, 12U);
    motor->feedback.torque_nm = DM_G6220_UintToFloat(
        torque, -motor->feedback_torque_range_nm,
        motor->feedback_torque_range_nm, 12U);
    motor->feedback.mos_temperature_c = (float)data[6];
    motor->feedback.rotor_temperature_c = (float)data[7];
    motor->feedback.update_tick_ms = HAL_GetTick();
    motor->rx_ok++;
    return 1U;
}

void DM_G6220_RxFIFO0_Handler(DM_G6220_Motor_t *motor)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[DM_G6220_CAN_DLC];
    uint8_t budget = 3U;

    if (motor == NULL || motor->hcan == NULL) {
        return;
    }

    while (budget-- && HAL_CAN_GetRxFifoFillLevel(motor->hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(motor->hcan, CAN_RX_FIFO0,
                                 &rx_header, data) != HAL_OK) {
            return;
        }
        if (rx_header.IDE == CAN_ID_STD && rx_header.RTR == CAN_RTR_DATA) {
            (void)DM_G6220_ParseFeedback(motor, rx_header.StdId,
                                         data, rx_header.DLC);
        } else {
            motor->rx_ignored++;
        }
    }
}

uint8_t DM_G6220_GetFeedback(DM_G6220_Motor_t *motor,
                             DM_G6220_Feedback_t *feedback)
{
    uint32_t primask;

    if (motor == NULL || feedback == NULL) {
        return 0U;
    }

    /* 反馈可能在 CAN 中断中更新，短暂关中断以避免取得半帧新旧混合数据。 */
    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor->feedback;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}
