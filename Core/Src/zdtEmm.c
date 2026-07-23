/*
 * ZDT_X42S CAN motor driver.
 * CAN uses extended IDs: (motor_address << 8) | packet_index.
 */
#include "zdtEmm.h"
#include "zdtCan.h"

#define ZDT_MAX_RPM                 3000.0f
#define ZDT_X_DEFAULT_ACCEL_RPM_S   500U
#define ZDT_EVENT_QUEUE_SIZE        8U

ZDT_Motor_t motors[4];

static volatile ZDT_Protocol_t active_protocol = ZDT_PROTOCOL_EMM;
static volatile uint8_t event_head = 0U;
static volatile uint8_t event_tail = 0U;
static ZDT_MotorEvent_t event_queue[ZDT_EVENT_QUEUE_SIZE];

static uint8_t MotorIndexFromId(uint8_t id)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        if (motors[i].node_id == id) return i;
    }
    return 0xFFU;
}

static float ClampRpm(float rpm)
{
    if (rpm > ZDT_MAX_RPM) return ZDT_MAX_RPM;
    if (rpm < -ZDT_MAX_RPM) return -ZDT_MAX_RPM;
    return rpm;
}

void ZDT_Emm_InitAll(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        motors[i].node_id = i + 1U;
        motors[i].target_speed = 0.0f;
        motors[i].actual_speed = 0.0f;
        motors[i].dir = 0U;
        motors[i].acc = 0U;
        motors[i].enabled = 0U;
    }
}

void ZDT_Emm_SetProtocol(ZDT_Protocol_t protocol)
{
    active_protocol = (protocol == ZDT_PROTOCOL_X) ? ZDT_PROTOCOL_X : ZDT_PROTOCOL_EMM;
}

ZDT_Protocol_t ZDT_Emm_GetProtocol(void)
{
    return active_protocol;
}

uint8_t ZDT_Emm_SetSpeedByID(uint8_t id, float speed_rpm)
{
    uint8_t tx_data[8];
    uint8_t dir;
    uint8_t index;
    float abs_rpm;

    if (id < 1U || id > 4U) return 3U;

    speed_rpm = ClampRpm(speed_rpm);
    dir = (speed_rpm < 0.0f) ? 1U : 0U;
    abs_rpm = (speed_rpm < 0.0f) ? -speed_rpm : speed_rpm;

    tx_data[0] = 0xF6;
    tx_data[1] = dir;

    if (active_protocol == ZDT_PROTOCOL_X) {
        uint16_t accel = ZDT_X_DEFAULT_ACCEL_RPM_S;
        uint16_t speed_x10 = (uint16_t)(abs_rpm * 10.0f + 0.5f);
        tx_data[2] = (uint8_t)(accel >> 8);
        tx_data[3] = (uint8_t)accel;
        tx_data[4] = (uint8_t)(speed_x10 >> 8);
        tx_data[5] = (uint8_t)speed_x10;
        tx_data[6] = 0x00; /* execute immediately */
        tx_data[7] = 0x6B;
        index = MotorIndexFromId(id);
        if (index != 0xFFU) motors[index].target_speed = speed_rpm;
        return ZDT_CAN_Send_ExtId(((uint32_t)id << 8), tx_data, 8U);
    }

    {
        uint16_t speed_int = (uint16_t)(abs_rpm + 0.5f);
        tx_data[2] = (uint8_t)(speed_int >> 8);
        tx_data[3] = (uint8_t)speed_int;
        tx_data[4] = 0x00; /* no acceleration curve */
        tx_data[5] = 0x00; /* execute immediately */
        tx_data[6] = 0x6B;
    }
    index = MotorIndexFromId(id);
    if (index != 0xFFU) motors[index].target_speed = speed_rpm;
    return ZDT_CAN_Send_ExtId(((uint32_t)id << 8), tx_data, 7U);
}

uint8_t ZDT_Emm_ReadSpeedByID(uint8_t id)
{
    uint8_t tx_data[2] = {0x35, 0x6B};
    if (id < 1U || id > 4U) return 3U;
    return ZDT_CAN_Send_ExtId(((uint32_t)id << 8), tx_data, 2U);
}

uint8_t ZDT_Emm_ReadStatusByID(uint8_t id)
{
    uint8_t tx_data[2] = {0x3A, 0x6B};
    if (id < 1U || id > 4U) return 3U;
    return ZDT_CAN_Send_ExtId(((uint32_t)id << 8), tx_data, 2U);
}

uint8_t ZDT_Emm_EnableSingleMotor(uint8_t id, uint8_t enable)
{
    uint8_t tx_data[5];
    uint8_t result;
    uint8_t index;

    if (id < 1U || id > 4U) return 3U;

    /* Manual 5.3.2: F3 AB enable sync checksum. */
    tx_data[0] = 0xF3;
    tx_data[1] = 0xAB;
    tx_data[2] = enable ? 0x01 : 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x6B;
    result = ZDT_CAN_Send_ExtId(((uint32_t)id << 8), tx_data, 5U);

    index = MotorIndexFromId(id);
    if (result == 0U && index != 0xFFU) motors[index].enabled = enable ? 1U : 0U;
    return result;
}

uint8_t ZDT_Emm_EnableByID(uint8_t id)
{
    return ZDT_Emm_EnableSingleMotor(id, 1U);
}

uint8_t ZDT_Emm_SetSingleMotorSpeed(uint8_t id, float speed_rpm)
{
    return ZDT_Emm_SetSpeedByID(id, speed_rpm);
}

uint8_t ZDT_Emm_ReadSingleMotorSpeed(uint8_t id)
{
    return ZDT_Emm_ReadSpeedByID(id);
}

float ZDT_Emm_GetSingleMotorSpeed(uint8_t id)
{
    uint8_t index = MotorIndexFromId(id);
    return (index == 0xFFU) ? 0.0f : motors[index].actual_speed;
}

void ZDT_Emm_RxHandler(uint32_t ExtId, uint8_t *Data, uint8_t Len)
{
    uint8_t sender_id = (uint8_t)((ExtId >> 8) & 0xFFU);
    uint8_t index;
    uint8_t next_head;
    float speed = 0.0f;
    ZDT_MotorEvent_t event;

    if (Data == NULL || Len < 2U || sender_id < 1U || sender_id > 4U) return;
    index = MotorIndexFromId(sender_id);

    event.motor_id = sender_id;
    event.function_code = Data[0];
    event.value = Data[1];
    event.speed_rpm = 0.0f;

    if (Data[0] == 0x35U && Len >= 5U) {
        uint16_t raw_speed = ((uint16_t)Data[2] << 8) | Data[3];
        speed = (float)raw_speed;
        if (active_protocol == ZDT_PROTOCOL_X) speed *= 0.1f;
        if (Data[1] == 0x01U) speed = -speed;
        event.speed_rpm = speed;
        if (index != 0xFFU) motors[index].actual_speed = speed;
    } else if (Data[0] == 0x3AU && Len >= 3U) {
        if (index != 0xFFU) motors[index].enabled = (Data[1] & 0x01U) ? 1U : 0U;
    }

    next_head = (uint8_t)((event_head + 1U) % ZDT_EVENT_QUEUE_SIZE);
    if (next_head == event_tail) {
        event_tail = (uint8_t)((event_tail + 1U) % ZDT_EVENT_QUEUE_SIZE);
    }
    event_queue[event_head] = event;
    event_head = next_head;
}

uint8_t ZDT_Emm_PollEvent(ZDT_MotorEvent_t *event)
{
    uint32_t primask;
    if (event == NULL || event_head == event_tail) return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    *event = event_queue[event_tail];
    event_tail = (uint8_t)((event_tail + 1U) % ZDT_EVENT_QUEUE_SIZE);
    if (primask == 0U) __enable_irq();
    return 1U;
}
