#ifndef __ZDTCAN_H__
#define __ZDTCAN_H__

#include <stdint.h>
#ifdef CONTROL_HOST_TEST
#include "control_test_hal.h"
#else
#include "main.h"
#endif

typedef void (*ZDT_CAN_RxCallback_t)(uint32_t ExtId, uint8_t *Data, uint8_t Len);

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_error;
    uint32_t rx_count;
    uint8_t last_tx_result;
    uint32_t enqueued, replaced, dropped, max_wait_ms;
} ZDT_CAN_Stats_t;

void ZDT_CAN_Process(uint32_t now);
/* 发送 API 仅由主循环调用；返回 0 表示已入队，不是电机 ACK。
 * STOP 丢弃旧普通队列并申请撤销邮箱，随后优先发送四轮零速。 */
void ZDT_CAN_BeginStop(void);
uint8_t ZDT_CAN_SendStop(uint32_t id, uint8_t *data, uint8_t length);
uint8_t ZDT_CAN_ConsumeFault(void);
uint8_t ZDT_CAN_StopPending(void);

void ZDT_CAN_ConfigFilter(void);
void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback);
uint8_t ZDT_CAN_Send_ExtId(uint32_t ExtId, uint8_t *Data, uint8_t Len);
void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *hcan);
void ZDT_CAN_GetStats(ZDT_CAN_Stats_t *stats);
#endif
