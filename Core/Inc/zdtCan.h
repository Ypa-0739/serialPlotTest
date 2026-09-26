#ifndef __ZDTCAN_H__
#define __ZDTCAN_H__

#include <stdint.h>
#ifdef CONTROL_HOST_TEST
#include "control_test_hal.h"
#else
#include "main.h"
#endif

typedef void (*ZDT_CAN_RxCallback_t)(uint32_t ExtId, uint8_t *Data, uint8_t Len);

/*
 * CAN1 传输层快照。就绪判定同时检查 tx_fault 和当前 ESR/HAL 状态。
 * 累计计数供诊断使用，恢复逻辑也用其增量确认通信进展。
 */
typedef struct {
    /* 收发结果 */
    uint32_t tx_ok;             /* TXOK 完成回调次数：真正发上总线的帧数 */
    uint32_t rx_count;          /* HAL 成功取出的帧数，包括协议层丢弃的帧 */
    uint32_t tx_error;          /* HAL_CAN_AddTxMessage 提交失败次数 */

    /* 当前状态 */
    uint32_t fault_generation;
    uint8_t recovery_phase;     /* 0=healthy, 1=fault, 2=link observed; not permission */
    uint8_t tx_fault;           /* 发送故障锁存：最近有发送未按时完成 */
    uint8_t last_tx_result;     /* 0=完成 2=提交失败 4=邮箱超时 5=总线错误 */

    /* 恢复计数 */
    uint32_t recoveries;        /* 静止条件下解除旧故障的次数 */
    uint32_t auto_recoveries;   /* 兼容旧状态字段；不再自动解锁，保持为 0 */
    uint32_t stall_recoveries;  /* 邮箱卡死(FREE=0 且 TXOK 停增)强制恢复次数 */

    /* 控制器寄存器快照 */
    uint32_t esr, tsr;

    /* 错误累计与队列统计（纯诊断） */
    uint32_t error_callbacks, fatal_error_callbacks, error_latched;
    uint32_t tx_queued, tx_aborted, tx_timeout;
    uint32_t enqueued, replaced, dropped, max_wait_ms;
} ZDT_CAN_Stats_t;

void ZDT_CAN_Process(uint32_t now);
uint8_t ZDT_CAN_IsReady(void);
/* Main-loop only. eligible requires no active motion, fresh four-wheel feedback
 * and confirmed stop. Returns 1 only when an old transport fault was cleared. */
uint8_t ZDT_CAN_RecoverWhenIdle(uint8_t eligible);
/* 发送 API 仅由主循环调用；返回 0 表示已入队，不是电机 ACK。
 * STOP 丢弃旧普通队列并申请撤销邮箱，随后优先发送四轮零速。 */
void ZDT_CAN_BeginStop(void);
uint8_t ZDT_CAN_SendStop(uint32_t id, uint8_t *data, uint8_t length);
/* Consume notification only. The safety latch is cleared by idle recovery. */
uint8_t ZDT_CAN_ConsumeFault(void);
/* 上层报告一次发送失败。与内部超时共用同一个锁存位，避免出现
 * “清了一个还剩另一个”的双标志问题。 */
void ZDT_CAN_RaiseFault(void);
/* Read-only safety state; event consumption never changes readiness. */
uint8_t ZDT_CAN_HasFault(void);
uint8_t ZDT_CAN_HardwareReady(void);
#ifdef CONTROL_HOST_TEST
void ZDT_CAN_TestResetFault(void);
#endif
uint8_t ZDT_CAN_StopPending(void);

void ZDT_CAN_ConfigFilter(void);
void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback);
uint8_t ZDT_CAN_Send_ExtId(uint32_t ExtId, uint8_t *Data, uint8_t Len);
void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *hcan);
void ZDT_CAN_GetStats(ZDT_CAN_Stats_t *stats);
#endif
