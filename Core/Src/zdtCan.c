/*
 * zdtCan.c
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */
#include "zdtCan.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
static ZDT_CAN_RxCallback_t pRxCallback = NULL;
static volatile uint32_t tx_ok_count = 0;
static volatile uint32_t tx_error_count = 0;
static volatile uint32_t rx_count = 0;
static volatile uint8_t last_tx_result = 0;

// 配置过滤器：允许所有扩展帧通过
void ZDT_CAN_ConfigFilter(void) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // ID和Mask都设为0，表示接收总线上所有数据
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback) {
    pRxCallback = callback;
}

#define CAN_QUEUE_SIZE 16U
#define CAN_TX_TIMEOUT_MS 50U
typedef struct {
    uint32_t id, queued_at;
    uint8_t length, bytes[8], pending;
} PendingCan;
static PendingCan commands[CAN_QUEUE_SIZE], speeds[4], stops[4];
static uint8_t command_head, command_tail, command_count, tx_fault;
static uint32_t stop_mailboxes, stop_started, enabled_at[4];
static uint8_t enable_wait_mask;
static uint32_t enqueued, replaced, dropped, max_wait;

static uint8_t enqueue(PendingCan *slot, uint32_t id, uint8_t *data, uint8_t length)
{
    slot->id = id;
    slot->queued_at = HAL_GetTick();
    slot->length = length;
    memcpy(slot->bytes, data, length);
    slot->pending = 1U;
    enqueued++;
    return 0U;
}

uint8_t ZDT_CAN_Send_ExtId(uint32_t id, uint8_t *data, uint8_t length)
{
    uint32_t motor = id >> 8;
    if (!data || length == 0U || length > 8U || motor < 1U || motor > 4U) return 3U;
    if (data[0] == 0xF6U) {
        uint8_t was_pending = speeds[motor - 1U].pending;
        uint32_t oldest = speeds[motor - 1U].queued_at;
        if (was_pending) replaced++;
        enqueue(&speeds[motor - 1U], id, data, length);
        if (was_pending) speeds[motor - 1U].queued_at = oldest;
        return 0U;
    }
    if (command_count == CAN_QUEUE_SIZE) {
        dropped++; tx_fault = 1U; return 1U;
    }
    enqueue(&commands[command_head], id, data, length);
    command_head = (uint8_t)((command_head + 1U) % CAN_QUEUE_SIZE);
    command_count++;
    return 0U;
}

void ZDT_CAN_BeginStop(void)
{
    uint8_t i;
    dropped += command_count;
    command_head = command_tail = command_count = 0U;
    for (i = 0U; i < 4U; ++i) {
        dropped += speeds[i].pending;
        speeds[i].pending = stops[i].pending = 0U;
    }
    /* Cancel unsent hardware requests as well as the software queue. */
    (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    stop_mailboxes = 0U;
    stop_started = HAL_GetTick();
}

uint8_t ZDT_CAN_SendStop(uint32_t id, uint8_t *data, uint8_t length)
{
    uint32_t motor = id >> 8;
    if (!data || length == 0U || length > 8U || motor < 1U || motor > 4U) return 3U;
    return enqueue(&stops[motor - 1U], id, data, length);
}

void ZDT_CAN_Process(uint32_t now)
{
    uint8_t budget, i, urgent;
    for (budget = 0U; budget < 3U; ++budget) {
        PendingCan *slot = NULL;
        CAN_TxHeaderTypeDef header = {0};
        uint32_t mailbox, age;
        urgent = 0U;
        for (i = 0U; i < 4U; ++i) if (stops[i].pending) { slot = &stops[i]; urgent = 1U; break; }
        if (!slot && stop_mailboxes) {
            if (HAL_CAN_IsTxMessagePending(&hcan1, stop_mailboxes)) {
                if ((uint32_t)(now - stop_started) > CAN_TX_TIMEOUT_MS) tx_fault = 1U;
                return;
            }
            stop_mailboxes = 0U;
        }
        if (!slot && command_count) slot = &commands[command_tail];
        if (!slot) for (i = 0U; i < 4U; ++i) {
            if (speeds[i].pending) { slot = &speeds[i]; break; }
        }
        if (!slot) return;
        age = now - slot->queued_at;
        if (age > max_wait) max_wait = age;
        if (age > CAN_TX_TIMEOUT_MS) {
            tx_fault = 1U;
            if (!urgent) {
                for (i = 0U; i < 4U; ++i) {
                    dropped += speeds[i].pending;
                    speeds[i].pending = 0U;
                }
                dropped += command_count;
                command_head = command_tail = command_count = 0U;
                return;
            }
        }
        i = (uint8_t)((slot->id >> 8) - 1U);
        if (!urgent && (enable_wait_mask & (1U << i))) {
            if ((uint32_t)(now - enabled_at[i]) < 5U) return;
            enable_wait_mask &= (uint8_t)~(1U << i);
        }
        if (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) return;
        header.ExtId = slot->id;
        header.IDE = CAN_ID_EXT;
        header.RTR = CAN_RTR_DATA;
        header.DLC = slot->length;
        if (HAL_CAN_AddTxMessage(&hcan1, &header, slot->bytes, &mailbox) != HAL_OK) {
            tx_error_count++; last_tx_result = 2U; tx_fault = 1U; return;
        }
        if (urgent) stop_mailboxes |= mailbox;
        if (slot->bytes[0] == 0xF3U) {
            enabled_at[i] = now;
            enable_wait_mask |= (uint8_t)(1U << i);
        }
        slot->pending = 0U;
        if (!urgent && command_count && slot == &commands[command_tail]) {
            command_tail = (uint8_t)((command_tail + 1U) % CAN_QUEUE_SIZE); command_count--;
        }
        tx_ok_count++; last_tx_result = 0U;
    }
}
uint8_t ZDT_CAN_ConsumeFault(void) { uint8_t value = tx_fault; tx_fault = 0U; return value; }

void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    uint8_t budget = 3U;
    while (budget-- && HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            rx_count++;
            if (pRxCallback != NULL && RxHeader.IDE == CAN_ID_EXT) {
                pRxCallback(RxHeader.ExtId, RxData, RxHeader.DLC);
            }
        }
    }
}

void ZDT_CAN_GetStats(ZDT_CAN_Stats_t *stats) {
    if (stats == NULL) return;
    stats->tx_ok = tx_ok_count;
    stats->tx_error = tx_error_count;
    stats->rx_count = rx_count;
    stats->last_tx_result = last_tx_result;
    stats->enqueued = enqueued; stats->replaced = replaced;
    stats->dropped = dropped; stats->max_wait_ms = max_wait;
}

uint8_t ZDT_CAN_StopPending(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; ++i) if (stops[i].pending) return 1U;
    return stop_mailboxes && HAL_CAN_IsTxMessagePending(&hcan1, stop_mailboxes);
}
