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
static volatile uint32_t tx_queued, tx_aborted, error_callbacks;
static volatile uint32_t fatal_error_callbacks, error_latched;
static uint32_t tx_timeout, mailbox_started[3], tracked_mailboxes;
static uint32_t recoveries;
static uint32_t auto_recoveries;
static uint32_t auto_clear_since, auto_clear_ok_seen;
static uint8_t auto_clear_armed;
/* 邮箱卡死（FREE=0 且 TXOK 长期不增）强制恢复次数。 */
static uint32_t stall_recoveries;
static uint32_t stall_ok_mark, stall_since;
static uint8_t stall_armed;
/* 强制 Stop/Start 的最小间隔，避免 50ms 空转把总线打断成 bus-off。 */
static uint32_t last_unstick_tick;
#define CAN_UNSTICK_MIN_GAP_MS 500U

/* 只开收/发完成事件，不开任何错误类中断。
 *
 * 1) CAN_IT_ERROR_WARNING / CAN_IT_ERROR_PASSIVE = EWGIE/EPVIE，电平触发：
 *    EWGF/EPVF 只读清不掉，进入 warning/passive 后会中断风暴。
 * 2) CAN_IT_ERROR / CAN_IT_LAST_ERROR_CODE = ERRIE/LECIE：总线在 error passive
 *    且 TEC/REC 很高时，几乎每个位错误都进 ErrorCallback，现场 ERR_CB 仍可达
 *    每秒数万次。BUSOFF 同理由主循环看 ESR 即可。
 * 当前 EWGF/EPVF/BOFF 全部改为 ZDT_CAN_Process 轮询 ESR。 */
#define CAN_NOTIFY_MASK (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY)

static HAL_StatusTypeDef CanEnableNotifications(void)
{
    return HAL_CAN_ActivateNotification(&hcan1, CAN_NOTIFY_MASK);
}

// 硬件接收所有帧；ISR 只把扩展数据帧交给协议层。
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
    if (CanEnableNotifications() != HAL_OK) {
        Error_Handler();
    }
}

void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback) {
    pRxCallback = callback;
}

#define CAN_QUEUE_SIZE 16U
#define CAN_TX_TIMEOUT_MS 50U
#define CAN_FATAL_HAL_ERRORS (HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF | \
    HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1 | HAL_CAN_ERROR_TIMEOUT | \
    HAL_CAN_ERROR_NOT_INITIALIZED | HAL_CAN_ERROR_NOT_READY | \
    HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_PARAM | \
    HAL_CAN_ERROR_INTERNAL)
/*
 * 真正代表“控制器当前不可用”的 HAL 错误位。
 *
 * 刻意不包含 HAL_CAN_ERROR_PARAM：HAL 在正常运行时也会置这一位，例如
 * HAL_CAN_AddTxMessage 命中 TSR.CODE 与 TME 位之间的竞态(此时邮箱其实不空)、
 * 或 HAL_CAN_GetRxMessage 读到已经空的 RX FIFO。ErrorCode 是 |= 累积的，
 * 只有 HAL_CAN_ResetError 才会清；一旦把它当作硬件故障，CAN 就会永久停留在
 * NOT READY，MOTOR RUN 与 TUNE 被永久拒绝，却看不到任何 ESR/TEC 异常。
 * 参数类错误仍然完整保留在 CAN STATUS 的 ERROR/ERR_LATCH 字段中可见。
 */
#define CAN_BLOCKING_HAL_ERRORS (HAL_CAN_ERROR_TIMEOUT | \
    HAL_CAN_ERROR_NOT_INITIALIZED | HAL_CAN_ERROR_NOT_READY | \
    HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_INTERNAL)
/* tx_fault 恢复所需的“总线确实还在工作”的观察窗口。 */
#define CAN_FAULT_AUTO_CLEAR_MS 100U
/* FREE=0 且 TXOK 持续不增达到该时长，视为邮箱硬件卡死（Abort 清不掉 TME）。 */
#define CAN_TX_STALL_MS 150U
typedef struct {
    uint32_t id, queued_at;
    uint8_t length, bytes[8], pending;
} PendingCan;
static PendingCan commands[CAN_QUEUE_SIZE], speeds[4], stops[4];
static uint8_t command_head, command_tail, command_count;
static volatile uint8_t tx_fault;
static volatile uint8_t fault_event;
static volatile uint32_t fault_generation;
static uint8_t recovery_phase;

static void CanLatchFault(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    tx_fault = fault_event = 1U;
    fault_generation++;
    recovery_phase = 1U;
    __set_PRIMASK(saved);
}
static uint32_t stop_mailboxes, stop_started, enabled_at[4];
static uint8_t enable_wait_mask;
static uint32_t enqueued, replaced, dropped, max_wait;

static void CanDropPendingQueue(void);

/* 控制器当前是否处于“可以正常收发”的硬件状态。这里只看硬件与 HAL 状态，
 * 不看 tx_fault —— 后者正是本函数之外要判定的“是否还需要继续锁存”。 */
static uint8_t CanHardwareHealthy(void)
{
    if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING) return 0U;
    if (HAL_CAN_GetError(&hcan1) & CAN_BLOCKING_HAL_ERRORS) return 0U;
    if (hcan1.Instance->ESR & (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) return 0U;
    return 1U;
}

/* Link observation does not grant motion permission. Only idle recovery can
 * clear the latch. A new fault invalidates all previous TX completion evidence. */
static void CanObserveRecovery(uint32_t now)
{
    static uint32_t generation;
    uint32_t ok = tx_ok_count;
    if (!tx_fault || !CanHardwareHealthy() || generation != fault_generation) {
        generation = fault_generation;
        auto_clear_armed = 0U;
        auto_clear_ok_seen = ok;
        recovery_phase = tx_fault ? 1U : 0U;
        return;
    }
    if (ok != auto_clear_ok_seen && !auto_clear_armed) {
        auto_clear_armed = 1U;
        auto_clear_since = now;
    }
    if (auto_clear_armed && (uint32_t)(now - auto_clear_since) >= CAN_FAULT_AUTO_CLEAR_MS)
        recovery_phase = 2U;
}

/*
 * 轮询实时 ESR：原先 EWGF/EPVF 依赖电平式中断，会形成中断风暴。
 * 这里只做“当前是否不健康”的采样，并在 EPV/BOFF 时锁存发送故障。
 */
static void CanPollEsrState(void)
{
    uint32_t esr = hcan1.Instance->ESR;
    if (esr & (CAN_ESR_EPVF | CAN_ESR_BOFF)) {
        error_latched |= (esr & CAN_ESR_EPVF) ? HAL_CAN_ERROR_EPV : 0U;
        error_latched |= (esr & CAN_ESR_BOFF) ? HAL_CAN_ERROR_BOF : 0U;
        CanLatchFault();
    }
}

/*
 * 邮箱卡死恢复。
 *
 * 现场表现：FREE=0、TX_OK/RX 冻结，但 TX_TIMEOUT/TX_ABORT 仍在增长——
 * 说明 AbortTxRequest 返回成功却清不掉 TME，普通 50ms 超时路径无法解锁。
 * 此时必须丢弃软件队列，并在 Abort 后仍 FREE=0 时对 CAN 外设做 Stop/Start。
 */
static void CanForceUnstick(void)
{
    uint32_t now = HAL_GetTick();

    /* 即使不能立刻 Stop/Start，也要锁存故障并重新武装观察窗。 */
    CanLatchFault();
    last_tx_result = 4U;
    stall_armed = 0U;
    stall_ok_mark = tx_ok_count;
    if (last_unstick_tick != 0U &&
        (uint32_t)(now - last_unstick_tick) < CAN_UNSTICK_MIN_GAP_MS) {
        return;
    }
    last_unstick_tick = now;

    (void)HAL_CAN_AbortTxRequest(&hcan1,
        CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    CanDropPendingQueue();
    /* Keep unsent STOP slots: recovery must not discard a safety request. */
    tracked_mailboxes = 0U;
    stop_mailboxes = 0U;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        /* Start resets HAL ErrorCode. Preserve diagnostics first, and never
         * report a successful recovery or erase errors after a failed step. */
        error_latched |= HAL_CAN_GetError(&hcan1);
        if (HAL_CAN_Stop(&hcan1) != HAL_OK ||
            HAL_CAN_Start(&hcan1) != HAL_OK ||
            CanEnableNotifications() != HAL_OK) {
            error_latched |= HAL_CAN_GetError(&hcan1);
            return;
        }
    }
    stall_recoveries++;
}

static void CanCheckTxStall(uint32_t now)
{
    uint32_t esr = hcan1.Instance->ESR;
    uint8_t busy;
    uint8_t i;

    /*
     * 判据：TXOK 长期不增 + 仍在尝试发送（队列/在途/邮箱占用/停车槽）。
     * 不能用 “FREE==0 一直成立” 当必要条件：超时-Abort 震荡会让 FREE 在
     * 0↔2 之间抖动，每次非 0 都清计时，STALL_REC 永远为 0（现场已复现）。
     * 一旦开计，只有 TXOK 进展才能解除，避免 FREE 短暂弹起就复位。
     */
    /* Bus errors are not proof of a stuck mailbox. Leave retransmission and
     * bus-off recovery to hardware; retain the existing safety readiness gate. */
    if (esr & (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) {
        stall_armed = 0U;
        stall_ok_mark = tx_ok_count;
        return;
    }
    if (tx_ok_count != stall_ok_mark) {
        stall_ok_mark = tx_ok_count;
        stall_armed = 0U;
        return;
    }
    if (!stall_armed) {
        busy = (command_count != 0U) || (tracked_mailboxes != 0U) ||
               (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U);
        if (!busy) {
            for (i = 0U; i < 4U; ++i) {
                if (speeds[i].pending || stops[i].pending) { busy = 1U; break; }
            }
        }
        if (!busy) return;
        stall_armed = 1U;
        stall_since = now;
        return;
    }
    if ((uint32_t)(now - stall_since) < CAN_TX_STALL_MS) return;
    /* A completed/aborted batch with free mailboxes needs no forced recovery. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0U) {
        stall_armed = 0U;
        return;
    }
    CanForceUnstick();
}

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
        dropped++; CanLatchFault(); return 1U;
    }
    enqueue(&commands[command_head], id, data, length);
    command_head = (uint8_t)((command_head + 1U) % CAN_QUEUE_SIZE);
    command_count++;
    return 0U;
}

void ZDT_CAN_BeginStop(void)
{
    uint8_t i;
    CanDropPendingQueue();
    for (i = 0U; i < 4U; ++i) {
        stops[i].pending = 0U;
    }
    /* Cancel unsent hardware requests as well as the software queue. */
    (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    tracked_mailboxes = 0U;
    stop_mailboxes = 0U;
    stop_started = HAL_GetTick();
    /* 刻意不重置 stall 观察窗：停车确认失败时 StopMask 每 100ms 会调
     * BeginStop，若在这里清零，FREE=0 的卡死计时永远到不了 150ms，
     * STALL_REC 将一直为 0（现场已复现）。 */
}

uint8_t ZDT_CAN_SendStop(uint32_t id, uint8_t *data, uint8_t length)
{
    uint32_t motor = id >> 8;
    if (!data || length == 0U || length > 8U || motor < 1U || motor > 4U) return 3U;
    return enqueue(&stops[motor - 1U], id, data, length);
}

/* 回收已完成的邮箱；返回需要在本次申请撤销的邮箱掩码。 */
static uint32_t CanReclaimMailboxes(uint32_t now)
{
    uint32_t expired = 0U;
    uint8_t i;
    for (i = 0U; i < 3U; ++i) {
        uint32_t mask = 1UL << i;
        if (!(tracked_mailboxes & mask)) continue;
        if (!HAL_CAN_IsTxMessagePending(&hcan1, mask)) tracked_mailboxes &= ~mask;
        else if ((uint32_t)(now - mailbox_started[i]) > CAN_TX_TIMEOUT_MS) expired |= mask;
    }
    return expired;
}

/*
 * 丢弃软件待发队列（停车队列除外）。邮箱超时或队列等待超时后必须清空，
 * 否则总线恢复的瞬间会把已经过期的运动指令补发出去。
 */
static void CanDropPendingQueue(void)
{
    uint8_t i;
    dropped += command_count;
    command_head = command_tail = command_count = 0U;
    for (i = 0U; i < 4U; ++i) {
        dropped += speeds[i].pending;
        speeds[i].pending = 0U;
    }
}

/* 按 停车 > 高优先命令 > 每电机速度槽 的顺序选出下一个待发项。 */
static PendingCan *CanPickNext(uint8_t *urgent)
{
    uint8_t i;
    *urgent = 0U;
    for (i = 0U; i < 4U; ++i) {
        if (stops[i].pending) { *urgent = 1U; return &stops[i]; }
    }
    if (command_count != 0U) return &commands[command_tail];
    for (i = 0U; i < 4U; ++i) {
        if (speeds[i].pending) return &speeds[i];
    }
    return NULL;
}

/* 装入硬件邮箱。返回 0 表示本轮发送循环应终止。 */
static uint8_t CanTransmit(PendingCan *slot, uint8_t urgent, uint32_t now)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t motor_index = (uint8_t)((slot->id >> 8) - 1U);
    uint8_t k;

    /*
     * 使能帧发出后短暂延后同一电机的速度帧：驱动器需要时间完成锁轴，
     * 紧跟的速度指令可能被忽略。
     */
    if (!urgent && (enable_wait_mask & (uint8_t)(1U << motor_index))) {
        if ((uint32_t)(now - enabled_at[motor_index]) < 5U) return 0U;
        enable_wait_mask &= (uint8_t)~(1U << motor_index);
    }
    if (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) return 0U;

    header.ExtId = slot->id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = slot->length;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, slot->bytes, &mailbox) != HAL_OK) {
        tx_error_count++; last_tx_result = 2U; CanLatchFault(); return 0U;
    }
    if (urgent) stop_mailboxes |= mailbox;
    if (slot->bytes[0] == 0xF3U) {
        enabled_at[motor_index] = now;
        enable_wait_mask |= (uint8_t)(1U << motor_index);
    }
    slot->pending = 0U;
    if (!urgent && command_count && slot == &commands[command_tail]) {
        command_tail = (uint8_t)((command_tail + 1U) % CAN_QUEUE_SIZE); command_count--;
    }
    for (k = 0U; k < 3U; ++k) {
        if (mailbox & (1UL << k)) mailbox_started[k] = now;
    }
    tracked_mailboxes |= mailbox;
    tx_queued++;
    return 1U;
}

void ZDT_CAN_Process(uint32_t now)
{
    uint8_t budget;
    uint32_t expired;

    CanObserveRecovery(now);
    CanPollEsrState();
    CanCheckTxStall(now);

    /* 即使软件队列为空也要检查硬件，且不做任何阻塞等待。 */
    expired = CanReclaimMailboxes(now);
    if (expired != 0U) {
        uint8_t i;
        CanLatchFault();
        last_tx_result = 4U;
        if (HAL_CAN_AbortTxRequest(&hcan1, expired) == HAL_OK) {
            for (i = 0U; i < 3U; ++i) if (expired & (1UL << i)) tx_timeout++;
            tracked_mailboxes &= ~expired;
        }
        CanDropPendingQueue();
        return;
    }

    for (budget = 0U; budget < 3U; ++budget) {
        uint8_t urgent;
        uint32_t age;
        PendingCan *slot = CanPickNext(&urgent);

        /*
         * 只有在没有停车帧待发时才等上一批停车帧离开邮箱；否则会推迟
         * 新到的停车请求。等待期间本轮不再发送其他命令。
         */
        if (slot == NULL || !urgent) {
            if (stop_mailboxes != 0U) {
                if (HAL_CAN_IsTxMessagePending(&hcan1, stop_mailboxes)) {
                    if ((uint32_t)(now - stop_started) > CAN_TX_TIMEOUT_MS) CanLatchFault();
                    return;
                }
                stop_mailboxes = 0U;
            }
        }
        if (slot == NULL) return;

        age = now - slot->queued_at;
        if (age > max_wait) max_wait = age;
        if (age > CAN_TX_TIMEOUT_MS) {
            CanLatchFault();
            /* 停车请求即使过期也要发出去；普通命令直接丢弃。 */
            if (!urgent) {
                CanDropPendingQueue();
                return;
            }
        }
        if (!CanTransmit(slot, urgent, now)) return;
    }
}
uint8_t ZDT_CAN_ConsumeFault(void) {
    uint32_t saved = __get_PRIMASK();
    uint8_t value;
    __disable_irq();
    value = fault_event; fault_event = 0U;
    __set_PRIMASK(saved);
    return value;
}

void ZDT_CAN_RaiseFault(void) { CanLatchFault(); }

uint8_t ZDT_CAN_HasFault(void) { return tx_fault; }
uint8_t ZDT_CAN_HardwareReady(void) { return CanHardwareHealthy(); }

#ifdef CONTROL_HOST_TEST
/* Fixture isolation only; production has no unconditional latch-clear API. */
void ZDT_CAN_TestResetFault(void)
{
    tx_fault = fault_event = auto_clear_armed = recovery_phase = 0U;
    fault_generation++;
}
#endif

uint8_t ZDT_CAN_IsReady(void)
{
    return CanHardwareHealthy() && !tx_fault;
}

uint8_t ZDT_CAN_RecoverWhenIdle(uint8_t eligible)
{
    static uint8_t watching;
    static uint32_t since, errors, submits, timeouts, drops, completed, received, generation;
    uint32_t saved = __get_PRIMASK();
    uint32_t now;
    uint8_t recovered = 0U;
    __disable_irq();
    now = HAL_GetTick();
    /* Hardware must be clean, and programming/configuration errors are not
     * eligible for automatic recovery. 0x1FFFF covers HAL CAN bus errors. */
    if (!eligible || HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING ||
        (hcan1.Instance->ESR & (0xFFFF0000UL | CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) ||
        (HAL_CAN_GetError(&hcan1) & ~0x0001FFFFUL) ||
        /*
         * 只有真正的传输故障或致命错误才值得走恢复流程。纯粹的接收侧事件
         * (位填充/格式/位隐性/ACK 错误)不含 CAN_FATAL_HAL_ERRORS 位，否则
         * 空闲期它们会让这个 500ms 窗口反复触发并打印 CAN RECOVERED，
         * 把串口刷满噪声。
         */
        (!tx_fault && !(HAL_CAN_GetError(&hcan1) & CAN_FATAL_HAL_ERRORS))) {
        watching = 0U;
    } else if (!watching || generation != fault_generation || errors != error_callbacks || submits != tx_error_count ||
               timeouts != tx_timeout || drops != dropped) {
        watching = 1U; since = now; generation = fault_generation;
        errors = error_callbacks; submits = tx_error_count;
        timeouts = tx_timeout; drops = dropped;
        completed = tx_ok_count; received = rx_count;
    } else if ((uint32_t)(now - since) >= 500U &&
               completed != tx_ok_count && received != rx_count) {
        error_latched |= HAL_CAN_GetError(&hcan1);
        if (HAL_CAN_ResetError(&hcan1) == HAL_OK) {
            tx_fault = 0U;
            recovery_phase = auto_clear_armed = 0U;
            recoveries++;
            recovered = 1U;
        }
        watching = 0U;
    }
    __set_PRIMASK(saved);
    return recovered;
}

/* Only TXOK callbacks prove completion; CAN2 callbacks are intentionally ignored. */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) { tx_ok_count++; last_tx_result = 0U; }
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_TxMailbox0CompleteCallback(hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_TxMailbox0CompleteCallback(hcan);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) tx_aborted++;
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_TxMailbox0AbortCallback(hcan);
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_TxMailbox0AbortCallback(hcan);
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t error;
    uint32_t esr;
    if (hcan != &hcan1) return;
    error = HAL_CAN_GetError(hcan);
    error_callbacks++;
    /* Arbitration loss and isolated protocol/TX errors are diagnostic events.
     * Feedback freshness, mailbox timeout and the current ESR state decide
     * whether communication has actually become unsafe. */
    error_latched |= error;
    last_tx_result = 5U;
    /*
     * hcan->ErrorCode 是 |= 累积的：历史上有过一次 Error Passive 或 Bus-Off，
     * 对应的位就会永久保留，只有 HAL_CAN_ResetError 才清。若直接用它判断
     * “当前是否致命”，此后每一次普通错误回调(例如多节点总线上完全正常的
     * 仲裁丢失 TX_ALST、或偶发的位填充/格式错误)都会锁存 tx_fault，现场表现
     * 为空闲期 CAN RECOVERED 反复刷屏、PID 轮次中途 ROUND STOP CAN FAULT。
     * 因此把这两位从累积值中剥离，改由不会累积的实时 ESR 决定。
     */
    error &= ~(HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_BOF);
    esr = hcan1.Instance->ESR;
    if (esr & CAN_ESR_EPVF) error |= HAL_CAN_ERROR_EPV;
    if (esr & CAN_ESR_BOFF) error |= HAL_CAN_ERROR_BOF;
    if (error & CAN_FATAL_HAL_ERRORS) {
        if (!tx_fault) fatal_error_callbacks++;
        CanLatchFault();
    }
}

void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    uint8_t budget = 3U;
    while (budget-- && HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            rx_count++;
            if (pRxCallback != NULL && RxHeader.IDE == CAN_ID_EXT &&
                RxHeader.RTR == CAN_RTR_DATA && RxHeader.DLC <= 8U) {
                pRxCallback(RxHeader.ExtId, RxData, RxHeader.DLC);
            }
        }
    }
}

void ZDT_CAN_GetStats(ZDT_CAN_Stats_t *stats) {
    if (stats == NULL) return;
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    stats->tx_ok = tx_ok_count;
    stats->tx_queued = tx_queued; stats->tx_aborted = tx_aborted;
    stats->tx_timeout = tx_timeout;
    stats->recoveries = recoveries;
    stats->auto_recoveries = auto_recoveries;
    stats->stall_recoveries = stall_recoveries;
    stats->error_callbacks = error_callbacks;
    stats->fatal_error_callbacks = fatal_error_callbacks;
    stats->error_latched = error_latched;
    stats->esr = hcan1.Instance->ESR; stats->tsr = hcan1.Instance->TSR;
    stats->tx_error = tx_error_count;
    stats->rx_count = rx_count;
    stats->last_tx_result = last_tx_result;
    stats->tx_fault = tx_fault;
    stats->fault_generation = fault_generation;
    stats->recovery_phase = recovery_phase;
    stats->enqueued = enqueued; stats->replaced = replaced;
    stats->dropped = dropped; stats->max_wait_ms = max_wait;
    __set_PRIMASK(saved);
}

uint8_t ZDT_CAN_StopPending(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; ++i) if (stops[i].pending) return 1U;
    return stop_mailboxes && HAL_CAN_IsTxMessagePending(&hcan1, stop_mailboxes);
}
