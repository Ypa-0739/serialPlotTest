#include "control_test_hal.h"
#include "control_runtime.h"
#include "host_uart_tx.h"
#include "mecanum_chassis.h"
#include "ops9.h"
#include "pid.h"
#include "zdtCan.h"
#include "zdtEmm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static CAN_TestRegisters can_regs;
CAN_HandleTypeDef hcan1 = {&can_regs};
static uint32_t can_error;
static uint32_t can_state = HAL_CAN_STATE_LISTENING;
uint32_t HAL_CAN_GetState(const CAN_HandleTypeDef *h) { (void)h; return can_state; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { (void)h; can_error = 0; return HAL_OK; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h) { (void)h; return can_error; }
UART_HandleTypeDef huart1, huart2 = {USART2, HAL_UART_STATE_READY};
static uint32_t tick, primask, pending, sent_count, abort_count, rx_remaining;
static uint8_t auto_complete, fail_can, fail_uart, fail_abort;
static uint32_t last_notify_mask, can_stop_count;
static uint8_t fail_can_stop;
static struct { uint32_t id; uint8_t bytes[8]; } sent[256];
static const uint8_t *dma_bytes;
static uint16_t dma_length;
static uint32_t dma_count;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void __set_PRIMASK(uint32_t value) { primask = value; }
void Error_Handler(void) { assert(0); }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{ (void)h; (void)f; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{
    (void)h; can_stop_count++;
    if (fail_can_stop) { can_error |= HAL_CAN_ERROR_TIMEOUT; return HAL_ERROR; }
    pending = 0U; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t n)
{ (void)h; last_notify_mask = n; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mask)
{
    (void)h; abort_count++;
    /* fail_abort 模拟 Abort 返回成功但硬件 TME 清不掉。 */
    if (!fail_abort) pending &= ~mask;
    return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *h, uint32_t mask)
{ (void)h; return pending & mask; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h)
{ (void)h; return 3U - !!(pending & 1U) - !!(pending & 2U) - !!(pending & 4U); }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr,
                                     uint8_t *bytes, uint32_t *mailbox)
{
    uint32_t mask;
    (void)h;
    if (fail_can) return HAL_ERROR;
    for (mask = 1U; mask <= 4U; mask <<= 1U) if (!(pending & mask)) break;
    assert(mask <= 4U && sent_count < 256U);
    *mailbox = mask;
    sent[sent_count].id = hdr->ExtId;
    memcpy(sent[sent_count++].bytes, bytes, hdr->DLC);
    if (!auto_complete) pending |= mask;
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return rx_remaining; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *hdr, uint8_t *bytes)
{
    (void)h; (void)fifo;
    memset(hdr, 0, sizeof(*hdr)); memset(bytes, 0, 8U);
    if (rx_remaining) rx_remaining--;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t length)
{
    assert(primask == 1U); /* Completion must not race the owner's busy flag. */
    if (fail_uart) return HAL_ERROR;
    assert(h->gState == HAL_UART_STATE_READY);
    h->gState = 1U; dma_bytes = bytes; dma_length = length; dma_count++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t length)
{ (void)h; assert(length == 4U && memcmp(bytes, "ACT0", 4U) == 0); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t length)
{ (void)h; (void)bytes; (void)length; return HAL_OK; }
static void near(float a, float b) { assert(fabsf(a - b) < 0.0001f); }
static void uart_done(void) { huart1.gState = HAL_UART_STATE_READY; HostUartTx_Complete(); }
static void uart_reset(void)
{ huart1.gState = HAL_UART_STATE_READY; fail_uart = 0U; HostUartTx_Reset(); }
static void write_line(const char *line) { assert(HostUartTx_Write(line, (int)strlen(line)) > 0); }
static void expect_line(const char *line)
{ assert(dma_length == strlen(line) && memcmp(dma_bytes, line, dma_length) == 0); }
static void can_reset(uint32_t now)
{
    tick = now;
    /* 先让邮箱空闲跑一轮 Process，清掉卡死观察窗，再走 BeginStop。 */
    pending = 0U;
    ZDT_CAN_Process(tick);
    ZDT_CAN_BeginStop();
    ZDT_CAN_TestResetFault();
    sent_count = 0U; fail_can = auto_complete = fail_abort = 0U;
    can_regs.ESR = 0U; can_error = 0U;
    can_state = HAL_CAN_STATE_LISTENING;
}
static void record_all(MotorFeedback samples[4], float rpm, uint32_t now)
{ unsigned i; for (i = 0U; i < 4U; ++i) MotorFeedback_Record(&samples[i], rpm, now); }
static void driver_feedback(unsigned id, uint16_t rpm)
{
    uint8_t bytes[5] = {0x35, 0, (uint8_t)(rpm >> 8), (uint8_t)rpm, 0x6B};
    ZDT_Emm_RxHandler(id << 8, bytes, sizeof(bytes));
}

static void test_pid_dt_and_limits(void)
{
    PID_Controller a, b;
    unsigned i;
    PID_Init(&a, 0, 1, 0, 1000, 1000); PID_Init(&b, 0, 1, 0, 1000, 1000);
    for (i = 0; i < 5; ++i) PID_CalcErrorDt(&a, 2, .020f);
    for (i = 0; i < 10; ++i) PID_CalcErrorDt(&b, 2, .010f);
    near(a.integral, b.integral); near(a.integral, 10);
    PID_CalcErrorDt(&a, 2, .035f); near(a.integral, 13.5f);
    PID_ApplyOutput(&a, 1); near(a.integral, 10);
    PID_CalcErrorDt(&a, -2, .020f); PID_ApplyOutput(&a, 0); near(a.integral, 8);
    PID_Reset(&a); a.max_out = 1; PID_CalcErrorDt(&a, 2, .020f); near(a.integral, 0);
    PID_Init(&a, 0, 0, 1, 1000, 1000);
    near(PID_CalcErrorDt(&a, 100, .020f), 0); /* No derivative kick at start. */
    near(PID_CalcErrorDt(&a, 102, .020f), 1); /* tau = 20 ms => alpha = 1/2. */
    a.derivative_tau_s = 0; near(PID_CalcErrorDt(&a, 103, .010f), 2);
    near(PID_CalcErrorDt(&a, 105, .020f), 2);
    near(PID_CalcErrorDt(&a, NAN, .020f), 0); assert(!a.has_last_error);
    near(PID_CalcErrorDt(&a, 2, 0), 0); near(PID_CalcErrorDt(&a, 2, .101f), 0);
    PID_Init(&a, 1, 0, 0, 100, 100); PID_SetTarget(&a, 5); near(PID_Calc(&a, 2), 3);
}

static void test_stop_confirmation(void)
{
    MotorFeedback samples[4] = {0};
    MotorStopMonitor stop = {0};
    record_all(samples, 0, 90); record_all(samples, 0, 95);
    MotorStop_Request(&stop, 100);
    MotorStop_Update(&stop, samples, 1, 100); assert(stop.state == MOTOR_STOP_REQUESTED);
    record_all(samples, 0, 110); record_all(samples, 0, 120);
    MotorStop_Update(&stop, samples, 0, 120); assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    record_all(samples, 0, 130);
    MotorStop_Update(&stop, samples, 0, 130); assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    record_all(samples, 0, 140);
    MotorStop_Update(&stop, samples, 0, 140); assert(stop.state == MOTOR_STOP_CONFIRMED);
    MotorFeedback_Record(&samples[2], 10, 150);
    MotorStop_Update(&stop, samples, 0, 150); assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    MotorStop_Request(&stop, 600); assert(stop.requested_tick == 100);
    MotorStop_Update(&stop, samples, 0, 701); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    record_all(samples, 0, 710); record_all(samples, 0, 750);
    MotorStop_Update(&stop, samples, 0, 750); assert(stop.state == MOTOR_STOP_CONFIRMED);
    MotorStop_Update(&stop, samples, 0, 1051); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    assert(MotorFeedback_FreshMask(samples, 1051) == 0);
    memset(&stop, 0, sizeof(stop));
    MotorStop_Request(&stop, 2000); record_all(samples, 0, 2601);
    MotorStop_Update(&stop, samples, 1, 2601); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    MotorStop_Update(&stop, samples, 0, 2602); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    record_all(samples, 0, 2610); record_all(samples, 0, 2620);
    MotorStop_Update(&stop, samples, 0, 2620); assert(stop.state == MOTOR_STOP_CONFIRMED);
    memset(&stop, 0, sizeof(stop));
    MotorStop_Request(&stop, UINT32_MAX - 20U);
    MotorStop_Update(&stop, samples, 0, UINT32_MAX - 20U);
    record_all(samples, 0, UINT32_MAX - 5U); record_all(samples, 0, 10U);
    MotorStop_Update(&stop, samples, 0, 10U); assert(stop.state == MOTOR_STOP_CONFIRMED);
    assert(MotorFeedback_FreshMask(samples, 20U) == 15U);

    /* A selected motor can confirm a stop without feedback from absent motors. */
    memset(samples, 0, sizeof(samples));
    memset(&stop, 0, sizeof(stop));
    MotorStop_Request(&stop, 3000U);
    MotorFeedback_Record(&samples[0], 0.0f, 3010U);
    MotorStop_UpdateMasked(&stop, samples, 0x01U, 0U, 3010U);
    assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    MotorFeedback_Record(&samples[0], 0.0f, 3020U);
    MotorFeedback_Record(&samples[0], 0.0f, 3030U);
    MotorStop_UpdateMasked(&stop, samples, 0x01U, 0U, 3030U);
    assert(stop.state == MOTOR_STOP_CONFIRMED);
    assert((MotorFeedback_FreshMask(samples, 3030U) & 0x0EU) == 0U);
}

static void test_can_queue_and_stop(void)
{
    uint8_t speed[7] = {0xF6, 0, 0, 20, 0, 0, 0x6B};
    uint8_t query[2] = {0x35, 0x6B};
    unsigned i;
    can_reset(100); pending = 7U;
    for (i = 1; i <= 4; ++i) assert(!ZDT_CAN_Send_ExtId(i << 8, speed, 7));
    speed[3] = 40; assert(!ZDT_CAN_Send_ExtId(0x100, speed, 7));
    ZDT_CAN_Process(tick); assert(sent_count == 0);
    pending = 0; auto_complete = 1;
    ZDT_CAN_Process(tick); assert(sent_count == 3 && sent[0].bytes[3] == 40);
    ZDT_CAN_Process(tick); assert(sent_count == 4);
    assert(!ZDT_CAN_Send_ExtId(0x100, query, 2));
    assert(!ZDT_CAN_Send_ExtId(0x100, speed, 7));
    pending = 7; auto_complete = 0;
    ZDT_CAN_BeginStop(); assert(pending == 0 && abort_count > 0);
    speed[3] = 0;
    for (i = 1; i <= 4; ++i) assert(!ZDT_CAN_SendStop(i << 8, speed, 7));
    assert(!ZDT_CAN_Send_ExtId(0x100, query, 2));
    sent_count = 0; ZDT_CAN_Process(tick);
    assert(sent_count == 3 && ZDT_CAN_StopPending());
    for (i = 0; i < 3; ++i) assert(sent[i].bytes[0] == 0xF6 && sent[i].bytes[3] == 0);
    pending = 0; ZDT_CAN_Process(tick); assert(sent_count == 4 && sent[3].id == 0x400);
    ZDT_CAN_Process(tick); assert(sent_count == 4); /* Last STOP still in mailbox. */
    pending = 0; assert(!ZDT_CAN_StopPending());
    ZDT_CAN_Process(tick); assert(sent_count == 5 && sent[4].bytes[0] == 0x35);
}

static void test_can_timeout_enable_and_rx_budget(void)
{
    uint8_t query[] = {0x35, 0x6B};
    uint8_t speed[] = {0xF6, 0, 0, 10, 0, 0, 0x6B};
    uint8_t enable[] = {0xF3, 0xAB, 1, 0, 0x6B};
    unsigned i;
    can_reset(1000); pending = 7;
    assert(!ZDT_CAN_Send_ExtId(0x100, speed, 7));
    tick += 40; assert(!ZDT_CAN_Send_ExtId(0x100, speed, 7));
    tick += 11; ZDT_CAN_Process(tick); assert(ZDT_CAN_ConsumeFault());
    pending = 0; ZDT_CAN_Process(tick); assert(!sent_count);
    can_reset(2000);
    for (i = 0; i < 16; ++i) assert(!ZDT_CAN_Send_ExtId(0x100, query, 2));
    assert(ZDT_CAN_Send_ExtId(0x100, query, 2)); assert(ZDT_CAN_ConsumeFault());
    can_reset(UINT32_MAX - 2U); auto_complete = 1;
    assert(!ZDT_CAN_Send_ExtId(0x100, enable, 5));
    assert(!ZDT_CAN_Send_ExtId(0x100, speed, 7));
    ZDT_CAN_Process(tick); assert(sent_count == 1);
    tick = 1; ZDT_CAN_Process(tick); assert(sent_count == 1);
    tick = 2; ZDT_CAN_Process(tick); assert(sent_count == 2);
    can_reset(0x80000010U); auto_complete = 1;
    assert(!ZDT_CAN_Send_ExtId(0x200, query, 2));
    ZDT_CAN_Process(tick); assert(sent_count == 1); /* Long uptime, never enabled. */
    fail_can = 1; assert(!ZDT_CAN_Send_ExtId(0x200, query, 2));
    ZDT_CAN_Process(tick); assert(ZDT_CAN_ConsumeFault());
    rx_remaining = 10; ZDT_CAN_RxFIFO0_Handler(&hcan1); assert(rx_remaining == 7);
    can_reset(3000); auto_complete = 0;
    assert(!ZDT_CAN_SendStop(0x100, speed, 7)); ZDT_CAN_Process(tick);
    tick += 51; ZDT_CAN_Process(tick); assert(ZDT_CAN_ConsumeFault());
}

static void test_can_diagnostics_and_mailbox_expiry(void)
{
    uint8_t query[] = {0x35, 0x6B};
    uint8_t speed[] = {0xF6, 0, 0, 10, 0, 0, 0x6B};
    CAN_HandleTypeDef other = {0};
    ZDT_CAN_Stats_t before, after;
    can_reset(7000);
    ZDT_CAN_GetStats(&before);
    assert(!ZDT_CAN_Send_ExtId(0x100, query, 2));
    ZDT_CAN_Process(tick);
    ZDT_CAN_GetStats(&after);
    assert(after.tx_queued == before.tx_queued + 1 && after.tx_ok == before.tx_ok);
    assert(sent[0].id == 0x100 && sent[0].bytes[0] == 0x35 && sent[0].bytes[1] == 0x6B);
    HAL_CAN_TxMailbox0CompleteCallback(&other);
    HAL_CAN_ErrorCallback(&other);
    ZDT_CAN_GetStats(&after);
    assert(after.tx_ok == before.tx_ok && after.error_callbacks == before.error_callbacks);
    pending = 0;
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    HAL_CAN_TxMailbox1CompleteCallback(&hcan1);
    HAL_CAN_TxMailbox2CompleteCallback(&hcan1);
    ZDT_CAN_GetStats(&after); assert(after.tx_ok == before.tx_ok + 3);
    can_error = HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&hcan1);
    ZDT_CAN_GetStats(&after);
    assert(after.error_callbacks == before.error_callbacks + 1);
    assert((after.error_latched & can_error) == can_error);
    /*
     * ErrorCode 里的 BOF/EPV 是 |= 累积位，可能只是历史遗留。此时实时 ESR
     * 干净，就不该锁存 tx_fault —— 否则此后每一次普通错误回调(例如正常的
     * 仲裁丢失)都会被误判为致命，现场表现为 CAN RECOVERED 反复刷屏。
     */
    assert(!ZDT_CAN_ConsumeFault());
    assert(ZDT_CAN_IsReady());
    /* 实时 ESR 真的处于 Bus-Off 时才锁存发送故障。 */
    can_regs.ESR = CAN_ESR_BOFF;
    HAL_CAN_ErrorCallback(&hcan1);
    ZDT_CAN_GetStats(&after);
    assert(after.fatal_error_callbacks == before.fatal_error_callbacks + 1);
    assert(ZDT_CAN_ConsumeFault());
    assert(!ZDT_CAN_ConsumeFault());
    can_regs.ESR = 0;
    before = after;
    can_error = HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_TX_ALST0;
    HAL_CAN_ErrorCallback(&hcan1);
    ZDT_CAN_GetStats(&after);
    assert(after.error_callbacks == before.error_callbacks + 1);
    assert(after.fatal_error_callbacks == before.fatal_error_callbacks);
    assert(!ZDT_CAN_IsReady()); /* Prior Bus-Off latch survives event consumption. */
    assert(!ZDT_CAN_ConsumeFault());
    can_error = 0;
    /* Empty software queue must not prevent expiry; subtraction must wrap safely. */
    can_reset(UINT32_MAX - 20U);
    ZDT_CAN_GetStats(&before);
    assert(!ZDT_CAN_Send_ExtId(0x100, query, 2)); ZDT_CAN_Process(tick);
    tick = 29U; ZDT_CAN_Process(tick); assert(pending == 1U);
    tick = 30U; ZDT_CAN_Process(tick); assert(pending == 0U);
    HAL_CAN_TxMailbox0AbortCallback(&hcan1);
    ZDT_CAN_GetStats(&after);
    assert(after.tx_timeout == before.tx_timeout + 1);
    assert(after.tx_aborted == before.tx_aborted + 1 && after.tx_ok == before.tx_ok);
    assert(ZDT_CAN_ConsumeFault());
    can_reset(8000);
    assert(!ZDT_CAN_Send_ExtId(0x100, query, 2)); ZDT_CAN_Process(tick);
    tick += 51;
    assert(!ZDT_CAN_Send_ExtId(0x100, speed, 7));
    ZDT_CAN_Process(tick); ZDT_CAN_Process(tick);
    assert(sent_count == 1); /* Fresh queued motion is discarded with the stalled request. */
}

static void test_can_stopped_recovery(void)
{
    ZDT_CAN_Stats_t before, after;
    can_reset(10000);
    can_regs.ESR = 0;
    /*
     * 累积 ErrorCode 里的 BOF 只是历史遗留：实时 ESR 干净时不应锁存 tx_fault，
     * 也不该触发恢复流程（否则空闲期会把 CAN RECOVERED 刷满串口）。
     */
    can_error = HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&hcan1);
    ZDT_CAN_GetStats(&before);
    assert(ZDT_CAN_IsReady());
    assert(!ZDT_CAN_RecoverWhenIdle(1));

    /* 实时 ESR 真的处于 Bus-Off 时才锁存发送故障。 */
    can_regs.ESR = CAN_ESR_BOFF;
    HAL_CAN_ErrorCallback(&hcan1);
    ZDT_CAN_GetStats(&before);
    assert(!ZDT_CAN_IsReady());
    assert(!ZDT_CAN_RecoverWhenIdle(0)); /* Running or feedback/stop not ready. */
    tick += 1000;
    assert(!ZDT_CAN_RecoverWhenIdle(1));
    tick += 500;
    assert(!ZDT_CAN_RecoverWhenIdle(1)); /* ESR 仍是 Bus-Off，不允许恢复 */

    can_regs.ESR = 0;                    /* 硬件自动退出 Bus-Off */
    tick += 500;
    assert(!ZDT_CAN_RecoverWhenIdle(1)); /* Quiet alone does not prove communication. */
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    rx_remaining = 1; ZDT_CAN_RxFIFO0_Handler(&hcan1);
    tick += 500;
    assert(ZDT_CAN_RecoverWhenIdle(1));
    assert(ZDT_CAN_IsReady());
    assert(ZDT_CAN_ConsumeFault()); /* Recovery does not erase unread history. */
    ZDT_CAN_GetStats(&after);
    assert(after.recoveries == before.recoveries + 1);
    assert(after.error_latched == before.error_latched); /* History survives recovery. */
    assert(!ZDT_CAN_RecoverWhenIdle(1)); /* No repeated recovery notifications. */

    can_error = HAL_CAN_ERROR_ACK;
    HAL_CAN_ErrorCallback(&hcan1);
    assert(!ZDT_CAN_RecoverWhenIdle(1)); /* 纯接收错误不进入恢复流程 */
    tick += 499;
    HAL_CAN_ErrorCallback(&hcan1);
    assert(!ZDT_CAN_RecoverWhenIdle(1));
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    rx_remaining = 1; ZDT_CAN_RxFIFO0_Handler(&hcan1);
    tick += 499; assert(!ZDT_CAN_RecoverWhenIdle(1));
    assert(!ZDT_CAN_RecoverWhenIdle(0));
    tick++; assert(!ZDT_CAN_RecoverWhenIdle(1));
    can_regs.ESR = CAN_ESR_BOFF;
    tick += 500; assert(!ZDT_CAN_RecoverWhenIdle(1));
    assert(!ZDT_CAN_IsReady());
    can_regs.ESR = 1UL << 16; /* TEC is nonzero even without BOFF. */
    assert(!ZDT_CAN_RecoverWhenIdle(1));
    can_regs.ESR = 0;
    can_state = 0;
    assert(!ZDT_CAN_RecoverWhenIdle(1));
    can_state = HAL_CAN_STATE_LISTENING;
    can_error = 0x00200000U; /* HAL parameter errors need a code/configuration fix. */
    assert(!ZDT_CAN_RecoverWhenIdle(1));

    /* 真实的 Error Passive 才需要恢复；验证 tick 回绕后的观察窗口。 */
    can_regs.ESR = CAN_ESR_EPVF;
    HAL_CAN_ErrorCallback(&hcan1);
    can_regs.ESR = 0;
    assert(!ZDT_CAN_IsReady());
    can_error = HAL_CAN_ERROR_ACK;
    tick = UINT32_MAX - 100U;
    assert(!ZDT_CAN_RecoverWhenIdle(1));
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    rx_remaining = 1; ZDT_CAN_RxFIFO0_Handler(&hcan1);
    tick = 399U; assert(ZDT_CAN_RecoverWhenIdle(1));
    assert(ZDT_CAN_IsReady());
}

static void test_uart_dma_ownership_and_priority(void)
{
    uint8_t saved[512];
    uint16_t saved_length;
    uart_reset(); tick = 10;
    write_line("# FRAG"); HostUartTx_Process(tick); assert(!HostUartTx_Busy());
    write_line("MENT\r\n"); HostUartTx_Process(tick); expect_line("# FRAGMENT\r\n");
    saved_length = dma_length; memcpy(saved, dma_bytes, saved_length);
    write_line("@W,old\n"); write_line("@P,pose\n"); write_line("@W,new\n");
    write_line("1,tune\n"); write_line("# NORMAL\n");
    write_line("# ROUND START 1\n");
    write_line("# CAN FEEDBACK LOST MASK=0x0F FRESH=0x0B\n");
    write_line("# ROUND STOP MOTOR FEEDBACK LOST\n");
    write_line("# POSE STOP\n");
    HostUartTx_Process(11); assert(!memcmp(saved, dma_bytes, saved_length));
    uart_done(); HostUartTx_Process(12); expect_line("# ROUND START 1\n");
    write_line("# STOP MODE=WORK\n");
    uart_done(); HostUartTx_Process(13); expect_line("# CAN FEEDBACK LOST MASK=0x0F FRESH=0x0B\n");
    uart_done(); HostUartTx_Process(14); expect_line("# ROUND STOP MOTOR FEEDBACK LOST\n");
    uart_done(); HostUartTx_Process(15); expect_line("# POSE STOP\n");
    uart_done(); HostUartTx_Process(16); expect_line("# STOP MODE=WORK\n");
    uart_done(); HostUartTx_Process(17); expect_line("# NORMAL\n");
    uart_done(); HostUartTx_Process(18); expect_line("@W,new\n");
    uart_done(); HostUartTx_Process(19); expect_line("@P,pose\n");
    uart_done(); HostUartTx_Process(20); expect_line("1,tune\n");
    /* Binary transition may discard queued text, never the active DMA buffer. */
    write_line("# STALE\n"); HostUartTx_DiscardPending(); expect_line("1,tune\n");
    HostUartTx_Process(121); assert(HostUartTx_ConsumeFault());
    uart_done(); HostUartTx_Process(122); assert(!HostUartTx_Busy());
    assert(HostUartTx_GetStats().replaced > 0);
}

static void test_uart_burst_and_errors(void)
{
    unsigned i;
    uint32_t before;
    char oversized[600];
    uart_reset();
    for (i = 0; i < 25; ++i) write_line("# HELP OR BOOT LINE\n");
    assert(!HostUartTx_ConsumeFault());
    before = dma_count;
    for (i = 0; i < 25; ++i) { HostUartTx_Process(i); uart_done(); }
    assert(dma_count - before == 25);
    for (i = 0; i < 33; ++i) write_line("# NORMAL\n");
    assert(HostUartTx_ConsumeFault()); uart_reset();
    memset(oversized, 'X', sizeof(oversized)); oversized[599] = '\n';
    HostUartTx_Write(oversized, sizeof(oversized)); assert(HostUartTx_ConsumeFault());
    write_line("# AFTER INVALID\n"); HostUartTx_Process(10); expect_line("# AFTER INVALID\n");
    uart_done(); fail_uart = 1; write_line("# FAIL\n"); HostUartTx_Process(11);
    assert(!HostUartTx_Busy() && HostUartTx_ConsumeFault()); uart_reset();
    for (i = 0; i < 5; ++i) write_line("# STOP MODE=WORK\n");
    assert(HostUartTx_ConsumeFault()); uart_reset();
    write_line("# WRAP\n"); HostUartTx_Process(UINT32_MAX - 10U);
    HostUartTx_Process(20); assert(!HostUartTx_ConsumeFault());
    HostUartTx_Process(100); assert(HostUartTx_ConsumeFault()); uart_reset();
}

static void test_motor_feedback_and_chassis(void)
{
    MotorFeedback samples[4];
    uint8_t bad[] = {0x35, 0, 0, 20, 0};
    unsigned i;
    can_reset(4900); ZDT_Emm_InitAll(); auto_complete = 1U;
    assert(Mecanum_SetRequiredMotorMask(0x01U));
    assert(Mecanum_GetRequiredMotorMask() == 0x01U);
    assert(!Mecanum_SetRequiredMotorMask(0x00U));
    assert(!Mecanum_SetRequiredMotorMask(0x10U));
    assert(Mecanum_GetRequiredMotorMask() == 0x01U);
    assert(!StopAllMotors());
    assert(sent_count == 1U && sent[0].id == 0x100U);
    can_reset(5000); ZDT_Emm_InitAll();
    assert(Mecanum_SetRequiredMotorMask(0x0FU));
    assert(ZDT_Emm_SetSpeedByID(1, 10) == 4U);
    for (i = 1; i <= 4; ++i) driver_feedback(i, 0);
    ZDT_Emm_GetFeedback(samples); assert(MotorFeedback_FreshMask(samples, tick) == 15);
    ZDT_Emm_RxHandler(0x100, bad, sizeof(bad));
    bad[4] = 0x6B; ZDT_Emm_RxHandler(0x101, bad, sizeof(bad));
    ZDT_Emm_RxHandler(0x100, bad, 4);
    ZDT_Emm_RxHandler(0x10100, bad, sizeof(bad)); /* Must not alias motor 1. */
    ZDT_Emm_GetFeedback(samples); assert(samples[0].sequence == 1);
    assert(!SetAllMotorsSpeed(1.6f, .8f, -.4f, -.8f)); near(Mecanum_GetAppliedScale(), .5f);
    ZDT_Emm_GetFeedback(samples); assert(MotorFeedback_FreshMask(samples, tick) == 15);
    (void)motors[0].actual_speed; (void)ZDT_Emm_EnableByID(1, 1U);
    ZDT_Emm_GetFeedback(samples); assert(samples[0].sequence == 1); /* Lookup must not reset feedback. */
    assert(!StopAllMotors()); assert(ZDT_CAN_StopPending());
    pending = 0; ZDT_CAN_Process(tick); pending = 0; Mecanum_ProcessFeedback(tick);
    assert(Mecanum_GetStopStatus().state == MOTOR_STOP_WAIT_FEEDBACK);
    tick += 40; for (i = 1; i <= 4; ++i) driver_feedback(i, 0);
    Mecanum_ProcessFeedback(tick); assert(Mecanum_GetStopStatus().state == MOTOR_STOP_WAIT_FEEDBACK);
    tick += 40; for (i = 1; i <= 4; ++i) driver_feedback(i, 0);
    Mecanum_ProcessFeedback(tick); assert(Mecanum_GetStopStatus().state == MOTOR_STOP_CONFIRMED);
    assert(!SetAllMotorsSpeed(.1f, .1f, .1f, .1f));
    Mecanum_ProcessFeedback(tick); assert(Mecanum_GetStopStatus().state == MOTOR_STOP_IDLE);
    tick += 301; assert(!Mecanum_FeedbackReady(15));
    assert(SetAllMotorsSpeed(.1f, .1f, .1f, .1f) == 4); near(Mecanum_GetAppliedScale(), 0);
    assert(Mecanum_ConsumeCanTxFault());
    assert(ZDT_Emm_SetSpeedByID(1, NAN) == 3);
}

extern uint8_t ops9_rx_byte;
static void ops_byte(uint8_t ch) { ops9_rx_byte = ch; OPS9_UART_RxCpltCallback(&huart2); }
static void ops_frame(float x, float y, float yaw)
{
    float values[6] = {yaw, 0, 0, x, y, 0};
    uint8_t bytes[24]; unsigned i;
    memcpy(bytes, values, sizeof(bytes));
    ops_byte(13); ops_byte(10);
    for (i = 0; i < sizeof(bytes); ++i) ops_byte(bytes[i]);
    ops_byte(10); ops_byte(13);
}
static void test_ops_snapshot_and_invalid_frame(void)
{
    OPS9_Snapshot snapshot;
    tick = 6000; ops_frame(123, -456, 90);
    primask = 1; snapshot = OPS9_GetSnapshot(); assert(primask == 1);
    primask = 0; snapshot = OPS9_GetSnapshot(); assert(primask == 0);
    near(snapshot.x_mm, 123); near(snapshot.y_mm, -456); near(snapshot.yaw_deg, 90);
    assert(snapshot.frame_count == 1 && snapshot.last_update_tick == tick);
    tick++; ops_frame(NAN, 888, 10); snapshot = OPS9_GetSnapshot();
    near(snapshot.x_mm, 123); near(snapshot.y_mm, -456);
    assert(snapshot.frame_count == 1 && snapshot.last_update_tick == 6000);
    assert(ops9_invalid_frame_count == 1);
    tick++; ops_frame(789, 222, -90); snapshot = OPS9_GetSnapshot();
    near(snapshot.x_mm, 789); near(snapshot.y_mm, 222); assert(snapshot.frame_count == 2);
    OPS9_Reset_Zero();
}

static void test_runtime_deadline(void)
{
    tick = UINT32_MAX - 10U; ControlRuntime_Init();
    ControlRuntime_Begin(20); ControlRuntime_End(21);
    assert(!ControlRuntime_GetStats().fault);
    ControlRuntime_RecordControl(20); ControlRuntime_RecordControl(35);
    assert(ControlRuntime_GetStats().control_max_ms == 35);
    assert(ControlRuntime_GetStats().control_late_count == 1);
    ControlRuntime_Begin(121); assert(ControlRuntime_GetStats().fault);
    assert(ControlRuntime_GetStats().loop_overruns == 1);
    ControlRuntime_ClearFault(); ControlRuntime_End(222);
    assert(ControlRuntime_GetStats().fault);
}

/*
 * 回归：一次历史发送故障不能让 ZDT_CAN_IsReady() 永久为假。
 * 现场表现为 TXOK/RX 持续增长、ESR/ErrorCode/TEC/REC 全为 0 的健康总线上，
 * MOTOR RUN 一直被拒绝并打印 CAN NOT READY，且刚下发的速度会被立即停车。
 */
static void test_can_fault_auto_recovery_and_clear(void)
{
    ZDT_CAN_Stats_t before, after;
    can_reset(20000U);
    ZDT_CAN_RaiseFault();
    ZDT_CAN_GetStats(&before);
    assert(ZDT_CAN_ConsumeFault());
    assert(!ZDT_CAN_ConsumeFault());
    assert(ZDT_CAN_HasFault() && !ZDT_CAN_IsReady());
    assert(ZDT_CAN_HardwareReady());
    ZDT_CAN_Process(tick);
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    tick += 100U; ZDT_CAN_Process(tick);
    tick += 100U; ZDT_CAN_Process(tick);
    assert(!ZDT_CAN_IsReady()); /* TXOK + 100ms cannot grant permission. */
    assert(!ZDT_CAN_RecoverWhenIdle(0U));
    assert(!ZDT_CAN_RecoverWhenIdle(1U));
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    rx_remaining = 1U; ZDT_CAN_RxFIFO0_Handler(&hcan1);
    tick += 499U; assert(!ZDT_CAN_RecoverWhenIdle(1U));
    ZDT_CAN_RaiseFault(); /* A new generation invalidates old progress. */
    (void)ZDT_CAN_ConsumeFault();
    tick++; assert(!ZDT_CAN_RecoverWhenIdle(1U));
    tick += 500U; assert(!ZDT_CAN_RecoverWhenIdle(1U));
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    rx_remaining = 1U; ZDT_CAN_RxFIFO0_Handler(&hcan1);
    assert(ZDT_CAN_RecoverWhenIdle(1U));
    assert(ZDT_CAN_IsReady());
    ZDT_CAN_GetStats(&after);
    assert(after.fault_generation == before.fault_generation + 1U);
    assert(after.recoveries == before.recoveries + 1U);
    assert(after.auto_recoveries == before.auto_recoveries);
    can_regs.ESR = CAN_ESR_EWGF;
    assert(!ZDT_CAN_IsReady());
    can_reset(23000U);
}

/*
 * P0-A：禁止注册电平式 EWGIE/EPVIE，否则 error passive 会中断风暴。
 * P0-B：Abort 清不掉 TME 时，FREE=0 且 TXOK 停增必须强制 Stop/Start 恢复。
 */
static void test_can_no_level_irq_and_mailbox_stall_recovery(void)
{
    uint8_t query[] = {0x35, 0x6B};
    ZDT_CAN_Stats_t before, after;
    uint32_t stops_before;

    can_reset(30000);
    ZDT_CAN_ConfigFilter();
    assert((last_notify_mask & CAN_IT_ERROR_WARNING) == 0U);
    assert((last_notify_mask & CAN_IT_ERROR_PASSIVE) == 0U);
    assert((last_notify_mask & CAN_IT_ERROR) == 0U);
    assert((last_notify_mask & CAN_IT_LAST_ERROR_CODE) == 0U);
    assert((last_notify_mask & CAN_IT_BUSOFF) == 0U);
    assert((last_notify_mask & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U);
    assert((last_notify_mask & CAN_IT_TX_MAILBOX_EMPTY) != 0U);

    /* ESR 轮询：EPVF/BOFF 时锁存发送故障，不依赖错误中断。 */
    can_regs.ESR = CAN_ESR_EPVF;
    ZDT_CAN_Process(tick);
    assert(ZDT_CAN_ConsumeFault());
    can_regs.ESR = 0U;

    /* Abort 失效 + 有发送企图但 TXOK 永不增加：150ms 后强制恢复。
     * 中途故意让 FREE 短暂弹起（模拟超时-Abort 震荡）+ BeginStop（停车重试），
     * 都不得清掉卡死计时。 */
    can_reset(31000);
    fail_abort = 1U;
    pending = 7U;
    ZDT_CAN_GetStats(&before);
    stops_before = can_stop_count;
    ZDT_CAN_Process(tick);                 /* FREE=0，开始计时 */
    tick += 40U;
    pending = 0U; ZDT_CAN_Process(tick);   /* FREE=3，不应复位 */
    pending = 7U;
    tick += 40U; ZDT_CAN_Process(tick);
    ZDT_CAN_BeginStop();
    pending = 7U;
    assert(can_stop_count == stops_before);
    tick += 70U; ZDT_CAN_Process(tick);    /* 累计 150ms */
    assert(can_stop_count == stops_before + 1U);
    assert(pending == 0U);
    ZDT_CAN_GetStats(&after);
    assert(after.stall_recoveries == before.stall_recoveries + 1U);

    /* 500ms 内再次卡死：只锁存故障，不得再 Stop/Start（防止打断总线）。 */
    pending = 7U;
    stops_before = can_stop_count;
    tick += 20U; ZDT_CAN_Process(tick);    /* 重新武装 */
    tick += 160U; ZDT_CAN_Process(tick);   /* 到阈值，但未过冷却 */
    assert(can_stop_count == stops_before);
    assert(ZDT_CAN_ConsumeFault());
    assert(pending == 7U);                 /* 冷却期内不拆邮箱 */

    /* 冷却结束后可以再次强制恢复。 */
    tick += 400U;
    ZDT_CAN_Process(tick);
    tick += 20U; ZDT_CAN_Process(tick);
    tick += 160U; ZDT_CAN_Process(tick);
    assert(can_stop_count == stops_before + 1U);
    assert(pending == 0U);
    fail_abort = 0U;
    ZDT_CAN_GetStats(&after);
    /* 首次完整恢复 + 冷却后再次恢复；冷却期内那次只锁存不计完整恢复。 */
    assert(after.stall_recoveries == before.stall_recoveries + 2U);
    assert(ZDT_CAN_ConsumeFault());
    fail_abort = 0U;

    /* FREE>0 时不得触发卡死恢复。 */
    can_reset(32000);
    pending = 0U;
    assert(!ZDT_CAN_Send_ExtId(0x100, query, 2));
    stops_before = can_stop_count;
    ZDT_CAN_GetStats(&before);
    for (tick = 32000U; tick < 32500U; tick += 50U) ZDT_CAN_Process(tick);
    assert(can_stop_count == stops_before);
    ZDT_CAN_GetStats(&after);
    assert(after.stall_recoveries == before.stall_recoveries);
}

static void test_can_recovery_preserves_stop_and_errors(void)
{
    uint8_t stop[] = {0xF6, 0, 0, 0, 0, 0, 0x6B};
    const uint32_t bus_errors[] = {CAN_ESR_EWGF, CAN_ESR_EPVF, CAN_ESR_BOFF};
    ZDT_CAN_Stats_t before, after;
    uint32_t stops_before;
    unsigned i;

    for (i = 0; i < 3U; ++i) {
        can_reset(40000U + i * 1000U);
        fail_abort = 1U; pending = 7U; can_regs.ESR = bus_errors[i];
        stops_before = can_stop_count;
        ZDT_CAN_GetStats(&before);
        ZDT_CAN_Process(tick);
        tick += 600U; ZDT_CAN_Process(tick);
        ZDT_CAN_GetStats(&after);
        assert(!ZDT_CAN_IsReady());
        assert(can_stop_count == stops_before);
        assert(after.stall_recoveries == before.stall_recoveries);
    }

    can_reset(44000U);
    pending = 7U; fail_abort = 1U;
    assert(!ZDT_CAN_SendStop(0x400, stop, sizeof(stop)));
    ZDT_CAN_Process(tick);
    tick += 150U; ZDT_CAN_Process(tick);
    assert(sent_count == 1U && sent[0].id == 0x400U);
    assert(sent[0].bytes[0] == 0xF6U && ZDT_CAN_StopPending());

    can_reset(46000U);
    pending = 7U; fail_abort = fail_can_stop = 1U;
    ZDT_CAN_GetStats(&before);
    ZDT_CAN_Process(tick);
    tick += 150U; ZDT_CAN_Process(tick);
    ZDT_CAN_GetStats(&after);
    assert(can_error & HAL_CAN_ERROR_TIMEOUT);
    assert(after.error_latched & HAL_CAN_ERROR_TIMEOUT);
    assert(after.stall_recoveries == before.stall_recoveries);
    assert(!ZDT_CAN_IsReady());
    fail_can_stop = fail_abort = 0U;
    can_reset(48000U);
}

int main(void)
{
    test_pid_dt_and_limits();
    test_stop_confirmation();
    test_can_queue_and_stop();
    test_can_timeout_enable_and_rx_budget();
    test_can_diagnostics_and_mailbox_expiry();
    test_can_stopped_recovery();
    test_can_fault_auto_recovery_and_clear();
    test_can_no_level_irq_and_mailbox_stall_recovery();
    test_can_recovery_preserves_stop_and_errors();
    test_uart_dma_ownership_and_priority();
    test_uart_burst_and_errors();
    test_motor_feedback_and_chassis();
    test_ops_snapshot_and_invalid_frame();
    test_runtime_deadline();
    puts("control layer: 14 test groups passed");
    return 0;
}
