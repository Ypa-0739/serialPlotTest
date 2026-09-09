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

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1, huart2 = {USART2, HAL_UART_STATE_READY};
static uint32_t tick, primask, pending, sent_count, abort_count, rx_remaining;
static uint8_t auto_complete, fail_can, fail_uart;
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
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t n)
{ (void)h; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mask)
{ (void)h; pending &= ~mask; abort_count++; return HAL_OK; }
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
    tick = now; ZDT_CAN_BeginStop(); (void)ZDT_CAN_ConsumeFault();
    sent_count = 0U; fail_can = auto_complete = 0U;
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
    write_line("# POSE STOP\n"); write_line("# STOP MODE=WORK\n");
    HostUartTx_Process(11); assert(!memcmp(saved, dma_bytes, saved_length));
    uart_done(); HostUartTx_Process(12); expect_line("# POSE STOP\n");
    uart_done(); HostUartTx_Process(13); expect_line("# STOP MODE=WORK\n");
    uart_done(); HostUartTx_Process(14); expect_line("# NORMAL\n");
    uart_done(); HostUartTx_Process(15); expect_line("@W,new\n");
    uart_done(); HostUartTx_Process(16); expect_line("@P,pose\n");
    uart_done(); HostUartTx_Process(17); expect_line("1,tune\n");
    /* Binary transition may discard queued text, never the active DMA buffer. */
    write_line("# STALE\n"); HostUartTx_DiscardPending(); expect_line("1,tune\n");
    HostUartTx_Process(118); assert(HostUartTx_ConsumeFault());
    uart_done(); HostUartTx_Process(119); assert(!HostUartTx_Busy());
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
    can_reset(5000); ZDT_Emm_InitAll();
    assert(ZDT_Emm_SetSpeedByID(1, 10) == 4U);
    for (i = 1; i <= 4; ++i) driver_feedback(i, 0);
    ZDT_Emm_GetFeedback(samples); assert(MotorFeedback_FreshMask(samples, tick) == 15);
    ZDT_Emm_RxHandler(0x100, bad, sizeof(bad));
    bad[4] = 0x6B; ZDT_Emm_RxHandler(0x101, bad, sizeof(bad));
    ZDT_Emm_RxHandler(0x100, bad, 4);
    ZDT_Emm_GetFeedback(samples); assert(samples[0].sequence == 1);
    assert(!SetAllMotorsSpeed(1.6f, .8f, -.4f, -.8f)); near(Mecanum_GetAppliedScale(), .5f);
    ZDT_Emm_GetFeedback(samples); assert(MotorFeedback_FreshMask(samples, tick) == 15);
    (void)ZDT_Emm_GetSingleMotorSpeed(1); (void)ZDT_Emm_EnableByID(1);
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

int main(void)
{
    test_pid_dt_and_limits();
    test_stop_confirmation();
    test_can_queue_and_stop();
    test_can_timeout_enable_and_rx_budget();
    test_uart_dma_ownership_and_priority();
    test_uart_burst_and_errors();
    test_motor_feedback_and_chassis();
    test_ops_snapshot_and_invalid_frame();
    test_runtime_deadline();
    puts("control layer: 9 test groups passed");
    return 0;
}
