#include "control_runtime.h"
#ifdef CONTROL_HOST_TEST
#include "control_test_hal.h"
#else
#include "main.h"
#endif
static ControlRuntimeStats stats;
static uint32_t loop_start, previous_start;
static uint8_t initialized;

void ControlRuntime_Init(void)
{
#ifndef CONTROL_HOST_TEST
    uint32_t start;
    stats.watchdog_reset = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? 1U : 0U;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    /* RM0090: LSI 独立时钟，256 分频、250 计数，名义约 2 秒。
     * 仅在上电等待初始化完成后启动；不要在中断或延时函数内喂狗。
     * MCU 复位不等于驱动器已停，驱动器通信超时仍须另行配置。 */
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 6U;
    IWDG->RLR = 249U;
    start = HAL_GetTick();
    while (IWDG->SR != 0U) {
        if ((uint32_t)(HAL_GetTick() - start) > 100U) Error_Handler();
    }
    IWDG->KR = 0xAAAAU;
#endif
    previous_start = HAL_GetTick();
    initialized = 1U;
}
void ControlRuntime_Begin(uint32_t now)
{
    uint32_t elapsed = now - previous_start;
    if (initialized) {
        if (elapsed > stats.loop_max_ms) stats.loop_max_ms = elapsed;
        if (elapsed > CONTROL_LOOP_FAULT_MS) { stats.fault = 1U; stats.loop_overruns++; }
    }
    previous_start = loop_start = now;
}
void ControlRuntime_End(uint32_t now)
{
    if ((uint32_t)(now - loop_start) > CONTROL_LOOP_FAULT_MS) {
        stats.fault = 1U;
        return;
    }
#ifndef CONTROL_HOST_TEST
    IWDG->KR = 0xAAAAU;
#endif
}
void ControlRuntime_RecordControl(uint32_t elapsed_ms)
{
    if (elapsed_ms > stats.control_max_ms) stats.control_max_ms = elapsed_ms;
    if (elapsed_ms > 25U) stats.control_late_count++;
}
ControlRuntimeStats ControlRuntime_GetStats(void) { return stats; }
void ControlRuntime_ClearFault(void) { stats.fault = 0U; }
