#ifndef CONTROL_RUNTIME_H
#define CONTROL_RUNTIME_H
#include <stdint.h>
#define CONTROL_LOOP_FAULT_MS 100U
typedef struct {
    uint32_t loop_max_ms, control_max_ms, control_late_count, loop_overruns;
    uint8_t fault, watchdog_reset;
} ControlRuntimeStats;
void ControlRuntime_Init(void);
void ControlRuntime_Begin(uint32_t now);
void ControlRuntime_End(uint32_t now);
void ControlRuntime_RecordControl(uint32_t elapsed_ms);
ControlRuntimeStats ControlRuntime_GetStats(void);
void ControlRuntime_ClearFault(void);
#endif
