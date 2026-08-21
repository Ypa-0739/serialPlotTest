#ifndef INC_LLM_TUNER_H_
#define INC_LLM_TUNER_H_

#include "main.h"
#include "pid.h"

#define LLM_TUNE_CONTROL_PERIOD_MS   20U
#define LLM_TUNE_DT_MAX_MS           100U
#define LLM_TUNE_DURATION_MS         5000U
#define LLM_TUNE_MAX_SESSION_ROUNDS  20U
#define LLM_TUNE_TARGET_MM           200.0f
#define LLM_TUNE_TARGET_YAW_DEG      30.0f
#define LLM_TUNE_MAX_SPEED_MPS       0.15f
#define LLM_TUNE_SPEED_HARD_MAX_MPS  0.30f
#define LLM_TUNE_MAX_ACCEL_MPS2      0.20f
#define LLM_TUNE_MAX_DECEL_MPS2      0.40f
#define LLM_TUNE_YAW_MAX_RADPS       0.30f
#define LLM_TUNE_YAW_HARD_MAX_RADPS  0.80f
#define LLM_TUNE_YAW_ACCEL_RADPS2    0.50f
#define LLM_TUNE_YAW_DECEL_RADPS2    0.80f
#define LLM_TUNE_OPS_TIMEOUT_MS      300U
#define LLM_TUNE_HOST_TIMEOUT_MS     1500U
#define LLM_TUNE_POSITION_TOL_MM     5.0f
#define LLM_TUNE_YAW_TOL_DEG         1.0f
#define LLM_TUNE_KP_MAX              0.005f
#define LLM_TUNE_KI_MAX              0.00005f
#define LLM_TUNE_KD_MAX              0.002f
#define LLM_TUNE_YAW_KP_MAX          0.05f
#define LLM_TUNE_YAW_KI_MAX          0.00010f
#define LLM_TUNE_YAW_KD_MAX          0.02f

typedef enum {
    LLM_TUNE_AXIS_Y = 0,
    LLM_TUNE_AXIS_X,
    LLM_TUNE_AXIS_YAW
} LLM_TuneAxis_t;

void LLM_TunerInit(PID_Controller *pid_x,
                   PID_Controller *pid_y,
                   PID_Controller *pid_yaw);
void LLM_TunerProcess(uint32_t now);
void LLM_TunerStartRound(void);
void LLM_TunerStopRound(const char *reason);
void LLM_TunerAbort(void);
void LLM_TunerResetSession(void);

void LLM_TunerSetAxis(LLM_TuneAxis_t axis);
LLM_TuneAxis_t LLM_TunerGetAxis(void);
const char *LLM_TunerAxisName(LLM_TuneAxis_t axis);
PID_Controller *LLM_TunerGetPid(void);
PID_Controller *LLM_TunerGetPidForAxis(LLM_TuneAxis_t axis);
uint8_t LLM_TunerIsRunning(void);
uint8_t LLM_TunerGetState(void);

#endif /* INC_LLM_TUNER_H_ */
