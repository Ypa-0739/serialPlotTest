#ifndef MOTION_MATH_H
#define MOTION_MATH_H

/* POSE 与 TUNE 共用的安装偏移，单位 mm；不依赖传感器或 HAL。 */
#define OPS_CENTER_OFFSET_X_MM 0.0f
#define OPS_CENTER_OFFSET_Y_MM 25.0f
/* 返回 angle - reference，归一化到 [-180, 180)。 */
float Motion_AngleDeltaDeg(float angle_deg, float reference_deg);
void Motion_OpsToCenter(float ops_x_mm, float ops_y_mm, float yaw_deg,
                        float *center_x_mm, float *center_y_mm);
float Motion_Clamp(float value, float min_value, float max_value);
float Motion_Slew(float current, float target, float max_step);
/* 输入必须为有限值，accel/decel/dt 非负；调用者负责硬限幅与无效输入停车。 */
void Motion_SlewVector2D(float current_x, float current_y, float target_x, float target_y,
                         float accel_mps2, float decel_mps2, float dt_s,
                         float *output_x, float *output_y);
#endif
