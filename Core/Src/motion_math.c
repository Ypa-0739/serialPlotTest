#include "motion_math.h"
#include <math.h>

float Motion_AngleDeltaDeg(float current_deg, float reference_deg)
{
    float error = fmodf(current_deg - reference_deg + 180.0f, 360.0f);
    if (error < 0.0f) {
        error += 360.0f;
    }
    return error - 180.0f;
}

void Motion_OpsToCenter(float ops_x_mm, float ops_y_mm, float yaw_deg,
                                    float *center_x_mm, float *center_y_mm)
{
    float yaw_rad = yaw_deg * (3.1415926f / 180.0f);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    /*
     * OPS9 安装点位于车体几何中心前方 25 mm（车体 +Y）。
     * 先把车体偏置旋转到 OPS 全局坐标，再从传感器坐标中扣除，
     * 得到不会因原地旋转而沿圆弧移动的车体中心坐标。
     */
    *center_x_mm = ops_x_mm -
                   (cos_yaw * OPS_CENTER_OFFSET_X_MM -
                    sin_yaw * OPS_CENTER_OFFSET_Y_MM);
    *center_y_mm = ops_y_mm -
                   (sin_yaw * OPS_CENTER_OFFSET_X_MM +
                    cos_yaw * OPS_CENTER_OFFSET_Y_MM);
}

float Motion_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

float Motion_Slew(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

void Motion_SlewVector2D(float current_x, float current_y,
                              float target_x, float target_y,
                              float accel_mps2, float decel_mps2, float dt_s,
                              float *output_x, float *output_y)
{
    float current_speed = sqrtf(current_x * current_x + current_y * current_y);
    float target_projection = 0.0f;
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;
    float delta_speed = sqrtf(delta_x * delta_x + delta_y * delta_y);
    float rate = accel_mps2;
    float max_delta;

    if (current_speed > 0.0001f) {
        target_projection = (current_x * target_x + current_y * target_y) /
                            current_speed;
        /* 反向、转弯或目标在当前速度方向上的投影变小，都按减速度约束。 */
        if (target_projection < current_speed) {
            rate = decel_mps2;
        }
    }

    max_delta = rate * dt_s;
    if (delta_speed > max_delta && delta_speed > 0.0001f) {
        *output_x = current_x + delta_x * max_delta / delta_speed;
        *output_y = current_y + delta_y * max_delta / delta_speed;
    } else {
        *output_x = target_x;
        *output_y = target_y;
    }
}
