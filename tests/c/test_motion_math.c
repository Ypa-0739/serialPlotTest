#include "motion_math.h"
#include <assert.h>
#include <math.h>
static void near(float a, float b) { assert(fabsf(a - b) < .0001f); }
int main(void)
{
    float x, y;
    int yaw;
    near(Motion_AngleDeltaDeg(-179, 179), 2);
    near(Motion_AngleDeltaDeg(179, -179), -2);
    near(Motion_AngleDeltaDeg(180, 0), -180);
    near(Motion_AngleDeltaDeg(1080, 0), 0);
    /* A sensor circling a fixed chassis center must compensate to that center. */
    for (yaw = -180; yaw <= 180; yaw += 30) {
        float radians = yaw * 3.1415926f / 180;
        Motion_OpsToCenter(100 - 25 * sinf(radians), 200 + 25 * cosf(radians),
                            (float)yaw, &x, &y);
        near(x, 100); near(y, 200);
    }
    near(Motion_Clamp(2, -1, 1), 1); near(Motion_Clamp(-2, -1, 1), -1);
    near(Motion_Slew(0, 1, .1f), .1f); near(Motion_Slew(0, -1, .1f), -.1f);
    near(Motion_Slew(0, .05f, .1f), .05f);
    Motion_SlewVector2D(0, 0, 3, 4, 1, 2, .02f, &x, &y);
    near(hypotf(x, y), .02f); near(x / y, .75f);
    Motion_SlewVector2D(1, 0, -1, 0, 1, 2, .02f, &x, &y);
    near(x, .96f); near(y, 0); /* Reversal uses deceleration. */
    Motion_SlewVector2D(0, 0, .001f, .002f, 1, 2, .02f, &x, &y);
    near(x, .001f); near(y, .002f); /* Never overshoot. */
    Motion_SlewVector2D(1, 0, 0, 1, 1, 2, 0, &x, &y);
    near(x, 1); near(y, 0);
    return 0;
}
