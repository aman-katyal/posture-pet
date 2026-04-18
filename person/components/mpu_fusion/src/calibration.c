#include "calibration.h"
#include <math.h>
#include <string.h>

#define WINDOW_SIZE 50
#define STILL_THRESHOLD 0.005f // variance in G^2

static float a_win[WINDOW_SIZE][3];
static int win_idx = 0;
static Bias gyro_bias = {0.0f, 0.0f, 0.0f};
static int still_count = 0;

void calib_init() {
    memset(a_win, 0, sizeof(a_win));
    win_idx = 0;
    gyro_bias.x = 0.0f;
    gyro_bias.y = 0.0f;
    gyro_bias.z = 0.0f;
    still_count = 0;
}

void calib_update(float ax, float ay, float az, float gx, float gy, float gz) {
    a_win[win_idx][0] = ax;
    a_win[win_idx][1] = ay;
    a_win[win_idx][2] = az;
    win_idx = (win_idx + 1) % WINDOW_SIZE;

    // Calculate Variance
    float mean[3] = {0.0f, 0.0f, 0.0f}, var[3] = {0.0f, 0.0f, 0.0f};
    for(int i = 0; i < WINDOW_SIZE; i++) {
        mean[0] += a_win[i][0];
        mean[1] += a_win[i][1];
        mean[2] += a_win[i][2];
    }
    mean[0] /= WINDOW_SIZE;
    mean[1] /= WINDOW_SIZE;
    mean[2] /= WINDOW_SIZE;

    for(int i = 0; i < WINDOW_SIZE; i++) {
        var[0] += powf(a_win[i][0] - mean[0], 2.0f);
        var[1] += powf(a_win[i][1] - mean[1], 2.0f);
        var[2] += powf(a_win[i][2] - mean[2], 2.0f);
    }
    var[0] /= WINDOW_SIZE;
    var[1] /= WINDOW_SIZE;
    var[2] /= WINDOW_SIZE;

    float total_var = var[0] + var[1] + var[2];
    if (total_var < STILL_THRESHOLD) {
        still_count++;
        if (still_count > 100) { // Still for ~1 second at 100Hz
            // Update bias with low-pass filter
            gyro_bias.x = gyro_bias.x * 0.99f + gx * 0.01f;
            gyro_bias.y = gyro_bias.y * 0.99f + gy * 0.01f;
            gyro_bias.z = gyro_bias.z * 0.99f + gz * 0.01f;
        }
    } else {
        still_count = 0;
    }
}

Bias calib_get_gyro_bias() { return gyro_bias; }
int calib_is_still() { return still_count > 100; }
