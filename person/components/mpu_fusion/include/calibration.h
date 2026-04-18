#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

typedef struct {
    float x; float y; float z;
} Bias;

void calib_init();
void calib_update(float ax, float ay, float az, float gx, float gy, float gz);
Bias calib_get_gyro_bias();
int calib_is_still();

#endif
