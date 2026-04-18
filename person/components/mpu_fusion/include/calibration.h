#ifdef __cplusplus
extern "C" {
#endif
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

typedef struct {
    float x; float y; float z;
} Bias;

void calib_init();
void calib_update(float ax, float ay, float az, float gx, float gy, float gz);
void calib_update_mag(float mx, float my, float mz);
Bias calib_get_gyro_bias();
Bias calib_get_mag_offset();
int calib_is_still();

#endif

#ifdef __cplusplus
}
#endif
