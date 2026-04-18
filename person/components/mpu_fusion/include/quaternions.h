#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <math.h>

typedef struct {
    float w; float x; float y; float z;
    float ix; float iy; float iz; // Integral feedback state
} MahonyFilter;

void mahony_init(MahonyFilter* f);
void mahony_update(MahonyFilter* f, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float kp, float ki);
float mahony_get_roll(const MahonyFilter* f);
float mahony_get_pitch(const MahonyFilter* f);
float mahony_get_yaw(const MahonyFilter* f);

#endif
