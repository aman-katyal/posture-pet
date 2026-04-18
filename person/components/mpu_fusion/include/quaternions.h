#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <math.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

void quaternion_init(Quaternion* q);
void mahony_update(Quaternion* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float kp, float ki);
float quaternion_get_roll(const Quaternion* q);
float quaternion_get_pitch(const Quaternion* q);
float quaternion_get_yaw(const Quaternion* q);

#endif
