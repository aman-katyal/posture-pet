#include "quaternions.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void mahony_init(MahonyFilter* f) {
    f->w = 1.0f; f->x = 0.0f; f->y = 0.0f; f->z = 0.0f;
    f->ix = 0.0f; f->iy = 0.0f; f->iz = 0.0f;
}

// Wrap angle to be within -180 to 180 degrees
static float wrap_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void mahony_update(MahonyFilter* f, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float kp, float ki) {
    float q1 = f->w, q2 = f->x, q3 = f->y, q4 = f->z;
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) {
        // 6-axis fallback if mag is zero
        vx = 2.0f * (q2 * q4 - q1 * q3);
        vy = 2.0f * (q1 * q2 + q3 * q4);
        vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);
    } else {
        norm = 1.0f / norm;
        mx *= norm; my *= norm; mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = mx * (q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4) + my * (2.0f * (q2 * q3 - q1 * q4)) + mz * (2.0f * (q2 * q4 + q1 * q3));
        hy = mx * (2.0f * (q2 * q3 + q1 * q4)) + my * (q1 * q1 - q2 * q2 + q3 * q3 - q4 * q4) + mz * (2.0f * (q3 * q4 - q1 * q2));
        bx = sqrtf(hx * hx + hy * hy);
        bz = mx * (2.0f * (q2 * q4 - q1 * q3)) + my * (2.0f * (q3 * q4 + q1 * q2)) + mz * (q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);

        // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q2 * q4 - q1 * q3);
        vy = 2.0f * (q1 * q2 + q3 * q4);
        vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;
        wx = bx * (q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4) + bz * (2.0f * (q2 * q4 - q1 * q3));
        wy = bx * (2.0f * (q2 * q3 - q1 * q4)) + bz * (2.0f * (q1 * q2 + q3 * q4));
        wz = bx * (2.0f * (q1 * q3 + q2 * q4)) + bz * (q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);

        // Error is cross product between estimated and measured direction of vectors
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    }

    if (ki > 0.0f) {
        f->ix += ex * ki * dt;
        f->iy += ey * ki * dt;
        f->iz += ez * ki * dt;
    }

    // Apply feedback terms
    gx += kp * ex + f->ix;
    gy += kp * ey + f->iy;
    gz += kp * ez + f->iz;

    // Integrate rate of change of quaternion
    float dq1 = (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * dt);
    float dq2 = (q1 * gx + q3 * gz - q4 * gy) * (0.5f * dt);
    float dq3 = (q1 * gy - q2 * gz + q4 * gx) * (0.5f * dt);
    float dq4 = (q1 * gz + q2 * gy - q3 * gx) * (0.5f * dt);
    f->w += dq1; f->x += dq2; f->y += dq3; f->z += dq4;

    // Normalise quaternion
    norm = sqrtf(f->w * f->w + f->x * f->x + f->y * f->y + f->z * f->z);
    f->w /= norm; f->x /= norm; f->y /= norm; f->z /= norm;
}

float mahony_get_roll(const MahonyFilter* f) {
    float roll = atan2f(2.0f * (f->w * f->x + f->y * f->z), 1.0f - 2.0f * (f->x * f->x + f->y * f->y)) * 180.0f / M_PI;
    return wrap_angle(roll);
}

float mahony_get_pitch(const MahonyFilter* f) {
    float sinp = 2.0f * (f->w * f->y - f->z * f->x);
    if (fabs(sinp) >= 1.0f) sinp = copysignf(1.0f, sinp);
    float pitch = asinf(sinp) * 180.0f / M_PI;
    return wrap_angle(pitch);
}

float mahony_get_yaw(const MahonyFilter* f) {
    float yaw = atan2f(2.0f * (f->w * f->z + f->x * f->y), 1.0f - 2.0f * (f->y * f->y + f->z * f->z)) * 180.0f / M_PI;
    return wrap_angle(yaw);
}
