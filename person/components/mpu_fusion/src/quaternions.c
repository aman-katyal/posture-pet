#include "quaternions.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// Wrap angle to be within -180 to 180 degrees
static float wrap_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void quaternion_init(Quaternion* q) {
    q->w = 1.0f; q->x = 0.0f; q->y = 0.0f; q->z = 0.0f;
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
}

void mahony_update(Quaternion* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float kp, float ki) {
    float q1 = q->w, q2 = q->x, q3 = q->y, q4 = q->z;
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
        integralFBx += ex * ki * dt;
        integralFBy += ey * ki * dt;
        integralFBz += ez * ki * dt;
    } else {
        integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    }

    // Apply feedback terms
    gx += kp * ex + integralFBx;
    gy += kp * ey + integralFBy;
    gz += kp * ez + integralFBz;

    // Integrate rate of change of quaternion
    float pa = q2, pb = q3, pc = q4;
    q1 += (-pa * gx - pb * gy - pc * gz) * (0.5f * dt);
    q2 += (q1 * gx + pb * gz - pc * gy) * (0.5f * dt);
    q3 += (q1 * gy - pa * gz + pc * gx) * (0.5f * dt);
    q4 += (q1 * gz + pa * gy - pb * gx) * (0.5f * dt);

    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q->w = q1 / norm; q->x = q2 / norm; q->y = q3 / norm; q->z = q4 / norm;
}

float quaternion_get_roll(const Quaternion* q) {
    float roll = atan2f(2.0f * (q->w * q->x + q->y * q->z), 1.0f - 2.0f * (q->x * q->x + q->y * q->y)) * 180.0f / M_PI;
    return wrap_angle(roll);
}

float quaternion_get_pitch(const Quaternion* q) {
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabs(sinp) >= 1.0f) sinp = copysignf(1.0f, sinp);
    float pitch = asinf(sinp) * 180.0f / M_PI;
    return wrap_angle(pitch);
}

float quaternion_get_yaw(const Quaternion* q) {
    float yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * 180.0f / M_PI;
    return wrap_angle(yaw);
}
