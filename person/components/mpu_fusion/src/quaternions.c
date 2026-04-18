/********************************************************************************************
 * Project: MPU6050 ESP32 Sensor Interface
 * Author: Muhammad Idrees
 * 
 * Description:
 * This source file introduces quaternion-based calculations for determining roll, pitch, 
 * and yaw angles. Quaternions offer a robust and gimbal-lock-free method for orientation
 * estimation, enhancing the reliability of motion-sensing applications.
 * 
 * Author's Background:
 * Name: Muhammad Idrees
 * Degree: Bachelor's in Electrical and Electronics Engineering
 * Institution: Institute of Space Technology, Islamabad
 * 
 * License:
 * All code within this file is authored by Muhammad Idrees and is released for educational
 * use. It may be used and adapted freely, provided proper credit is maintained.
 * 
 * Key Features:
 * - Quaternion math for angle estimation.
 * - Supports full 360-degree yaw rotation.
 * - Accurate roll and pitch computation.
 * 
 * Date: [28/7/2024]
 ********************************************************************************************/



#include "quaternions.h"
#include <math.h>

#define BETA 0.1f // Beta coefficient for the filter

// Fast inverse square root approximation
static float fast_inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

// Wrap angle to be within -180 to 180 degrees
static float wrap_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void quaternion_init(Quaternion* q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quaternion_update(Quaternion* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    float q1 = q->w, q2 = q->x, q3 = q->y, q4 = q->z;   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx, _2q1my, _2q1mz, _2q2mx, _4q1, _4q2, _4q3, _8q2, _8q3, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3, _2q4 = 2.0f * q4, _2q1q3 = 2.0f * q1 * q3, _2q3q4 = 2.0f * q3 * q4, q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4, q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4, q3q3 = q3 * q3, q3q4 = q3 * q4, q4q4 = q4 * q4;

    // Normalize accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalize magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) {
        // Fallback to 6-axis if magnetometer is missing
        // Gradient descent algorithm corrective step
        float f1 = 2.0f * (q2 * q4 - q1 * q3) - ax;
        float f2 = 2.0f * (q1 * q2 + q3 * q4) - ay;
        float f3 = 2.0f * (0.5f - q2 * q2 - q3 * q3) - az;
        float j11or24 = _2q3;
        float j12or23 = _2q4;
        float j13or22 = _2q1;
        float j14or21 = _2q2;
        float j32 = 2.0f * j14or21;
        float j33 = 2.0f * j11or24;
          
        s1 = j13or22 * f1 + j14or21 * f2;
        s2 = j14or21 * f1 + j13or22 * f2 + j32 * f3;
        s3 = j11or24 * f1 + j12or23 * f2 + j33 * f3;
        s4 = j12or23 * f1 + j11or24 * f2;
    } else {
        norm = 1.0f / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2 * my * q2 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2 * mz * q2 + _2q3 * mz * q3 - mz * q2q2 - mz * q3q3 + mz * q4q4;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _4q3 = 4.0f * q3;
        _8q2 = 8.0f * q2;
        _8q3 = 8.0f * q3;

        // Gradient descent algorithm corrective step
        s1 = -_2q3 * (2.0f * (q2q4 - q1q3) - ax) + _2q2 * (2.0f * (q1q2 + q3q4) - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * (q2q4 - q1q3) - ax) + _2q1 * (2.0f * (q1q2 + q3q4) - ay) - _4q2 * (1.0f - 2.0f * (q2q2 + q3q3) - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _8q2 * _2bz) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * (q2q4 - q1q3) - ax) + _2q4 * (2.0f * (q1q2 + q3q4) - ay) - _4q3 * (1.0f - 2.0f * (q2q2 + q3q3) - az) + (-_4q3 * _2bx - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _8q3 * _2bz) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * (q2q4 - q1q3) - ax) + _2q3 * (2.0f * (q1q2 + q3q4) - ay) + (-_4q4 * _2bx + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    }

    norm = 1.0f / sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - BETA * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - BETA * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - BETA * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - BETA * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;
    norm = 1.0f / sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    q->w = q1 * norm;
    q->x = q2 * norm;
    q->y = q3 * norm;
    q->z = q4 * norm;
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

