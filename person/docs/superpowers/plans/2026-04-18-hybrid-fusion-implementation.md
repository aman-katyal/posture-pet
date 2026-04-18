# Hybrid Sensor Fusion Engine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a drift-free 9-axis sensor fusion system using a Mahony Filter, Adaptive Gain, and Background Stillness Detection for the MPU-9250.

**Architecture:**
1.  **Mahony Filter:** Replaces Madgwick with an integral-feedback loop to eliminate gyro drift.
2.  **Stillness Monitor:** Continuously monitors accelerometer variance to "auto-zero" gyro bias.
3.  **Adaptive Gain:** High gain at startup (10s) for instant lock, switching to precision gain for stability.

**Tech Stack:** C/C++, ESP-IDF v6.0, I2C Master Driver.

---

### Task 1: Mahony Filter Core Implementation

**Files:**
- Modify: `components/mpu_fusion/include/quaternions.h`
- Modify: `components/mpu_fusion/src/quaternions.c`

- [ ] **Step 1: Update Header with Mahony state and Ki/Kp**

```c
// components/mpu_fusion/include/quaternions.h
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
void mahony_init(MahonyFilter* f);
void mahony_update(MahonyFilter* f, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, float kp, float ki);
float mahony_get_roll(const MahonyFilter* f);
float mahony_get_pitch(const MahonyFilter* f);
float mahony_get_yaw(const MahonyFilter* f);

#endif
```

- [ ] **Step 2: Implement Mahony Math in Source**

```c
// components/mpu_fusion/src/quaternions.c
#include "quaternions.h"

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

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
```

- [ ] **Step 3: Commit**

```bash
git add components/mpu_fusion/
git commit -m "feat(fusion): implement Mahony filter core"
```

---

### Task 2: Stillness Detection and Bias Management

**Files:**
- Create: `components/mpu_fusion/include/calibration.h`
- Create: `components/mpu_fusion/src/calibration.c`

- [ ] **Step 1: Create Calibration Header**

```c
// components/mpu_fusion/include/calibration.h
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
```

- [ ] **Step 2: Implement Stillness Logic**

```c
// components/mpu_fusion/src/calibration.c
#include "calibration.h"
#include <math.h>
#include <string.h>

#define WINDOW_SIZE 50
#define STILL_THRESHOLD 0.005f // variance in G^2

static float a_win[WINDOW_SIZE][3];
static int win_idx = 0;
static Bias gyro_bias = {0,0,0};
static int still_count = 0;

void calib_init() {
    memset(a_win, 0, sizeof(a_win));
    win_idx = 0;
}

void calib_update(float ax, float ay, float az, float gx, float gy, float gz) {
    a_win[win_idx][0] = ax; a_win[win_idx][1] = ay; a_win[win_idx][2] = az;
    win_idx = (win_idx + 1) % WINDOW_SIZE;

    // Calculate Variance
    float mean[3] = {0}, var[3] = {0};
    for(int i=0; i<WINDOW_SIZE; i++) {
        mean[0] += a_win[i][0]; mean[1] += a_win[i][1]; mean[2] += a_win[i][2];
    }
    mean[0] /= WINDOW_SIZE; mean[1] /= WINDOW_SIZE; mean[2] /= WINDOW_SIZE;

    for(int i=0; i<WINDOW_SIZE; i++) {
        var[0] += powf(a_win[i][0] - mean[0], 2);
        var[1] += powf(a_win[i][1] - mean[1], 2);
        var[2] += powf(a_win[i][2] - mean[2], 2);
    }
    var[0] /= WINDOW_SIZE; var[1] /= WINDOW_SIZE; var[2] /= WINDOW_SIZE;

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
```

- [ ] **Step 3: Commit**

```bash
git add components/mpu_fusion/
git commit -m "feat(fusion): add background stillness detection"
```

---

### Task 3: Integrating Hybrid Engine in IMUManager

**Files:**
- Modify: `main/imu_manager.cpp`
- Modify: `main/include/imu_manager.hpp`

- [ ] **Step 1: Update IMUManager Header**

```cpp
// main/include/imu_manager.hpp
#include "calibration.h" // Add this
// ...
private:
    IMUManager() : mpu(i2c0) {}
    MPU_t mpu;
    Quaternion qState;
    int64_t lastTime = 0;
    int64_t startTime = 0; // Add this for adaptive gain
```

- [ ] **Step 2: Update Update Loop with Adaptive Gain and Bias Correction**

```cpp
// main/imu_manager.cpp
#include "calibration.h"

void IMUManager::init() {
    // ... same init as before ...
    startTime = esp_timer_get_time();
    calib_init();
    quaternion_init(&qState);
}

Orientation IMUManager::update() {
    mpud::raw_axes_t accelRaw, gyroRaw, magRaw;
    Orientation o = {0, 0, 0};
    
    if (mpu.motion(&accelRaw, &gyroRaw, &magRaw) == ESP_OK) {
        mpud::float_axes_t accel = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        mpud::float_axes_t gyro_raw = mpud::gyroRadPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        
        // Background calibration update
        calib_update(accel.x, accel.y, accel.z, gyro_raw.x, gyro_raw.y, gyro_raw.z);
        Bias bias = calib_get_gyro_bias();
        
        // Correct gyro
        float gx = gyro_raw.x - bias.x;
        float gy = gyro_raw.y - bias.y;
        float gz = gyro_raw.z - bias.z;

        float mx = (float)magRaw.y; float my = (float)magRaw.x; float mz = -(float)magRaw.z;

        int64_t now = esp_timer_get_time();
        float dt = (float)(now - lastTime) / 1000000.0f;
        float elapsed = (float)(now - startTime) / 1000000.0f;
        lastTime = now;

        // Adaptive Gain
        float kp = (elapsed < 10.0f) ? 10.0f : 0.5f;
        float ki = (elapsed < 10.0f) ? 0.0f : 0.1f;

        mahony_update(&qState, gx, gy, gz, accel.x, accel.y, accel.z, mx, my, mz, dt, kp, ki);
        
        o.roll = quaternion_get_roll(&qState);
        o.pitch = quaternion_get_pitch(&qState);
        o.yaw = quaternion_get_yaw(&qState);
    }
    return o;
}
```

- [ ] **Step 3: Commit**

```bash
git add main/
git commit -m "feat(imu): integrate hybrid mahony engine with adaptive gain"
```
