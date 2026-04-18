# Design Spec: Hybrid Sensor Fusion Engine (Drift-Free)

**Date:** 2026-04-18  
**Topic:** Eliminating sensor drift on MPU-9250 using Mahony Filter + Adaptive Gain + Background Calibration.

## 1. Problem Statement
The current 9-axis Madgwick implementation suffers from Yaw drift due to uncompensated gyroscope bias and insufficient magnetometer correction. The user cannot guarantee stillness at startup or perform manual calibration motions.

## 2. Architecture Overview
The system will use a **Mahony Filter** as its core, enhanced by two autonomous control loops.

### 2.1 Core Filter: Mahony Implementation
The filter will utilize Proportional (Kp) and Integral (Ki) feedback to correct the orientation estimated by integrating gyroscope data.
*   **Kp (Proportional Gain):** Corrects current orientation errors based on Accel/Mag vectors.
*   **Ki (Integral Gain):** Accumulates errors over time to estimate and subtract gyroscope bias (eliminating drift).

### 2.2 Adaptive Gain Controller
To solve the "slow lock" problem, the filter will maintain a `convergence_factor`:
*   **Fast Lock Phase (T < 10s):** `Kp = 10.0`, `Ki = 0.0`. High gain forces the quaternion to align with gravity and the magnetic field instantly.
*   **Stability Phase (T > 10s):** `Kp = 0.5`, `Ki = 0.1`. Low gain for smoothness, with integral action enabled to "eat" residual gyro drift.

### 2.3 Background Stillness Detection (Auto-Zero)
A software module will monitor the variance of raw accelerometer data over a sliding window (50 samples).
*   **Threshold:** If `variance < 0.01g` for 2 continuous seconds.
*   **Action:** Recalculate `gyro_bias` by averaging the current raw gyro readings.
*   **Impact:** Constantly "re-zeros" the sensor whenever it is placed on a table.

## 3. Data Flow
1.  **Read Raw Data:** Fetch Accel, Gyro, Mag from `MPUdriver`.
2.  **Preprocessing:**
    *   Subtract current `gyro_bias`.
    *   Apply axis mapping (Accel X, Y, Z -> Mag Y, X, -Z).
3.  **Stillness Monitor:** Check variance. If still, update `gyro_bias`.
4.  **Update Filter:** Execute Mahony update with current adaptive Kp/Ki.
5.  **Output:** Convert Quaternion to Euler angles (Roll, Pitch, Yaw).

## 4. Components
*   `components/mpu_fusion`:
    *   `quaternions.h/c`: Replace Madgwick logic with Mahony math.
    *   `calibration.h/c`: New module for stillness detection and bias management.
*   `main/imu_manager.cpp`: Update to orchestrate the new flow.

## 5. Success Criteria
*   **Zero Yaw Drift:** Yaw remains stable within ±1 degree over 5 minutes while stationary.
*   **Instant Lock:** Orientation is accurate within 2 seconds of power-on.
*   **No Manual Intervention:** Works immediately regardless of startup motion.
