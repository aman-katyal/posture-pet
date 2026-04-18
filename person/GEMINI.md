# Starkhacks Person Project Context

## Project Architecture & Stack
- **Hardware**: ESP32-S3
- **Sensor**: MPU-9250 (compatible with MPU-6050 driver for basic IMU data)
- **Communication**: BLE via NimBLE stack
- **Language/Framework**: C++ (Modular OOP structure), ESP-IDF v6.0

## Modular Structure
- **`main/main.cpp`**: Application entry and task orchestration.
- **`main/ble_server.cpp`**: NimBLE peripheral implementation and orientation notifications.
- **`main/imu_manager.cpp`**: I2C management and sensor fusion (Madgwick filter).
- **`components/mpu_driver`**: Patched stable driver for MPU sensors.
- **`components/mpu_fusion`**: Madgwick filter and quaternion math.

## Hardware Configuration
- **I2C Bus**: Port 0
- **SCL Pin**: GPIO 18
- **SDA Pin**: GPIO 17
- **I2C Address**: 0x68
- **DLPF**: 42Hz enabled for vibration stability.

## BLE Configuration
- **Device Name**: `ESP32S3_PERSON`
- **Service UUID**: `59462f12-9543-9999-12c8-58b459a2712d`
- **Characteristic UUID**: `33333333-2222-2222-1111-111100000000`
- **Packet Format (12 Bytes)**:
  - `[0-3]` : Roll (float32)
  - `[4-7]` : Pitch (float32)
  - `[8-11]`: Yaw (float32)
  - *Note: Little-endian byte order.*

## Key References & Conventions
- **Calibration**: 200-sample static calibration runs on boot. Keep device still.
- **Drift**: Yaw will drift over time as no magnetometer grounding is implemented in the current fusion loop.
- **Build**: Use `idf.py build` and flash over `COM7` (typical).