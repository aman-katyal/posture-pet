# Starkhacks Person Project Context

## Project Architecture & Stack
- **Hardware**: ESP32-S3
- **Sensor**: MPU-6500 (High-performance 6-axis IMU)
- **Communication**: BLE via NimBLE stack
- **Language/Framework**: C++ (Modular OOP structure), ESP-IDF v6.0

## Modular Structure
- **`main/main.cpp`**: Application entry and task orchestration.
- **`main/ble_server.cpp`**: NimBLE peripheral implementation and orientation notifications.
- **`main/imu_manager.cpp`**: I2C management and sensor fusion (Mahony filter).
- **`components/mpu_driver`**: Patched stable driver for MPU sensors.
- **`components/mpu_fusion`**: Mahony filter and quaternion math.

## Hardware Configuration
- **I2C Bus 0**: SCL=18, SDA=17
  - 0x68: Left Shoulder
  - 0x69: Right Shoulder
- **I2C Bus 1**: SCL=4, SDA=5
  - 0x68: Neck

## BLE Configuration
- **Device Name**: `ESP32S3_PERSON`
- **Service UUID**: `59462f12-9543-9999-12c8-58b459a2712d`
- **Characteristic UUID**: `33333333-2222-2222-1111-111100000000`
- **Packet Format (48 Bytes Total)**:
  - 3 Sensors × 16 Bytes each (Quaternions)
  - Each sensor: `[0-3]` qw, `[4-7]` qx, `[8-11]` qy, `[12-15]` qz (float32)
  - Order: Neck, L Shoulder, R Shoulder
  - *Note: Little-endian byte order.*

## Key References & Conventions
- **Calibration**: 50-sample static calibration runs on boot. Keep device still.
- **Drift**: Yaw will drift over time as no magnetometer grounding is implemented in the current fusion loop.
- **Build**: Use `idf.py build` and flash over `COM7` (typical).

## Documentation
- See [docs/SYSTEM_REFERENCE.md](docs/SYSTEM_REFERENCE.md) for full hardware pinouts, system rates, and calibration logic.
