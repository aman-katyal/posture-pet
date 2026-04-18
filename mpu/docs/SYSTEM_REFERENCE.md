# System Configuration & Technical Reference

This document serves as a persistent log of the **Person** project configuration, hardware setup, and performance characteristics.

## 1. Hardware Mapping

### ESP32-S3 Pinout
| Component | Bus/Interface | Pins (SCL/SDA) | I2C Address | Notes |
|-----------|---------------|----------------|-------------|-------|
| Neck MPU | I2C Bus 1 | 4 / 5 | 0x68 | Port 1 |
| Shoulder L| I2C Bus 0 | 18 / 17 | 0x68 | Port 0 |
| Shoulder R| I2C Bus 0 | 18 / 17 | 0x69 | Port 0 (AD0 High) |

### Communication
- **BLE Device Name:** `ESP32S3_PERSON`
- **Static MAC Address:** `DE:AD:BE:EF:00:01` (Set via `esp_base_mac_addr_set`)
- **BLE Service UUID:** `59462f12-9543-9999-12c8-58b459a2712d`

## 2. System Rates & Timing

### Operating Frequencies
- **I2C Clock:** 400 kHz (Fast Mode)
- **BLE Update Rate:** 20 Hz (50ms interval)
- **BLE Advertising Interval:** 100 ms (Faster discovery)
- **IMU Internal Sampling:** 100 Hz
- **DLPF (Low Pass Filter):** 42 Hz (Vibration rejection)

### Startup Sequence
1. **NVS Init:** Loads non-volatile storage.
2. **MAC Config:** Sets static BLE MAC address and prints to console.
3. **I2C Init:** Initializes Bus 0 and Bus 1.
4. **Static Calibration (2s):** Samples all active sensors for 1 second (50 samples) while the device is still to neutralize gyroscope bias.
5. **BLE Start:** Begins advertising.

### BLE Packet Structure
The orientation data is sent as a **48-byte payload** (3 sensors × 16 bytes each) containing only quaternions in **Little-Endian** format.

#### Orientation Data (Per Sensor - 16 Bytes)
| Bytes | Data Type | Field | Description |
|-------|-----------|-------|-------------|
| **0 - 3** | `float32` | **qw** | Quaternion W |
| **4 - 7** | `float32` | **qx** | Quaternion X |
| **8 - 11**| `float32` | **qy** | Quaternion Y |
| **12 - 15**| `float32` | **qz** | Quaternion Z |

#### Multi-Sensor Layout (84 Bytes Total)
1. **Neck:** Bytes 0 – 15
2. **Left Shoulder:** Bytes 16 – 31
3. **Right Shoulder:** Bytes 32 – 47

---

## 3. Sensor Fusion Logic

- **Algorithm:** Mahony Filter (Quaternion-based).
- **Adaptive Gain:**
  - **Boot (0-5s):** Kp=10.0, Ki=0.0 (Fast lock-on)
  - **Running (>5s):** Kp=1.0, Ki=0.05 (Smooth, drift-resistant tracking)
- **Gyro Correction:** Real-time subtraction of static biases calculated at boot.

## 4. Maintenance & Build
- **Toolchain:** ESP-IDF v6.0
- **Build Command:** `idf.py build`
- **Flash Command:** `idf.py -p COM7 flash monitor`
- **Baud Rate:** 115200 (Monitor), 921600 (Flash)
