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
4. **Static Calibration (1s):** Samples all active sensors for 1 second (50 samples) while the device is still to neutralize gyroscope bias.
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

#### Multi-Sensor Layout (48 Bytes Total)
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

## 5. System Flows

### Code Flow
1. **Entry Point (`app_main`):** Initializes system components (NVS, MAC address) and instantiates `BLEServer` and `IMUManager`.
2. **IMU Initialization:** `IMUManager::init()` sets up dual I2C buses, detects active sensors, performs a 1-second static calibration to calculate gyro bias, and initializes the Mahony filters for each sensor.
3. **BLE Setup:** `BLEServer::init()` configures the NimBLE stack, defines the orientation service/characteristic, and starts advertising.
4. **Execution Loop:** A dedicated task runs at 20Hz:
    - Calls `IMUManager::update()` to acquire raw data and perform sensor fusion.
    - Updates Mahony state with calculated `dt`.
    - Returns a `MultiOrientation` struct containing quaternions.
    - Calls `BLEServer::notify()` to serialize and transmit the data.

### Processing Flow (Data Pipeline)
1. **Data Acquisition:** Raw 16-bit Accelerometer and Gyroscope values are read from the MPU-6500 sensors over I2C.
2. **Signal Conditioning:**
    - **Gyroscope:** Converted to $rad/s$; static bias (calculated at boot) is subtracted.
    - **Accelerometer:** Converted to $g$'s; normalized to a unit vector within the fusion algorithm.
3. **Sensor Fusion (Mahony Algorithm):**
    - **Error Estimation:** Calculates the cross product between the estimated gravity vector (derived from the current quaternion) and the measured gravity vector (from the accelerometer).
    - **PI Feedback:** A Proportional-Integral controller uses this error to "drift correct" the gyroscope rates.
    - **Integration:** The corrected rates are used to calculate the quaternion derivative, which is integrated over the sample period `dt`.
    - **Normalization:** The resulting quaternion is re-normalized to maintain its unit length.
4. **Serialization:** The quaternions ($q_w, q_x, q_y, q_z$) for each active sensor are packed into a 48-byte binary buffer (Little-Endian).
5. **BLE Transmission:** The buffer is sent as a GATT notification to the connected client.
