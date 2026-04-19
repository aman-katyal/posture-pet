# Hardware Architecture: The Multi-Processor Hub

## 🛠️ The Stack
The project spans three distinct hardware layers, communicating across BLE, UDP/WiFi, and internal Serial Bridges.

### 1. The Wearable (ESP32-S3)
*   **Sensors**: 3x MPU-9250 IMUs (Neck, Left Shoulder, Right Shoulder).
*   **Sensor Fusion**: Implements a high-speed **Mahony Filter** at the edge to convert raw gyro/accel data into stable **Quaternions**.
*   **Optimization**: A 42Hz Digital Low Pass Filter (DLPF) is applied at the register level to eliminate environmental vibration noise.
*   **Communication**: Pushes a 48-byte custom binary payload (3x Quaternions) via **NimBLE** notifications at 20Hz.

### 2. The Hub (Arduino Uno Q)
The Uno Q serves as the "System Brain" using its unique dual-architecture:
*   **Qualcomm QRB2210 (Linux)**:
    *   Runs the primary **TFLite Inference Engine** for MPU data.
    *   Manages the BLE connection to the wearable.
    *   Hosts a UDP server to coordinate with the PC.
*   **STM32U585 (MCU)**:
    *   Handles real-time hardware IO and physical feedback.
    *   Controls servos for postural alerts and manages the LED indicator matrix.

### 3. The Visual Sensor (PC + Webcam)
*   Runs a high-fidelity **MediaPipe CV model** to track skeletal alignment.
*   Acts as a "Cross-Verification" source to validate IMU-based AI predictions.

## 🔗 Protocol Map
| Link | Protocol | Data Type |
| :--- | :--- | :--- |
| **ESP32 -> Uno Q** | BLE (NimBLE) | 12x Float32 (3x Quaternions) |
| **PC -> Uno Q** | UDP (WiFi) | CV Classification + Metrics |
| **Qualcomm -> STM32** | RouterBridge | Custom RPC (set_posture) |
