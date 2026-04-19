# Hardware Architecture: The Multi-Processor Hub

## 🛠 The Stack
The project spans three distinct hardware layers, communicating across BLE, UDP/WiFi, and internal Serial Bridges.

### 1. The Wearable (ESP32-S3)
*   **Sensors**: 3x MPU-9250 IMUs.
*   **Sensor Fusion**: Implements a high-speed **Madgwick Filter** at the edge to convert raw gyro/accel data into stable **Quaternions**.
*   **Optimization**: A 42Hz Digital Low Pass Filter (DLPF) is applied at the register level to eliminate environmental vibration noise.
*   **Communication**: Pushes a 48-byte custom binary payload (3x Quaternions) via **NimBLE** notifications at 50Hz.

### 2. The Hub (Arduino Uno Q)
The Uno Q serves as the "System Brain" using its unique dual-architecture:
*   **Qualcomm QRB2210 (Linux)**:
    *   Runs the primary **TFLite Inference Engine**.
    *   Manages the BLE connection to the wearable.
    *   Hosts a UDP server on Port 9999 to receive visual data from the PC.
*   **STM32U585 (MCU)**:
    *   Handles the real-time hardware IO.
    *   Receives commands from the Qualcomm side via the **Arduino_RouterBridge**.
    *   Controls servos for physical feedback and manages the LED indicator matrix.

### 3. The Visual Sensor (PC + Webcam)
*   Captures high-fidelity skeletal data to serve as the "Ground Truth" for the wearable's calibration.
*   Uses skeletal geometry to calculate "Slump," "Lean," and "Head Pitch" metrics.

## 🔗 Protocol Map
| Link | Protocol | Data Type |
| :--- | :--- | :--- |
| **ESP32 -> Uno Q** | BLE (NimBLE) | 12x Float32 (Quaternions) |
| **PC -> Uno Q** | UDP (WiFi) | JSON Classification + Confidence |
| **Qualcomm -> STM32** | RouterBridge | Custom RPC (set_posture) |
