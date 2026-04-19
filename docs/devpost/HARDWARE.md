# Project Hardware: Multi-Processor Hub

## Hardware Architecture
The project spans three distinct hardware layers, communicating across BLE, UDP over WiFi, and internal Serial Bridges.

### 1. The Wearable (ESP32-S3)
*   **Sensors**: 3x MPU-6500 IMUs (Neck, Left Shoulder, Right Shoulder).
*   **Sensor Fusion**: Implements a high-speed Mahony Filter at the edge to convert raw gyro and accelerometer data into stable Quaternions.
*   **Optimization**: A 42Hz Digital Low Pass Filter (DLPF) is applied at the register level to eliminate environmental vibration noise.
*   **Communication**: Transmits a 48-byte custom binary payload containing three quaternions via NimBLE notifications at 20Hz.

### 2. The Hub (Arduino Uno Q)
The Uno Q serves as the primary system brain, utilizing its dual-architecture design.
*   **Qualcomm QRB2210 (Linux)**:
    *   Runs the primary TensorFlow Lite inference engine for processing wearable sensor data.
    *   Manages the BLE connection to the ESP32-S3.
    *   Hosts a UDP server to receive and process visual classification data from the PC.
*   **STM32U585 (MCU)**:
    *   Handles real-time hardware IO and low-level physical feedback.
    *   Controls the servos that move the physical robot and manages the OLED display used for the pet's facial expressions.

### 3. The Visual Sensor (PC + Webcam)
*   Runs a MediaPipe-based computer vision model to track 3D skeletal alignment.
*   Sends classification data to the Uno Q via UDP over WiFi to cross-verify the wearable's AI predictions.

## Protocol Map
| Link | Protocol | Data Type |
| :--- | :--- | :--- |
| **ESP32 -> Uno Q** | BLE (NimBLE) | 12x Float32 (3x Quaternions) |
| **PC -> Uno Q** | UDP (WiFi) | CV Classification + Confidence |
| **Qualcomm -> STM32** | RouterBridge | Custom RPC Commands (set_posture) |
