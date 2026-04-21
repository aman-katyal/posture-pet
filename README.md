# Posture Pet

Posture Pet is a wearable AI companion that tracks your posture in real-time and reflects your physical state through a robotic pet. It uses a hybrid system of IMU sensors and computer vision to ensure you stay upright and healthy.

## How it works

The system is built across three main layers:

1.  **The Wearable**: An ESP32-S3 equipped with three MPU-6500 IMU sensors (placed on the neck and shoulders). It uses a Mahony filter to process raw motion data into stable quaternions.
2.  **The Hub**: An Arduino Uno Q that serves as the central brain. It runs a TensorFlow Lite inference engine on its Qualcomm Linux core to classify posture data received via BLE.
3.  **The Visual Sensor**: A PC running a MediaPipe computer vision model that cross-verifies the wearable's data via UDP over WiFi.

The robotic pet physically mimics your posture using servos and changes its OLED facial expressions based on how well you are sitting.

## Project Structure

*   `mpu/`: ESP32-S3 firmware for the wearable sensor array.
*   `uno q/`: Python inference engine and STM32 sketch for the Arduino Uno Q hub.
*   `cam/`: FastAPI backend and computer vision pipeline for the PC.
*   `training/`: Scripts and models for the posture classification AI.
*   `docs/devpost/`: Full project documentation and submission details.

## Technical Details

*   **Sensors**: 3x MPU-6500 (Neck, Left Shoulder, Right Shoulder).
*   **Communication**: BLE (Wearable to Hub), UDP over WiFi (PC to Hub).
*   **AI**: 4-layer MLP (Input -> 128 -> 64 -> 32 -> Output) running as Float16 TFLite.
*   **Filtering**: Mahony fusion for IMUs, One-Euro filter for signal smoothing.

## Submission Details

See [docs/devpost/SUBMISSION.md](docs/devpost/SUBMISSION.md) for the full hackathon story, including our inspiration, challenges, and future plans.
For 3D files, see [Onshape Files](https://cad.onshape.com/documents/3082847f6ca31ed59c7c837c/w/17ba40e810914c84b67b23d4/e/1b2842b0d1ac7e2d64288ee7?renderMode=0&uiState=69e6d445022eefb972fd2488).
