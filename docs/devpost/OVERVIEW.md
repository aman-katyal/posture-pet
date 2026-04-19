# Posture Sentinel: Hybrid Vision-IMU Edge AI
**The ultimate context-aware posture tracking platform.**

## 💡 Inspiration
Most posture trackers are limited by their perspective. Cameras lose you when you walk away; wearables can't distinguish between a "relaxed lean" and a "harmful slouch." **Posture Sentinel** solves this by fusing high-fidelity Computer Vision with a wearable 3-sensor IMU array, all processed on the cutting-edge **Arduino Uno Q**.

## 🚀 What it does
Posture Sentinel is a dual-brain system:
1.  **Visual Brain**: Uses a PC-based MediaPipe engine to track your 3D skeletal alignment. It provides a "Ground Truth" for calibration and handles tracking while you are at your desk.
2.  **Kinetic Brain**: A wearable array of three MPU-9250 sensors (Neck and Shoulders) that tracks your relative body orientation in 3D space using quaternions.
3.  **The Hub**: The **Arduino Uno Q** acts as the central coordinator. Its Qualcomm processor runs the AI inference, while its STM32 microcontroller provides immediate physical feedback via servos and haptics.

## 🌟 Key Features
*   **98.18% Accuracy**: Locally trained models using AMD ROCm hardware acceleration.
*   **Spectral Analysis**: Detects micro-tremors and muscle fatigue using FFT-based frequency analysis.
*   **Digital Twin**: A real-time 3D BabylonJS visualization of your exact spinal alignment.
*   **Intelligent Fusion**: Automatically switches between Visual and Sensor-based tracking depending on your environment.
