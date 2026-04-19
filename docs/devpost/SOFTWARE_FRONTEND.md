# Software & Frontend: The Digital Twin

## 🌐 The Dashboard
The system features a high-performance web-based dashboard built with **FastAPI** and **WebSockets** for sub-10ms telemetry streaming.

### 1. 3D Digital Twin (BabylonJS)
*   **Real-time Mapping**: A high-fidelity 3D human model that mirrors your exact spinal and shoulder orientation.
*   **Inverse Kinematics**: Uses the quaternion data to rotate hierarchical mesh nodes (Neck -> SpineMid -> SpineLow), visualizing "Slump" and "Lean" in real-time.
*   **Visual Cues**: The skin of the 3D model glows **Emerald Green** when posture is optimal and pulses **Deep Red** when a violation is detected.

### 2. Signal Smoothing (One-Euro Filter)
To prevent the "jitter" common in both webcam landmarks and IMU sensors, we implemented the **One-Euro Filter**. 
*   This adaptive filter provides high-speed responsiveness during fast movements while maintaining rock-solid stability during static poses.

### 3. High-Accuracy Visual Isolation
*   **Selfie Segmentation**: We integrated a **Deep Lab v3** segmentation model to isolate the user from their background. This ensures that the posture detection engine isn't confused by chair backs, pillows, or background movement.
*   **Teaching Center**: A custom UI module that allows users to "Teach" the AI their specific "Good," "Mid," and "Bad" postures, which are then used to dynamically adjust the system's sensitivity.

## 🛠 Tech Stack
*   **Backend**: Python, FastAPI, Uvicorn, OpenCV, MediaPipe.
*   **Frontend**: BabylonJS (3D Render), WebSockets, Tailwind-inspired CSS.
*   **Networking**: UDP (Vision packets), BLE (Sensor packets).
