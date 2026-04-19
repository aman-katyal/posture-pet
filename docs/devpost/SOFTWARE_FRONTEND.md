# Software and Frontend: The Digital Twin

## Web Dashboard
The system includes a high-performance web dashboard developed with FastAPI. It uses WebSockets to stream telemetry data with minimal latency.

### 1. 3D Visualization
*   **Real-time Mapping**: The dashboard features a 3D model that reflects the user's spinal and shoulder orientation.
*   **Hierarchical Animation**: We use quaternion data to rotate model nodes (Neck, Mid Spine, and Lower Spine), providing a clear visual representation of the user's posture.
*   **Visual Feedback**: The model changes color based on the current posture state - turning green when aligned and red when a violation is detected.

### 2. Signal Filtering
We implemented the One-Euro Filter to smooth data from both the IMU sensors and the webcam landmarks. This adaptive filter provides responsiveness during movement while maintaining stability when the user is still.

### 3. Image Segmentation
To ensure the vision model focuses only on the user, we integrated a selfie segmentation model. This prevents the system from being distracted by the background or objects in the room.

## Technology Stack
*   **Backend**: Python, FastAPI, Uvicorn, OpenCV, MediaPipe.
*   **Frontend**: BabylonJS for 3D rendering, WebSockets, and CSS.
*   **Networking**: UDP for vision data and BLE for sensor data.
