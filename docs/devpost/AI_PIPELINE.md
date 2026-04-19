# AI & Signal Processing: Edge Intelligence

## 🧠 The Inference Model
We built a custom **Dense Neural Network (MLP)** designed specifically for edge deployment on the Qualcomm processor.

*   **Architecture**: 4-Layer MLP (Input -> 128 -> 64 -> 32 -> Output).
*   **Optimization**: Applied **Dropout (0.3)** and **L2 Regularization** to ensure the model generalizes across different body types.
*   **Quantization**: Model converted to **Float16 TFLite** to maximize throughput on the Qualcomm ARM core.

## 📊 Feature Engineering (The 60-Feature Engine)
Rather than feeding raw data, we implemented a sophisticated real-time signal processing engine:
1.  **Temporal Statistics**: Mean, StdDev, Min, and Max over a 200-sample window.
2.  **Spectral Energy (FFT)**: We compute the spectral power of each axis. This allows the AI to distinguish between "static good posture" and the "trembling jitter" of muscles reaching the point of failure.
3.  **Relative Quaternions**: By computing orientations relative to the Neck MPU, the model is immune to the user's absolute facing direction in the room.

## 🛠 Training Pipeline
*   **Hardware**: Trained locally on an **AMD Ryzen 9 7940HS** using **ROCm (Radeon Open Compute)** acceleration.
*   **Data Augmentation**: To improve robustness, we implemented a synthetic jitter generator that simulates sensor noise, doubling the training dataset to **110 samples**.
*   **Accuracy**: Achieved **98.18% cross-validation accuracy** (Average of 5-fold CV).
