# AI Pipeline: Edge Intelligence and Signal Processing

## Machine Learning Model
We developed a custom Dense Neural Network (MLP) for edge deployment on the Qualcomm QRB2210 processor.

*   **Architecture**: A four-layer MLP consisting of 128, 64, and 32 hidden units.
*   **Optimization**: We used Dropout (0.3) and L2 Regularization to improve the model's ability to generalize across different users.
*   **Deployment**: The model was converted to a Float16 TFLite format to ensure high throughput on the ARM core.

## Feature Engineering
To provide the AI with high-quality input, we built a real-time signal processing engine that extracts 60 distinct features:
1.  **Statistical Analysis**: The system calculates the mean, standard deviation, minimum, and maximum of sensor data over a 200-sample rolling window.
2.  **Frequency Domain (FFT)**: We compute the spectral power for each sensor axis. This helps the AI identify micro-tremors and muscle fatigue that are often invisible to simple threshold-based systems.
3.  **Coordinate Normalization**: By calculating orientations relative to the neck sensor, the system remains accurate regardless of which direction the user is facing.

## Training and Accuracy
The model was trained on an AMD Ryzen platform using ROCm acceleration. We used data augmentation techniques, including synthetic jitter generation, to double our dataset and improve robustness. The final model achieved a cross-validation accuracy of 98.18%.
