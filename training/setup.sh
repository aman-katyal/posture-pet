#!/bin/bash

# Setup environment for Edge Impulse local training with AMD GPU (ROCm)

echo "Creating virtual environment..."
python3 -m venv .venv
source .venv/bin/activate

echo "Installing ROCm-compatible TensorFlow..."
# Note: For gfx1100 (Radeon 780M), we use the standard tensorflow-rocm or the one from AMD index
pip install --upgrade pip
pip install tensorflow-rocm

echo "Installing Edge Impulse SDK and other dependencies..."
pip install edgeimpulse numpy pandas matplotlib scikit-learn

echo "Verifying GPU detection..."
python3 -c "import tensorflow as tf; print('GPUs found:', tf.config.list_physical_devices('GPU'))"

echo ""
echo "Setup complete!"
echo "To activate the environment: source .venv/bin/activate"
echo "To run training: python training/train.py"
echo "Note: Edit training/train.py with your API Key before running."
