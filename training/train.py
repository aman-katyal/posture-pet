import os
import tensorflow as tf
import numpy as np
import glob
import json
from sklearn.model_selection import train_test_split

# --- CONFIGURATION ---
API_KEY = "ei_ae7962939fff88e0a205069eca7afe9e09793314bf00be79"
os.environ["HSA_OVERRIDE_GFX_VERSION"] = "11.0.0"

def setup_gpu():
    print("Checking for GPU...")
    gpus = tf.config.list_physical_devices('GPU')
    if gpus:
        try:
            for gpu in gpus:
                tf.config.experimental.set_memory_growth(gpu, True)
            print(f"Successfully configured {len(gpus)} GPU(s).")
        except RuntimeError as e:
            print(f"GPU Configuration Error: {e}")
    else:
        print("WARNING: No GPU found. Training on CPU.")

def load_data(data_dir, folder="training"):
    """
    Loads JSON samples and averages the window to get 12 features per sample.
    """
    X = []
    y = []
    
    label_map = {"good": 0, "mid": 1, "bad": 2}
    
    search_path = os.path.join(data_dir, folder, "*.json")
    files = glob.glob(search_path)
    
    if not files:
        print(f"No files found in {search_path}")
        return None, None

    print(f"Loading and preprocessing {len(files)} samples from {folder}...")
    for f in files:
        basename = os.path.basename(f).split('.')[0]
        if basename not in label_map:
            # If it's testing data, the label might be different or not in name
            # For this exercise, we assume the 'training' folder has labeled files
            continue
            
        with open(f, 'r') as j:
            content = json.load(j)
            values = np.array(content['payload']['values'])
            avg_features = np.mean(values, axis=0)
            
            X.append(avg_features)
            y.append(label_map[basename])

    return np.array(X), np.array(y)

def build_model(input_shape, num_classes):
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=input_shape),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(32, activation='relu'),
        tf.keras.layers.Dense(num_classes, activation='softmax')
    ])
    
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    return model

def main():
    setup_gpu()

    # 1. Load Data
    X, y = load_data("data", "training")
    if X is None or len(X) == 0:
        print("No training data loaded. Check filenames and paths.")
        return

    print(f"Total Dataset shape: {X.shape}")
    print(f"Labels distribution: {np.bincount(y)}")

    # 2. Split Data: 80% Train, 20% Val
    # We will also use the separate 'testing' folder for final testing if available
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.20, random_state=42)
    
    # Optional: If you want a 70/15/15 split manually from the training data:
    # X_train, X_temp, y_train, y_temp = train_test_split(X, y, test_size=0.30, random_state=42)
    # X_val, X_test, y_val, y_test = train_test_split(X_temp, y_temp, test_size=0.50, random_state=42)

    print(f"Train set: {len(X_train)} samples")
    print(f"Val set:   {len(X_val)} samples")

    # 3. Build and Train
    model = build_model(input_shape=(12,), num_classes=3)
    model.summary()

    print("\nStarting training on AMD APU...")
    # Using EarlyStopping to avoid overfitting on small datasets
    early_stop = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=15, restore_best_weights=True)

    history = model.fit(
        X_train, y_train,
        epochs=200,
        batch_size=8,
        validation_data=(X_val, y_val),
        callbacks=[early_stop],
        verbose=1
    )

    # 4. Final Evaluation on Validation set
    val_loss, val_acc = model.evaluate(X_val, y_val, verbose=0)
    print(f"\nFinal Validation Accuracy: {val_acc:.2%}")

    # 5. Save Model
    os.makedirs("models", exist_ok=True)
    model.save("models/posture_model.h5")
    
    # Export to TFLite for the Qualcomm processor on Uno Q
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    with open("models/posture_model.tflite", "wb") as f:
        f.write(tflite_model)
    
    print("\nModel saved to models/posture_model.h5 and models/posture_model.tflite")

if __name__ == "__main__":
    main()
