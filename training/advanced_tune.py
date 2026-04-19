import os
import tensorflow as tf
import numpy as np
import glob
import json
from sklearn.model_selection import KFold
from scipy.stats import skew, kurtosis

# --- CONFIGURATION ---
os.environ["HSA_OVERRIDE_GFX_VERSION"] = "11.0.0"

def extract_advanced_features(values):
    """
    Simulates Edge Impulse 'Spectral Analysis' + 'Statistics' blocks.
    Input: (window_size, 12)
    Output: (N_features,)
    """
    features = []
    # For each of the 12 quaternion channels
    for i in range(12):
        channel = values[:, i]
        # Statistics
        features.append(np.mean(channel))
        features.append(np.std(channel))
        features.append(np.min(channel))
        features.append(np.max(channel))
        # Spectral energy (simple version)
        fft_vals = np.abs(np.fft.rfft(channel))
        features.append(np.mean(fft_vals))
    return np.array(features)

def load_and_augment_data(data_dir, augment=True):
    X_raw, y = [], []
    label_map = {"good": 0, "mid": 1, "bad": 2}
    files = glob.glob(os.path.join(data_dir, "training", "*.json"))
    
    for f in files:
        basename = os.path.basename(f).split('.')[0]
        if basename not in label_map: continue
        with open(f, 'r') as j:
            vals = np.array(json.load(j)['payload']['values'])
            X_raw.append(vals)
            y.append(label_map[basename])
    
    # Feature extraction
    X_features = np.array([extract_advanced_features(sample) for sample in X_raw])
    y = np.array(y)
    
    if augment:
        # Simple Data Augmentation: Add Gaussian Noise
        noise = np.random.normal(0, 0.01, X_features.shape)
        X_aug = X_features + noise
        X_features = np.concatenate([X_features, X_aug], axis=0)
        y = np.concatenate([y, y], axis=0)
        
    return X_features, y, X_raw

def build_cnn_model(input_shape):
    # CNN expects (timesteps, channels)
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=input_shape),
        tf.keras.layers.Conv1D(16, 3, activation='relu', padding='same'),
        tf.keras.layers.MaxPooling1D(2),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(32, activation='relu'),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(3, activation='softmax')
    ])
    model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return model

def build_dense_model(input_shape):
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=input_shape),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dense(3, activation='softmax')
    ])
    model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return model

def main():
    print("Loading data and generating advanced features...")
    X, y, X_raw = load_and_augment_data("data")
    print(f"Total samples after augmentation: {len(X)}")
    
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    
    architectures = [
        ("Advanced Dense MLP", build_dense_model, (X.shape[1],)),
        # CNN requires raw time series, we'd need a different loading strategy for it
        # For now, let's focus on the feature-engineered Dense model which is 
        # usually superior for small IMU datasets on Edge Impulse.
    ]
    
    best_overall_acc = 0
    
    for name, model_fn, shape in architectures:
        print(f"\nEvaluating {name} with 5-Fold Cross Validation...")
        fold_accs = []
        
        for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
            model = model_fn(shape)
            early_stop = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=15, restore_best_weights=True)
            
            model.fit(X[train_idx], y[train_idx], epochs=100, batch_size=16, 
                      validation_data=(X[val_idx], y[val_idx]), callbacks=[early_stop], verbose=0)
            
            _, acc = model.evaluate(X[val_idx], y[val_idx], verbose=0)
            fold_accs.append(acc)
            print(f"Fold {fold+1} Acc: {acc:.2%}")
            
        avg_acc = np.mean(fold_accs)
        print(f"Average CV Accuracy for {name}: {avg_acc:.2%}")
        
        if avg_acc > best_overall_acc:
            best_overall_acc = avg_acc
            # Save the final best model (trained on all data)
            final_model = model_fn(shape)
            final_model.fit(X, y, epochs=150, verbose=0)
            os.makedirs("models", exist_ok=True)
            final_model.save(f"models/advanced_posture_model.h5")
            
            converter = tf.lite.TFLiteConverter.from_keras_model(final_model)
            with open("models/advanced_posture_model.tflite", "wb") as f:
                f.write(converter.convert())

    print(f"\nAdvanced pipeline training complete. Best CV Accuracy: {best_overall_acc:.2%}")

if __name__ == "__main__":
    main()
