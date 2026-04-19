import os
import tensorflow as tf
import numpy as np
import glob
import json
from sklearn.model_selection import train_test_split

# --- CONFIGURATION ---
os.environ["HSA_OVERRIDE_GFX_VERSION"] = "11.0.0"

def load_data(data_dir, folder="training"):
    X, y = [] , []
    label_map = {"good": 0, "mid": 1, "bad": 2}
    search_path = os.path.join(data_dir, folder, "*.json")
    files = glob.glob(search_path)
    
    for f in files:
        basename = os.path.basename(f).split('.')[0]
        if basename not in label_map: continue
        with open(f, 'r') as j:
            content = json.load(j)
            values = np.array(content['payload']['values'])
            X.append(np.mean(values, axis=0))
            y.append(label_map[basename])
    return np.array(X), np.array(y)

def build_model(hp):
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(12,)),
        tf.keras.layers.Dense(hp['units1'], activation='relu'),
        tf.keras.layers.Dropout(hp['dropout']),
        tf.keras.layers.Dense(hp['units2'], activation='relu'),
        tf.keras.layers.Dense(3, activation='softmax')
    ])
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=hp['lr']),
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    return model

def main():
    X, y = load_data("data", "training")
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.20, random_state=42)

    # Search space
    units1_options = [32, 64, 128]
    units2_options = [16, 32]
    dropout_options = [0.1, 0.2, 0.3]
    lr_options = [0.001, 0.0005]
    
    results = []
    best_acc = 0
    best_hp = None

    print(f"Starting Hyperparameter Tuning across {len(units1_options)*len(units2_options)*len(dropout_options)*len(lr_options)} combinations...")

    for u1 in units1_options:
        for u2 in units2_options:
            for d in dropout_options:
                for lr in lr_options:
                    hp = {'units1': u1, 'units2': u2, 'dropout': d, 'lr': lr}
                    model = build_model(hp)
                    
                    # Small patience for tuning
                    early_stop = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True)
                    
                    history = model.fit(
                        X_train, y_train,
                        epochs=50,
                        batch_size=8,
                        validation_data=(X_val, y_val),
                        callbacks=[early_stop],
                        verbose=0
                    )
                    
                    val_acc = max(history.history['val_accuracy'])
                    print(f"Units:({u1},{u2}), Drop:{d}, LR:{lr} -> Val Acc: {val_acc:.2%}")
                    
                    results.append((hp, val_acc))
                    if val_acc > best_acc:
                        best_acc = val_acc
                        best_hp = hp

    print("\n--- TUNING REPORT ---")
    print(f"Best Accuracy: {best_acc:.2%}")
    print(f"Best Config: {best_hp}")
    
    # Save the absolute best one
    final_model = build_model(best_hp)
    final_model.fit(X_train, y_train, epochs=150, validation_data=(X_val, y_val), verbose=0)
    final_model.save("models/best_tuned_model.h5")
    
    # Export best to TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(final_model)
    with open("models/best_tuned_model.tflite", "wb") as f:
        f.write(converter.convert())

if __name__ == "__main__":
    main()
