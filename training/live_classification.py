import asyncio
import struct
import numpy as np
import tensorflow as tf
from bleak import BleakClient, BleakScanner
from scipy.stats import skew, kurtosis
import time

ESP32_ADDRESS = "90:70:69:35:2C:E2"
TARGET_CHAR_UUID = "33333333-2222-2222-1111-111100000000"
MODEL_PATH = "models/advanced_posture_model.tflite"
WINDOW_SIZE = 200
LABELS = ["Good", "Mid", "Bad"]

class LiveClassifier:
    def __init__(self, model_path):
        # Load TFLite model
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        self.window = []
        self.last_inference_time = 0
        
    def extract_features(self, data):
        """Matches advanced_tune.py feature extraction"""
        features = []
        # data is (200, 12)
        for i in range(12):
            channel = data[:, i]
            features.append(np.mean(channel))
            features.append(np.std(channel))
            features.append(np.min(channel))
            features.append(np.max(channel))
            fft_vals = np.abs(np.fft.rfft(channel))
            features.append(np.mean(fft_vals))
        return np.array(features, dtype=np.float32)

    def process_new_sample(self, sample):
        self.window.append(sample)
        if len(self.window) > WINDOW_SIZE:
            self.window.pop(0)
            
        # Run inference every 10 samples (at ~50Hz sampling, this is every 200ms)
        # Or you can do it every sample if CPU allows
        if len(self.window) == WINDOW_SIZE:
            current_time = time.time()
            if current_time - self.last_inference_time > 0.1: # 10Hz UI update
                self.run_inference()
                self.last_inference_time = current_time

    def run_inference(self):
        data = np.array(self.window)
        features = self.extract_features(data).reshape(1, -1)
        
        self.interpreter.set_tensor(self.input_details[0]['index'], features)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        
        idx = np.argmax(output_data)
        confidence = output_data[idx]
        
        # ANSI colors for status
        color = "\033[92m" if idx == 0 else ("\033[93m" if idx == 1 else "\033[91m")
        reset = "\033[0m"
        
        print(f"\rPosture: {color}{LABELS[idx]:<5}{reset} | Confidence: {confidence:.2%} | Samples: {len(self.window)}", end="")

classifier = LiveClassifier(MODEL_PATH)

def notification_handler(sender, data):
    try:
        # 12 floats = 48 bytes. Order: qw, qx, qy, qz for 3 sensors
        # Training data format: [nx, ny, nz, nw, lx, ly, lz, lw, rx, ry, rz, rw]
        # Our struct: [qw, qx, qy, qz] * 3
        # IMPORTANT: The training data used (x,y,z,w). We must match that order.
        raw = struct.unpack('<ffffffffffff', data)
        
        # Rearrange from [w,x,y,z] to [x,y,z,w] to match training data
        # Sensor 1 (Neck)
        n = [raw[1], raw[2], raw[3], raw[0]]
        # Sensor 2 (Left)
        l = [raw[5], raw[6], raw[7], raw[4]]
        # Sensor 3 (Right)
        r = [raw[9], raw[10], raw[11], raw[8]]
        
        combined_sample = n + l + r
        classifier.process_new_sample(combined_sample)
        
    except Exception as e:
        print(f"\nError in notification: {e}")

async def main():
    print(f"Scanning for ESP32 ({ESP32_ADDRESS})...")
    device = await BleakScanner.find_device_by_address(ESP32_ADDRESS, timeout=10)
    
    if not device:
        print("Device not found! Is it advertising?")
        return

    async with BleakClient(device) as client:
        print(f"Connected to {device.name}")
        await client.start_notify(TARGET_CHAR_UUID, notification_handler)
        print("Live Classification Active. Sit in different postures!")
        
        try:
            while True:
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            await client.stop_notify(TARGET_CHAR_UUID)

if __name__ == "__main__":
    asyncio.run(main())
