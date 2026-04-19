import asyncio
import struct
import numpy as np
import time
import os

# Try to import tflite_runtime for the edge device, fallback to full tf
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

from bleak import BleakClient, BleakScanner
from arduino.app_utils import Bridge

# --- CONFIGURATION ---
ESP32_ADDRESS = "90:70:69:35:2C:E2"
TARGET_CHAR_UUID = "33333333-2222-2222-1111-111100000000"
MODEL_PATH = "advanced_posture_model.tflite"
WINDOW_SIZE = 200
LABELS = ["Good", "Mid", "Bad"]

class UnoQInference:
    def __init__(self, model_path):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file {model_path} not found!")
            
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        self.window = []
        self.last_inference_time = 0
        self.current_state = -1
        
    def extract_features(self, data):
        features = []
        for i in range(12):
            channel = data[:, i]
            features.append(np.mean(channel))
            features.append(np.std(channel))
            features.append(np.min(channel))
            features.append(np.max(channel))
            fft_vals = np.abs(np.fft.rfft(channel))
            features.append(np.mean(fft_vals))
        return np.array(features, dtype=np.float32)

    def process_sample(self, sample):
        self.window.append(sample)
        if len(self.window) > WINDOW_SIZE:
            self.window.pop(0)
            
        if len(self.window) == WINDOW_SIZE:
            current_time = time.time()
            if current_time - self.last_inference_time > 0.2: # 5Hz inference
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
        
        # Only notify STM32 if state changed to reduce Bridge traffic
        if idx != self.current_state and confidence > 0.7:
            self.current_state = idx
            print(f"Postue: {LABELS[idx]} ({confidence:.2%}) -> Notifying STM32")
            try:
                # We'll map 0=90deg (Good), 1=45deg (Mid), 2=0deg (Bad) for the servo
                servo_angle = 90 if idx == 0 else (45 if idx == 1 else 0)
                Bridge.call("set_posture", idx, servo_angle)
            except Exception as e:
                print(f"Bridge error: {e}")

inference_engine = UnoQInference(MODEL_PATH)

def notification_handler(sender, data):
    try:
        raw = struct.unpack('<ffffffffffff', data)
        # Rearrange [w,x,y,z] -> [x,y,z,w] for 3 sensors
        n = [raw[1], raw[2], raw[3], raw[0]]
        l = [raw[5], raw[6], raw[7], raw[4]]
        r = [raw[9], raw[10], raw[11], raw[8]]
        inference_engine.process_sample(n + l + r)
    except Exception as e:
        print(f"Data error: {e}")

async def main():
    print(f"Uno Q Posture Detection Starting...")
    device = await BleakScanner.find_device_by_address(ESP32_ADDRESS, timeout=15)
    
    if not device:
        print("ESP32 not found. check power/BLE.")
        return

    async with BleakClient(device) as client:
        print(f"Connected to ESP32")
        await client.start_notify(TARGET_CHAR_UUID, notification_handler)
        
        while True:
            await asyncio.sleep(1.0)

if __name__ == "__main__":
    asyncio.run(main())
