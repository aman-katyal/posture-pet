import cv2
import mediapipe as mp
import numpy as np
import time
import json
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
from typing import List

# --- FILTERS & ENGINE ---
class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.x_prev, self.dx_prev, self.t_prev = None, 0.0, None

    def __call__(self, x):
        t = time.time()
        if self.t_prev is None:
            self.t_prev, self.x_prev = t, x
            return x
        dt = t - self.t_prev
        if dt <= 0: return self.x_prev
        alpha = 1.0 / (1.0 + (1.0 / (2 * np.pi * self.min_cutoff)) / dt)
        dx = (x - self.x_prev) / dt
        edx = self.dx_prev + alpha * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * abs(edx)
        alpha = 1.0 / (1.0 + (1.0 / (2 * np.pi * cutoff)) / dt)
        x_filtered = self.x_prev + alpha * (x - self.x_prev)
        self.t_prev, self.x_prev, self.dx_prev = t, x_filtered, edx
        return x_filtered

class PostureEngine:
    def __init__(self):
        self.filters = [OneEuroFilter(0.5, 0.01) for _ in range(3)]
        self.baselines = {"p": 0, "y": 0, "r": 0}
        self.model_pts = np.array([
            (0, 0, 0), (0, -0.35, -0.05), (-0.15, -0.35, 0), (0.15, -0.35, 0), (-0.15, -0.75, 0), (0.15, -0.75, 0)
        ], dtype=np.float32)

    def solve(self, landmarks, w, h):
        def pt(i): return [landmarks[i].x * w, landmarks[i].y * h]
        img_pts = np.array([pt(0), [(pt(11)[0]+pt(12)[0])/2, (pt(11)[1]+pt(12)[1])/2], pt(11), pt(12), pt(23), pt(24)], dtype=np.float32)
        cam_mat = np.array([[w, 0, w/2], [0, w, h/2], [0, 0, 1]], dtype=np.float32)
        success, rvec, _ = cv2.solvePnP(self.model_pts, img_pts, cam_mat, np.zeros(4))
        if not success: return None
        rmat, _ = cv2.Rodrigues(rvec)
        p = np.degrees(np.arctan2(-rmat[2,1], rmat[2,2]))
        y = np.degrees(np.arctan2(rmat[2,0], np.sqrt(rmat[2,1]**2 + rmat[2,2]**2)))
        r = np.degrees(np.arctan2(rmat[1,0], rmat[0,0]))
        if r > 90: r -= 180
        if r < -90: r += 180
        return {"p": self.filters[0](p), "y": self.filters[1](y), "r": self.filters[2](r)}

# --- GLOBAL STATE ---
app = FastAPI()
engine = PostureEngine()
mp_pose = mp.solutions.pose
# Using model_complexity=2 for the heaviest/most accurate model
pose = mp_pose.Pose(
    model_complexity=2, 
    smooth_landmarks=True, 
    enable_segmentation=True,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.latest_frame = None
        self.latest_data = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
        self.metrics = {"slump": 0.0, "lean": 0.0}
        self.baselines = {"slump": 15.0, "lean": 0.0, "roll": 0.0}
        self.calibration_requested = False
        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        while self.running:
            success, frame = self.cap.read()
            if not success:
                time.sleep(0.1)
                continue
            
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb_frame)
            h, w, _ = frame.shape

            if results.segmentation_mask is not None:
                condition = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
                bg_image = np.zeros(frame.shape, dtype=np.uint8)
                frame = np.where(condition, frame, bg_image)

            if results.pose_landmarks and results.pose_world_landmarks:
                data = engine.solve(results.pose_landmarks.landmark, w, h)
                wlm = results.pose_world_landmarks.landmark
                
                # --- WORLD-SPACE 3D METRICS ---
                # Origin (0,0,0) is at the center of the hips.
                # All values are in meters.
                
                # 1. SLUMP: Craniovertebral Angle Approximation (Z-Y plane)
                # Using 3D World coordinates to measure how far forward the head is.
                mid_shoulder_z = (wlm[11].z + wlm[12].z) / 2
                mid_shoulder_y = (wlm[11].y + wlm[12].y) / 2
                nose_z = wlm[0].z
                nose_y = wlm[0].y
                
                # Dz and Dy in meters
                dz = mid_shoulder_z - nose_z
                dy = mid_shoulder_y - nose_y
                # Slump Angle (degrees from vertical)
                current_slump = np.degrees(np.arctan2(dz, dy))
                
                # 2. LEAN: Torso Lateral Tilt (X-Y plane)
                # Mid-hip is origin (approx 0,0,0 in world landmarks)
                mid_shoulder_x = (wlm[11].x + wlm[12].x) / 2
                mid_shoulder_y = (wlm[11].y + wlm[12].y) / 2
                # Use mid-hip world coords (23 and 24)
                mid_hip_x = (wlm[23].x + wlm[24].x) / 2
                mid_hip_y = (wlm[23].y + wlm[24].y) / 2
                
                dx_lean = mid_shoulder_x - mid_hip_x
                dy_lean = mid_hip_y - mid_shoulder_y # Positive up
                current_lean = np.degrees(np.arctan2(dx_lean, dy_lean))

                if self.calibration_requested:
                    if data:
                        engine.baselines = data
                        self.baselines["slump"] = current_slump
                        self.baselines["lean"] = current_lean
                        self.calibration_requested = False

                if data:
                    self.latest_data["pitch"] = round(data["p"] - engine.baselines["p"], 2)
                    self.latest_data["yaw"] = round(data["y"] - engine.baselines["y"], 2)
                    self.latest_data["roll"] = round(data["r"] - engine.baselines["r"], 2)
                    
                    self.metrics["slump"] = current_slump
                    self.metrics["lean"] = current_lean

                    # --- VISUAL OVERLAY ---
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0, 0), (320, 220), (20, 20, 20), -1)
                    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

                    def draw_bar(y, label, value, max_val, color):
                        cv2.putText(frame, label, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.rectangle(frame, (80, y - 12), (300, y + 2), (50, 50, 50), -1)
                        width = int(min(abs(value) / max_val, 1.0) * 220)
                        cv2.rectangle(frame, (80, y - 12), (80 + width, y + 2), color, -1)
                        cv2.putText(frame, f"{abs(value):.1f}", (305, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

                    # Calculate Violations relative to Baseline
                    pitch_dev = self.latest_data["pitch"]
                    # Deviations in degrees
                    slump_violation = current_slump - self.baselines["slump"]
                    lean_violation = current_lean - self.baselines["lean"]

                    red, green = (0, 0, 255), (0, 255, 0)
                    
                    # Thresholds in degrees
                    draw_bar(40,  "HEAD",  pitch_dev, 20, red if abs(pitch_dev) > 12 else green)
                    draw_bar(80,  "SLUMP", slump_violation, 15, red if slump_violation > 8 else green)
                    draw_bar(120, "LEAN",  lean_violation, 15, red if abs(lean_violation) > 8 else green)

                    status_color = green
                    status_msg = "POSTURE: GOOD"
                    if abs(pitch_dev) > 12 or slump_violation > 8 or abs(lean_violation) > 8:
                        status_color = red
                        status_msg = "POSTURE: VIOLATED"
                    
                    cv2.putText(frame, status_msg, (15, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                    cv2.putText(frame, "C: CALIBRATE BASELINE", (15, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
                    
                    mp.solutions.drawing_utils.draw_landmarks(
                        frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                        mp.solutions.drawing_utils.DrawingSpec(color=(255, 255, 255), thickness=1, circle_radius=1),
                        mp.solutions.drawing_utils.DrawingSpec(color=status_color, thickness=2, circle_radius=1)
                    )


            _, buffer = cv2.imencode('.jpg', frame)
            self.latest_frame = buffer.tobytes()
            time.sleep(0.01)

            _, buffer = cv2.imencode('.jpg', frame)
            self.latest_frame = buffer.tobytes()
            time.sleep(0.01)

    def get_frame(self):
        return self.latest_frame

    def get_data(self):
        return self.latest_data

import threading
camera = Camera()

# --- GENERATORS ---
def gen_frames():
    while True:
        frame = camera.get_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.04) # ~25 FPS

# --- ROUTES ---
@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(gen_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(camera.get_data())
            try:
                # Non-blocking receive
                data = await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
                msg = json.loads(data)
                if msg.get("command") == "calibrate":
                    camera.calibration_requested = True
            except asyncio.TimeoutError:
                pass
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass

app.mount("/", StaticFiles(directory="frontend", html=True), name="frontend")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
