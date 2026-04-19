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
        # Increased filtering for head pose (min_cutoff=0.05 for maximum smoothness)
        self.filters = [OneEuroFilter(0.05, 0.01) for _ in range(3)]
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
# Dedicated segmentation model for much higher accuracy
mp_selfie = mp.solutions.selfie_segmentation
segmenter = mp_selfie.SelfieSegmentation(model_selection=1) # 1 = Landscape mode (accurate)

# Using model_complexity=2 for the heaviest/most accurate model
pose = mp_pose.Pose(
    model_complexity=2, 
    smooth_landmarks=True, 
    enable_segmentation=False, # Use dedicated segmenter instead
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8
)

class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.latest_frame = None
        self.latest_data = {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
        self.metrics = {"head": 0.0, "slump": 0.0, "lean": 0.0}
        self.baselines = {"head": 0.0, "slump": 0.0, "lean": 0.0}
        
        # User-definable thresholds
        self.thresholds = {"head": 6.0, "slump": 2.5, "lean": 2.0}
        
        # Faster filters (min_cutoff=0.3 instead of 0.1) for better responsiveness
        self.head_filter = OneEuroFilter(0.3, 0.01)
        self.slump_filter = OneEuroFilter(0.3, 0.01)
        self.lean_filter = OneEuroFilter(0.3, 0.01)
        
        # Reduced buffer (10 frames) for faster status transitions
        self.violation_history = []
        self.status_msg = "POSTURE: GOOD"
        self.status_color = (0, 255, 0)
        
        self.calibration_requested = False
        self.sensitivity = 1.0
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
            
            # --- HIGH-ACCURACY ISOLATION ---
            seg_results = segmenter.process(rgb_frame)
            if seg_results.segmentation_mask is not None:
                mask = seg_results.segmentation_mask
                # Aggressive threshold (0.6) and blur for clean silhouette
                mask = cv2.GaussianBlur(mask, (11, 11), 0)
                condition = np.stack((mask,) * 3, axis=-1) > 0.6
                
                # Apply blackout to the frame
                bg_image = np.zeros(frame.shape, dtype=np.uint8)
                frame = np.where(condition, frame, bg_image)
                
                # CRITICAL: Re-sync the RGB frame for Pose detection so it ONLY sees the cutout
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Pose detector now only sees the isolated person against a black void
            results = pose.process(rgb_frame)
            h, w, _ = frame.shape

            if results.pose_landmarks and results.pose_world_landmarks:
                data = engine.solve(results.pose_landmarks.landmark, w, h)
                wlm = results.pose_world_landmarks.landmark
                
                # --- UNIFIED 3D METRIC ENGINE ---
                mid_sh_z = (wlm[11].z + wlm[12].z) / 2
                mid_sh_y = (wlm[11].y + wlm[12].y) / 2
                mid_sh_x = (wlm[11].x + wlm[12].x) / 2
                
                mid_hip_z = (wlm[23].z + wlm[24].z) / 2
                mid_hip_y = (wlm[23].y + wlm[24].y) / 2
                mid_hip_x = (wlm[23].x + wlm[24].x) / 2
                
                nose = wlm[0]

                # 1. HEAD (Nose-to-Shoulder Pitch)
                head_dz = mid_sh_z - nose.z
                head_dy = mid_sh_y - nose.y
                raw_head = np.degrees(np.arctan2(head_dz, head_dy))
                
                # 2. SLUMP (Shoulder-to-Hip Pitch)
                slump_dz = mid_hip_z - mid_sh_z
                slump_dy = mid_hip_y - mid_sh_y
                raw_slump = np.degrees(np.arctan2(slump_dz, slump_dy))
                
                # 3. LEAN (Torso Lateral tilt)
                # Sensitive Lean: Pure shoulder line roll + torso shift
                shoulder_dx = wlm[12].x - wlm[11].x
                shoulder_dy = wlm[12].y - wlm[11].y
                shoulder_roll = np.degrees(np.arctan2(shoulder_dy, shoulder_dx))
                torso_tilt = np.degrees(np.arctan2(mid_sh_x - mid_hip_x, abs(mid_hip_y - mid_sh_y)))
                raw_lean = (shoulder_roll * 0.6) + (torso_tilt * 0.4)

                # Apply Filtering
                smoothed_head = self.head_filter(raw_head)
                smoothed_slump = self.slump_filter(raw_slump)
                smoothed_lean = self.lean_filter(raw_lean)

                if self.calibration_requested:
                    if data:
                        engine.baselines = data
                        self.baselines["head"] = smoothed_head
                        self.baselines["slump"] = smoothed_slump
                        self.baselines["lean"] = smoothed_lean
                        self.calibration_requested = False

                if data:
                    self.metrics["head"] = smoothed_head
                    self.metrics["slump"] = smoothed_slump
                    self.metrics["lean"] = smoothed_lean

                    # --- VIOLATION LOGIC ---
                    head_dev = self.metrics["head"] - self.baselines["head"]
                    slump_dev = self.metrics["slump"] - self.baselines["slump"]
                    lean_dev = self.metrics["lean"] - self.baselines["lean"]

                    # Use user-defined thresholds (with sensitivity)
                    t_head = self.thresholds["head"] / self.sensitivity
                    t_slump = self.thresholds["slump"] / self.sensitivity
                    t_lean = self.thresholds["lean"] / self.sensitivity

                    is_violating = (abs(head_dev) > t_head or 
                                    slump_dev > t_slump or 
                                    abs(lean_dev) > t_lean)

                    # Faster status transition (10 frames ~0.3s)
                    self.violation_history.append(is_violating)
                    if len(self.violation_history) > 10:
                        self.violation_history.pop(0)
                    
                    if sum(self.violation_history) >= (len(self.violation_history) * 0.7):
                        self.status_msg = "POSTURE: VIOLATED"
                        self.status_color = (0, 0, 255)
                    elif sum(self.violation_history) <= (len(self.violation_history) * 0.3):
                        self.status_msg = "POSTURE: GOOD"
                        self.status_color = (0, 255, 0)

                    # --- VISUAL OVERLAY ---
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0, 0), (320, 230), (20, 20, 20), -1)
                    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

                    def draw_bar(y, label, value, max_val, current_color):
                        cv2.putText(frame, label, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.rectangle(frame, (80, y - 12), (300, y + 2), (50, 50, 50), -1)
                        width = int(min(abs(value) / max_val, 1.0) * 220)
                        cv2.rectangle(frame, (80, y - 12), (80 + width, y + 2), current_color, -1)
                        cv2.putText(frame, f"{abs(value):.1f}", (305, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

                    draw_bar(40,  "HEAD",  head_dev,  12/self.sensitivity, (0, 0, 255) if abs(head_dev) > t_head else (0, 255, 0))
                    draw_bar(80,  "SLUMP", slump_dev, 10/self.sensitivity, (0, 0, 255) if slump_dev > t_slump else (0, 255, 0))
                    draw_bar(120, "LEAN",  lean_dev,  8/self.sensitivity, (0, 0, 255) if abs(lean_dev) > t_lean else (0, 255, 0))

                    cv2.putText(frame, self.status_msg, (15, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.status_color, 2)
                    cv2.putText(frame, f"SENSITIVITY: {self.sensitivity:.1f}x (+/-)", (15, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                    cv2.putText(frame, "C: CALIBRATE BASELINE", (15, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
                    
                    mp.solutions.drawing_utils.draw_landmarks(
                        frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                        mp.solutions.drawing_utils.DrawingSpec(color=(255, 255, 255), thickness=1, circle_radius=1),
                        mp.solutions.drawing_utils.DrawingSpec(color=self.status_color, thickness=2, circle_radius=1)
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
        return {
            "head": round(self.metrics["head"] - self.baselines["head"], 2),
            "slump": round(self.metrics["slump"] - self.baselines["slump"], 2),
            "lean": round(self.metrics["lean"] - self.baselines["lean"], 2),
            "status": self.status_msg
        }

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
            # Send current data + current thresholds
            await websocket.send_json({
                "metrics": camera.get_data(),
                "thresholds": camera.thresholds,
                "sensitivity": camera.sensitivity
            })
            try:
                # Non-blocking receive
                data = await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
                msg = json.loads(data)
                cmd = msg.get("command")
                if cmd == "calibrate":
                    camera.calibration_requested = True
                elif cmd == "increase_sensitivity":
                    camera.sensitivity = round(min(5.0, camera.sensitivity + 0.1), 1)
                elif cmd == "decrease_sensitivity":
                    camera.sensitivity = round(max(0.1, camera.sensitivity - 0.1), 1)
                elif cmd == "set_thresholds":
                    new_ts = msg.get("thresholds", {})
                    for k in ["head", "slump", "lean"]:
                        if k in new_ts:
                            camera.thresholds[k] = float(new_ts[k])
            except asyncio.TimeoutError:
                pass
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        pass

app.mount("/", StaticFiles(directory="frontend", html=True), name="frontend")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
