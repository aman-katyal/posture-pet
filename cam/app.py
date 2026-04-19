import cv2
import mediapipe as mp
import numpy as np
import time
import json
import asyncio
import socket
import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
from typing import List

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

app = FastAPI()
engine = PostureEngine()
mp_pose = mp.solutions.pose
mp_selfie = mp.solutions.selfie_segmentation
segmenter = mp_selfie.SelfieSegmentation(model_selection=1) # 1 = Landscape mode (accurate)

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
        self.metrics = {"head": 0.0, "slump": 0.0, "lean": 0.0}
        self.baselines = {"head": 0.0, "slump": 0.0, "lean": 0.0}
        
        # Teaching limits (Degrees of deviation from baseline)
        self.mid_limits = {"head": 4.0, "slump": 2.5, "lean": 2.0}
        self.bad_limits = {"head": 8.0, "slump": 5.0, "lean": 4.0}
        self.thresholds = self.bad_limits.copy() 

        # Scoring & Calibration state
        self.posture_score = 100.0
        self.classification = "GOOD"
        self.confidence = 100.0
        self.is_auto_calibrating = True
        self.stability_buffer = [] # Store last 60 frames for stability check

        self.head_filter = OneEuroFilter(0.4, 0.01)
        self.slump_filter = OneEuroFilter(0.4, 0.01)
        self.lean_filter = OneEuroFilter(0.4, 0.01)
        self.confidence_filter = OneEuroFilter(0.2, 0.0) # Faster confidence smoothing

        self.violation_history = []
        self.status_msg = "CALIBRATING..."
        self.status_color = (255, 165, 0) # Orange

        self.calibration_requested = False
        self.sensitivity = 1.0
        self.running = True

        # Persistence & Robustness State
        self.raw_history = {"head": [], "slump": [], "lean": []} # For Median filtering
        self.max_d_filter = OneEuroFilter(0.8, 0.0) # More responsive max deviation filter
        self.live_stability = 100.0
        
        # State Debouncer
        self.target_classification = "GOOD"
        self.classification_hold_frames = 0
        self.hold_thresholds = {"GOOD": 4, "MID": 8, "BAD": 12} # Faster responsive debouncing
        
        self.bad_posture_start_time = None # For Auto-recalibration
        self.ema_beta = 0.001 # Slow dynamic adaptation

        # --- UDP SOCKET SETUP (Target: Uno Q) ---
        self.udp_ip = "10.10.11.84"
        self.udp_port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_udp_send = 0.0

        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        import threading
        while self.running:
            success, frame = self.cap.read()
            if not success:
                time.sleep(0.1)
                continue
            
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            seg_results = segmenter.process(rgb_frame)
            if seg_results.segmentation_mask is not None:
                mask = seg_results.segmentation_mask
                # Aggressive threshold (0.6) and blur for clean silhouette
                mask = cv2.GaussianBlur(mask, (11, 11), 0)
                condition = np.stack((mask,) * 3, axis=-1) > 0.6
                
                # Apply blackout to the frame
                bg_image = np.zeros(frame.shape, dtype=np.uint8)
                frame = np.where(condition, frame, bg_image)
                
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Pose detector now only sees the isolated person against a black void
            results = pose.process(rgb_frame)
            h, w, _ = frame.shape

            if results.pose_landmarks and results.pose_world_landmarks:
                wlm = results.pose_world_landmarks.landmark
                
                mid_sh_z, mid_sh_y, mid_sh_x = (wlm[11].z + wlm[12].z)/2, (wlm[11].y + wlm[12].y)/2, (wlm[11].x + wlm[12].x)/2
                mid_hip_z, mid_hip_y, mid_hip_x = (wlm[23].z + wlm[24].z)/2, (wlm[23].y + wlm[24].y)/2, (wlm[23].x + wlm[24].x)/2
                nose = wlm[0]

                raw_head = np.degrees(np.arctan2(mid_sh_z - nose.z, mid_sh_y - nose.y))
                raw_slump = np.degrees(np.arctan2(mid_hip_z - mid_sh_z, mid_hip_y - mid_sh_y))
                raw_lean = (np.degrees(np.arctan2(wlm[12].y - wlm[11].y, wlm[12].x - wlm[11].x)) * 0.6) + \
                           (np.degrees(np.arctan2(mid_sh_x - mid_hip_x, abs(mid_hip_y - mid_sh_y))) * 0.4)

                # --- ADVANCED FILTERING (MEDIAN + ONEEURO) ---
                def apply_robust_filter(key, val, filter_obj):
                    self.raw_history[key].append(val)
                    if len(self.raw_history[key]) > 9: self.raw_history[key].pop(0)
                    median_val = np.median(self.raw_history[key])
                    return filter_obj(median_val)

                self.metrics["head"] = apply_robust_filter("head", raw_head, self.head_filter)
                self.metrics["slump"] = apply_robust_filter("slump", raw_slump, self.slump_filter)
                self.metrics["lean"] = apply_robust_filter("lean", raw_lean, self.lean_filter)

                # --- SMART AUTO-CALIBRATION (Interquartile Mean) ---
                if self.is_auto_calibrating:
                    self.stability_buffer.append([self.metrics["head"], self.metrics["slump"], self.metrics["lean"]])
                    if len(self.stability_buffer) > 60: # ~2 seconds
                        self.stability_buffer.pop(0)
                        vars_s = np.var(self.stability_buffer, axis=0)
                        if np.all(vars_s < 0.2): # Very stable
                            # Use Interquartile Mean (Trim top/bottom 10%)
                            buf = np.array(self.stability_buffer)
                            for i, k in enumerate(["head", "slump", "lean"]):
                                data = np.sort(buf[:, i])
                                low, high = int(len(data)*0.1), int(len(data)*0.9)
                                self.baselines[k] = np.mean(data[low:high])
                            self.is_auto_calibrating = False
                            self.status_msg = "CALIBRATION OK"
                
                if self.calibration_requested:
                    self.baselines["head"], self.baselines["slump"], self.baselines["lean"] = self.metrics["head"], self.metrics["slump"], self.metrics["lean"]
                    self.calibration_requested = False
                    self.is_auto_calibrating = False

                head_dev = abs(self.metrics["head"] - self.baselines["head"])
                slump_dev = max(0, self.metrics["slump"] - self.baselines["slump"])
                lean_dev = abs(self.metrics["lean"] - self.baselines["lean"])

                # --- DYNAMIC TOLERANCE ADAPTATION (Slow EMA) ---
                if not self.is_auto_calibrating:
                    # If posture is GOOD, slowly tighten. If MID, slowly relax.
                    if self.classification == "GOOD":
                        self.mid_limits["head"] *= (1.0 - self.ema_beta * 0.1)
                    elif self.classification == "MID":
                        self.mid_limits["head"] *= (1.0 + self.ema_beta)

                t_head, t_slump, t_lean = self.thresholds["head"]/self.sensitivity, self.thresholds["slump"]/self.sensitivity, self.thresholds["lean"]/self.sensitivity

                def get_norm_d(val, mid, bad):
                    if val <= mid:
                        return (val / mid) * 0.5 if mid > 0 else 0
                    else:
                        return 0.5 + ((val - mid) / (bad - mid)) * 0.5 if (bad - mid) > 0 else 0.5

                d_h = get_norm_d(head_dev, self.mid_limits["head"], self.bad_limits["head"])
                d_s = get_norm_d(slump_dev, self.mid_limits["slump"], self.bad_limits["slump"])
                d_l = get_norm_d(lean_dev, self.mid_limits["lean"], self.bad_limits["lean"])
                raw_max_d = max(d_h, d_s, d_l)
                
                # --- HEAVY DEVIATION SMOOTHING ---
                max_d = self.max_d_filter(raw_max_d)

                # --- STATE DEBOUNCER (Hysteresis + Temporal) ---
                if self.classification == "GOOD":
                    # Only leave GOOD if deviation > 0.65
                    if max_d < 0.65: new_class = "GOOD"
                    elif max_d < 1.25: new_class = "MID"
                    else: new_class = "BAD"
                elif self.classification == "MID":
                    # Return to GOOD if < 0.55. Stay in MID until > 1.25
                    if max_d < 0.55: new_class = "GOOD"
                    elif max_d < 1.25: new_class = "MID"
                    else: new_class = "BAD"
                else: # Currently BAD
                    # Return to MID if < 1.10. Return to GOOD if < 0.55
                    if max_d < 0.55: new_class = "GOOD"
                    elif max_d < 1.10: new_class = "MID"
                    else: new_class = "BAD"

                # --- NON-LINEAR CONFIDENCE (Squared for forgiveness) ---
                if max_d < 0.65: 
                    raw_conf = 100 * (1.0 - (max_d / 0.65)**2)
                elif max_d < 1.25: 
                    # Center of MID is around 0.95
                    raw_conf = 100 * (1.0 - (abs(max_d - 0.95) / 0.30)**2)
                else: 
                    raw_conf = min(100.0, 80 + (max_d - 1.25) * 20)

                # --- LIVE STABILITY CALCULATION ---
                # Based on the variance of the last 9 frames in raw_history
                all_raw = np.array([self.raw_history["head"], self.raw_history["slump"], self.raw_history["lean"]])
                if all_raw.shape[1] >= 5:
                    var_sum = np.sum(np.var(all_raw, axis=1))
                    self.live_stability = max(0, min(100, 100 - (var_sum * 10))) # 0-100% stability

                if new_class != self.target_classification:
                    self.target_classification = new_class
                    self.classification_hold_frames = 0
                else:
                    self.classification_hold_frames += 1
                
                # Commit with faster thresholds
                if self.classification_hold_frames >= self.hold_thresholds.get(self.target_classification, 10):
                    self.classification = self.target_classification

                # Smooth the final displayed confidence
                self.confidence = round(self.confidence_filter(max(40.0, raw_conf)), 1)
                self.posture_score = round(max(0, 100 - (max_d * 100)), 1)

                is_violating = (max_d >= 1.0)

                # --- AUTO-RECALIBRATION LOGIC ---
                if self.classification == "BAD":
                    if self.bad_posture_start_time is None:
                        self.bad_posture_start_time = time.time()
                    elif time.time() - self.bad_posture_start_time > 10.0: # 10 seconds of BAD
                        self.calibration_requested = True
                        self.bad_posture_start_time = None
                        self.status_msg = "AUTO-RECALIBRATING..."
                else:
                    self.bad_posture_start_time = None

                # --- SEND CLASSIFICATION TO UNO Q VIA UDP (10Hz) ---
                current_time = time.time()
                if current_time - self.last_udp_send >= 0.1:
                    try:
                        payload = json.dumps({
                            "class": self.classification,
                            "conf": self.confidence,
                            "score": self.posture_score
                        }).encode('utf-8')
                        self.sock.sendto(payload, (self.udp_ip, self.udp_port))
                        self.last_udp_send = current_time
                    except Exception:
                        pass # Keep the loop running if network is down

                self.violation_history.append(is_violating)
                if len(self.violation_history) > 10: self.violation_history.pop(0)
                
                if not self.is_auto_calibrating:
                    if self.classification == "GOOD":
                        self.status_color, self.status_msg = (0, 255, 0), "POSTURE: GOOD"
                    elif self.classification == "MID":
                        self.status_color, self.status_msg = (0, 165, 255), "POSTURE: MID"
                    else:
                        self.status_color, self.status_msg = (0, 0, 255), "POSTURE: VIOLATED"

                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (320, 240), (20, 20, 20), -1)
                cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

                def draw_bar(y, label, val, t, color):
                    cv2.putText(frame, label, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                    cv2.rectangle(frame, (70, y-10), (280, y), (40, 40, 40), -1)
                    w_b = int(min(val/t, 1.0) * 210) if t > 0 else 0
                    cv2.rectangle(frame, (70, y-10), (70+w_b, y), color, -1)

                draw_bar(40, "HEAD", head_dev, t_head*2, (0, 255, 0) if head_dev < t_head else (0, 0, 255))
                draw_bar(70, "SLUMP", slump_dev, t_slump*2, (0, 255, 0) if slump_dev < t_slump else (0, 0, 255))
                draw_bar(100, "LEAN", lean_dev, t_lean*2, (0, 255, 0) if lean_dev < t_lean else (0, 0, 255))

                cv2.putText(frame, f"SCORE: {self.posture_score}%", (15, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.status_color, 2)
                cv2.putText(frame, f"{self.status_msg} ({self.sensitivity}x)", (15, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.status_color, 2)
                cv2.putText(frame, "C: CALIBRATE • +/-: SENSITIVITY", (15, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
                
                mp.solutions.drawing_utils.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    mp.solutions.drawing_utils.DrawingSpec(color=(255,255,255), thickness=1, circle_radius=1),
                    mp.solutions.drawing_utils.DrawingSpec(color=self.status_color, thickness=2, circle_radius=1))

            _, buffer = cv2.imencode('.jpg', frame)
            self.latest_frame = buffer.tobytes()
            time.sleep(0.01)

    def get_frame(self):
        return self.latest_frame

    def get_data(self):
        return {
            "head": round(self.metrics["head"] - self.baselines["head"], 1),
            "slump": round(self.metrics["slump"] - self.baselines["slump"], 1),
            "lean": round(self.metrics["lean"] - self.baselines["lean"], 1),
            "score": self.posture_score,
            "classification": self.classification,
            "confidence": self.confidence,
            "stability": round(self.live_stability, 1),
            "status": self.status_msg
        }

import threading
camera = Camera()

def gen_frames():
    while True:
        frame = camera.get_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.04) # ~25 FPS

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
                if cmd == "teach_good":
                    camera.baselines = {"head": camera.metrics["head"], "slump": camera.metrics["slump"], "lean": camera.metrics["lean"]}
                    camera.is_auto_calibrating = False
                    camera.status_msg = "GOOD POSE SAVED"
                elif cmd == "teach_mid":
                    camera.mid_limits["head"] = max(1.0, abs(camera.metrics["head"] - camera.baselines["head"]))
                    camera.mid_limits["slump"] = max(0.5, abs(camera.metrics["slump"] - camera.baselines["slump"]))
                    camera.mid_limits["lean"] = max(0.5, abs(camera.metrics["lean"] - camera.baselines["lean"]))
                    camera.status_msg = "MID POSE SAVED"
                elif cmd == "teach_bad":
                    camera.bad_limits["head"] = max(camera.mid_limits["head"] + 1.0, abs(camera.metrics["head"] - camera.baselines["head"]))
                    camera.bad_limits["slump"] = max(camera.mid_limits["slump"] + 0.5, abs(camera.metrics["slump"] - camera.baselines["slump"]))
                    camera.bad_limits["lean"] = max(camera.mid_limits["lean"] + 0.5, abs(camera.metrics["lean"] - camera.baselines["lean"]))
                    camera.thresholds = camera.bad_limits.copy()
                    camera.status_msg = "BAD POSE SAVED"
                elif cmd == "calibrate":
                    camera.calibration_requested = True
                    # save_settings will be called if auto-cal finishes or via explicit teach
                elif cmd == "increase_sensitivity":
                    camera.sensitivity = round(min(5.0, camera.sensitivity + 0.1), 1)
                elif cmd == "decrease_sensitivity":
                    camera.sensitivity = round(max(0.1, camera.sensitivity - 0.1), 1)
                elif cmd == "capture_limit":
                    # Calculate current deviation from baseline
                    h_dev = abs(camera.metrics["head"] - camera.baselines["head"])
                    s_dev = abs(camera.metrics["slump"] - camera.baselines["slump"])
                    l_dev = abs(camera.metrics["lean"] - camera.baselines["lean"])
                    
                    # Set thresholds to current deviation (with a floor of 1.0 to avoid 0-thresholds)
                    camera.thresholds["head"] = max(1.5, round(h_dev, 1))
                    camera.thresholds["slump"] = max(1.0, round(s_dev, 1))
                    camera.thresholds["lean"] = max(1.0, round(l_dev, 1))
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
