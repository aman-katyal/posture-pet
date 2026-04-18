import cv2
import mediapipe as mp
import numpy as np
from collections import deque
import time

mp_pose = mp.solutions.pose
mp_selfie = mp.solutions.selfie_segmentation
mp_drawing = mp.solutions.drawing_utils

class OneEuroFilter:
    def __init__(self, min_cutoff=1.5, beta=0.1, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev, self.dx_prev, self.t_prev = None, 0.0, None

    def _alpha(self, cutoff, dt):
        tau = 1.0 / (2 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t=None):
        t = time.time() if t is None else t
        if self.t_prev is None:
            self.t_prev, self.x_prev = t, x
            return x
        dt = t - self.t_prev
        if dt <= 0: return self.x_prev
        dx = (x - self.x_prev) / dt
        edx = self.dx_prev + self._alpha(self.d_cutoff, dt) * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * abs(edx)
        x_filtered = self.x_prev + self._alpha(cutoff, dt) * (x - self.x_prev)
        self.t_prev, self.x_prev, self.dx_prev = t, x_filtered, edx
        return x_filtered

class ClinicalPostureEngine:
    def __init__(self):
        self.filters = [OneEuroFilter() for _ in range(33 * 3)]
        self.baselines = {'head_angle': 0, 'lean': 0, 'slump': 0}
        self.is_calibrated = False
        self.sensitivity = 1.0
        self.alert_start = None
        self.score_history = deque(maxlen=300)

    def filter_landmarks(self, landmarks):
        for i, lm in enumerate(landmarks):
            lm.x = self.filters[i*3](lm.x)
            lm.y = self.filters[i*3+1](lm.y)
            lm.z = self.filters[i*3+2](lm.z)
        return landmarks

    def get_clinical_metrics(self, landmarks):
        l_sh = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        r_sh = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        l_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value]
        r_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]
        nose = landmarks[mp_pose.PoseLandmark.NOSE.value]

        mid_sh = np.array([(l_sh.x + r_sh.x)/2, (l_sh.y + r_sh.y)/2])
        mid_hip = np.array([(l_hip.x + r_hip.x)/2, (l_hip.y + r_hip.y)/2])
        sh_width = abs(l_sh.x - r_sh.x)
        
        # Distance Gate: Ignore background
        if sh_width < 0.15 or l_sh.visibility < 0.7: 
            return None

        head_vec = np.array([nose.x - mid_sh[0], nose.y - mid_sh[1]])
        head_angle = np.abs(np.degrees(np.arctan2(head_vec[1], head_vec[0])))
        shoulder_diff = abs(l_sh.y - r_sh.y) / (sh_width + 1e-6)
        torso_height = (mid_hip[1] - mid_sh[1]) / (sh_width + 1e-6)

        return {'head_angle': head_angle, 'lean': shoulder_diff, 'slump': torso_height}

    def calibrate(self, metrics):
        if metrics:
            self.baselines = metrics
            self.is_calibrated = True
            print(f"Clinically Calibrated: {self.baselines}")

    def evaluate(self, current):
        if not self.is_calibrated:
            return "READY TO CALIBRATE", (200, 200, 200), {}

        head_dev = abs(current['head_angle'] - self.baselines['head_angle']) / 10.0
        lean_dev = current['lean'] / 0.1
        slump_dev = (self.baselines['slump'] - current['slump']) / (self.baselines['slump'] * 0.15)
        
        scores = {
            'HEAD': max(0, head_dev * self.sensitivity),
            'LEAN': max(0, lean_dev * self.sensitivity),
            'SLUMP': max(0, slump_dev * self.sensitivity)
        }
        
        max_score = max(scores.values())
        self.score_history.append(max_score)

        if max_score > 1.0:
            if self.alert_start is None: self.alert_start = time.time()
            if time.time() - self.alert_start > 1.0:
                worst_key = max(scores, key=scores.get)
                return f"ISSUE: {worst_key}", (0, 0, 255), scores
            return "CHECKING...", (0, 165, 255), scores
        
        self.alert_start = None
        return "POSTURE: GOOD", (0, 255, 0), scores

def draw_trend_graph(image, history):
    if not history: return
    cv2.rectangle(image, (350, 10), (630, 110), (0, 0, 0), -1)
    pts = [[350 + int((i/300)*280), 100 - int(min(1.0, s)*70)] for i, s in enumerate(history)]
    if len(pts) > 1: cv2.polylines(image, [np.array(pts)], False, (0, 255, 255), 1)
    cv2.line(image, (350, 30), (630, 30), (0, 0, 150), 1)

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened(): return

    engine = ClinicalPostureEngine()
    mode = 1 # 0: Lite, 1: Balanced, 2: Heavy
    
    def get_pose_model(complexity):
        try:
            return mp_pose.Pose(model_complexity=complexity, smooth_landmarks=True)
        except Exception as e:
            print(f"Error loading model {complexity}: {e}. Falling back to Balanced.")
            return mp_pose.Pose(model_complexity=1, smooth_landmarks=True)

    pose = get_pose_model(mode)
    segmenter = mp_selfie.SelfieSegmentation(model_selection=1)

    print("--- Posture Monitor Started ---")
    print("Press 'P' to toggle Performance Modes")
    print("Press 'C' to Calibrate")
    print("Press 'Q' to Quit")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        
        # 1. HARD BACKGROUND REMOVAL (Locked to Person)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results_seg = segmenter.process(frame_rgb)
        
        # Convert mask to binary
        mask = (results_seg.segmentation_mask > 0.4).astype(np.uint8) * 255
        
        # Apply mask to frame (Hard Blackout)
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

        # 2. Pose Detection on blacked-out frame
        image_rgb = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2RGB)
        results_pose = pose.process(image_rgb)
        
        status, color, scores = "SEARCHING...", (100, 100, 100), {}

        if results_pose.pose_landmarks:
            lms = engine.filter_landmarks(results_pose.pose_landmarks.landmark)
            metrics = engine.get_clinical_metrics(lms)
            if metrics:
                status, color, scores = engine.evaluate(metrics)
                mp_drawing.draw_landmarks(masked_frame, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            else:
                status = "USER IGNORED (BACK)"

        # UI Overlays
        cv2.rectangle(masked_frame, (10, 10), (330, 160), (0, 0, 0), -1)
        cv2.putText(masked_frame, status, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        if engine.is_calibrated and scores:
            for i, (key, val) in enumerate(scores.items()):
                y = 80 + i*25
                cv2.rectangle(masked_frame, (80, y-10), (280, y), (50, 50, 50), -1)
                cv2.rectangle(masked_frame, (80, y-10), (80 + int(min(1.0, val) * 200), y), color if val > 1.0 else (0, 255, 0), -1)
                cv2.putText(masked_frame, key, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        draw_trend_graph(masked_frame, engine.score_history)
        
        mode_text = ["LITE (Laptop)", "BALANCED", "HEAVY (Ryzen)"][mode]
        cv2.putText(masked_frame, f"Mode: {mode_text} | Sens: {engine.sensitivity:.1f}x", (20, 150), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        cv2.imshow('Pro Posture Monitor', masked_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('c') and results_pose.pose_landmarks:
            m = engine.get_clinical_metrics(results_pose.pose_landmarks.landmark)
            if m: engine.calibrate(m)
        elif key == ord('p'):
            mode = (mode + 1) % 3
            pose.close()
            pose = get_pose_model(mode)
            print(f"Switched to Mode {mode}")
        elif key == ord('+') or key == ord('='): engine.sensitivity += 0.1
        elif key == ord('-') or key == ord('_'): engine.sensitivity = max(0.1, engine.sensitivity - 0.1)

    cap.release()
    pose.close()
    segmenter.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
