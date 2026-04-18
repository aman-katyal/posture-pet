import cv2
import mediapipe as mp
import numpy as np

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

# Calculates the internal angle of three points (used for side-view hunching)
def calculate_3pt_angle(a, b, c):
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    if angle > 180.0:
        angle = 360 - angle
    return angle

# Calculates the deviation from vertical (used for front-view side-to-side slouching)
def calculate_vertical_deviation(top_point, bottom_point):
    top, bottom = np.array(top_point), np.array(bottom_point)
    # Calculate angle of the line relative to the X-axis
    radians = np.arctan2(top[1] - bottom[1], top[0] - bottom[0])
    angle = np.abs(radians * 180.0 / np.pi)
    # A perfectly vertical line is 90 degrees. Subtract 90 to get the deviation.
    deviation = np.abs(angle - 90.0)
    return deviation

def main():
    cap = cv2.VideoCapture(1)
    
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break
            
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = pose.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            try:
                landmarks = results.pose_landmarks.landmark
                
                # 1. Grab Core Landmarks
                l_sh = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
                r_sh = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
                l_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP.value]
                r_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]
                l_ear = landmarks[mp_pose.PoseLandmark.LEFT_EAR.value]
                r_ear = landmarks[mp_pose.PoseLandmark.RIGHT_EAR.value]

                # 2. Determine Camera View (Front vs Side)
                shoulder_width = abs(l_sh.x - r_sh.x)
                mid_shoulder_y = (l_sh.y + r_sh.y) / 2
                mid_hip_y = (l_hip.y + r_hip.y) / 2
                torso_length = abs(mid_hip_y - mid_shoulder_y)
                
                # If apparent width is less than half the torso length, they are turned sideways
                is_front_view = (shoulder_width / torso_length) > 0.5

                status = "UNKNOWN"
                color = (255, 255, 255)
                display_angle = 0
                view_text = "FRONT" if is_front_view else "SIDE"

                # 3. Handle Front View (Lateral Lean)
                if is_front_view:
                    mid_shoulder = [(l_sh.x + r_sh.x) / 2, (l_sh.y + r_sh.y) / 2]
                    mid_hip = [(l_hip.x + r_hip.x) / 2, (l_hip.y + r_hip.y) / 2]
                    
                    deviation = calculate_vertical_deviation(mid_shoulder, mid_hip)
                    display_angle = deviation
                    
                    # Threshold for leaning (e.g., leaning more than 10 degrees off vertical)
                    if deviation > 10.0:
                        status = "LATERAL SLOUCH"
                        color = (0, 0, 255)
                    else:
                        status = "GOOD POSTURE"
                        color = (0, 255, 0)

                # 4. Handle Side View (Hunching / Forward Head)
                else:
                    # Determine which side is facing the camera using Z-depth
                    # Negative Z means closer to the camera
                    if l_sh.z < r_sh.z:
                        view_text = "SIDE (LEFT VIEW)"
                        ear = [l_ear.x, l_ear.y]
                        shoulder = [l_sh.x, l_sh.y]
                        hip = [l_hip.x, l_hip.y]
                    else:
                        view_text = "SIDE (RIGHT VIEW)"
                        ear = [r_ear.x, r_ear.y]
                        shoulder = [r_sh.x, r_sh.y]
                        hip = [r_hip.x, r_hip.y]

                    angle = calculate_3pt_angle(ear, shoulder, hip)
                    display_angle = angle
                    
                    # Threshold for hunching (adjust based on camera angle)
                    if angle < 150.0:
                        status = "FORWARD SLOUCH"
                        color = (0, 0, 255)
                    else:
                        status = "GOOD POSTURE"
                        color = (0, 255, 0)

                # 5. Render the UI
                cv2.putText(image, f'View: {view_text}', (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, f'Angle/Dev: {int(display_angle)} deg', (20, 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, status, (20, 130), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3, cv2.LINE_AA)

            except Exception as e:
                # Triggers if a landmark is missing from the frame
                pass 
            
            # Draw Skeleton
            if results.pose_landmarks:
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            
            cv2.imshow('Dynamic Posture Tracker', image)
            
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()