# AI Posture Monitor (Pro Edition)

A high-performance, clinical-grade posture detection system using MediaPipe Pose and Selfie Segmentation. Optimized for both laptops and high-end Ryzen AI PCs.

## 🚀 Key Features

- **Clinical Metrics:** Tracks Craniovertebral Angle (Head), Shoulder Symmetry (Lean), and Torso Height (Slump).
- **Hard Background Removal:** Uses AI segmentation to black out everything except the primary user, physically preventing the system from detecting background people.
- **Multi-Mode Engine:** Toggle between LITE, BALANCED, and HEAVY models on the fly to match your hardware power.
- **Zero-Lag Filtering:** Implements the One Euro Filter for butter-smooth, real-time tracking without "floaty" delays.
- **Analytics Dashboard:** Real-time stress bars and a 10-second history trend graph.

## 🛠️ Setup & Usage

### 1. Installation
Ensure you have Python 3.12+ installed.
```powershell
cd cam
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Running the Monitor
```powershell
python detector.py
```

### 3. Controls
| Key | Action |
| :--- | :--- |
| **C** | **Calibrate:** Sit in your best posture and press C to set baselines. |
| **P** | **Performance Toggle:** Cycle through LITE (Laptop), BALANCED, and HEAVY (Ryzen) models. |
| **+ / -** | **Sensitivity:** Adjust how harsh the posture alerts are. |
| **Q** | **Quit:** Safely close the application. |

## 📐 How it Works
The system uses normalized medical heuristics. Instead of absolute angles, it measures ratios relative to your shoulder width. This ensures detection remains accurate even if you move closer or further from the camera.
