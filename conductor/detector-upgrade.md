# Posture Detector Upgrade Plan

## Objective
Upgrade the webcam posture detection system (`cam/detector.py`) to use a more robust, multi-metric pipeline that detects both forward head posture and slumping, and add real-time strictness tuning.

## Proposed Solution
The current simple angle-based detection is brittle and depends heavily on camera placement. We will replace it with a composite metric approach based on normalized vertical drops.

### Key Metrics
1. **Neck Drop Ratio (Forward Head)**: Measures the vertical distance from the nose to the shoulders, normalized by shoulder width. Looking down or hunching forward causes this ratio to shrink.
2. **Torso Drop Ratio (Slumping)**: Measures the vertical distance from the shoulders to the hips, normalized by shoulder width. Slumping in a chair causes this ratio to shrink.
3. **Lateral Lean**: Measures the vertical difference between the left and right shoulders, normalized by shoulder width.

### Features
* **Calibration**: Pressing 'C' will lock in the baseline ratios for perfect posture.
* **Composite Scoring**: The system calculates the percentage deviation for each metric from the baseline. The worst deviation dictates the overall posture state.
* **Strictness Tuning**: Users can press `+` or `-` (or `=` and `_`) to increase or decrease the strictness multiplier on the fly, directly updating the UI.
* **Diagnostics UI**: The screen will clearly display the active strictness level and the current deviation percentage.

## Implementation Steps
1. Refactor `detector.py` to use a `PostureDetector` class to cleanly encapsulate buffers, baselines, and evaluation logic.
2. Implement the normalized math for `Neck Drop Ratio`, `Torso Drop Ratio`, and `Lateral Lean` using MediaPipe coordinates.
3. Add keyboard listeners for strictness tuning (`+` and `-`).
4. Update the OpenCV UI to display the new metrics, countdown timer, and specific cause of bad posture (e.g., "BAD POSTURE! (FORWARD HEAD)").

## Verification
* Run `python detector.py`.
* Calibrate sitting straight.
* Test hunching forward (should trigger "FORWARD HEAD").
* Test slumping back (should trigger "SLUMPING").
* Press `+` to make it extremely harsh and ensure it triggers instantly on slight movement.
* Press `-` to make it generous and ensure it ignores minor movements.