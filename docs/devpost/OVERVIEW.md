# Project Overview: Posture Pet

## Project Description
Posture Pet is a context-aware tracking platform that fuses computer vision with wearable sensors to help users improve their physical health.

## The Problem
Many posture trackers are limited by their hardware. Camera-based systems lose tracking when a user walks away, and simple wearables often can't distinguish between a relaxed lean and a harmful slouch. We built Posture Pet to bridge this gap.

## The Solution
Our system uses a dual-brain architecture:
1.  **Kinetic Brain**: A wearable array of three MPU-6500 sensors that tracks relative body orientation in 3D space.
2.  **Visual Brain**: A PC-based computer vision engine that provides a reference for skeletal alignment.
3.  **The Hub**: An Arduino Uno Q that coordinates these two sources. It runs AI inference on its Qualcomm chip and provides physical feedback through an STM32-controlled robotic companion.

## Key Features
*   **Dual-Source Validation**: The system cross-verifies IMU data against visual tracking for high accuracy.
*   **Digital Twin**: A real-time 3D visualization that mirrors the user's spinal alignment.
*   **Physical Feedback**: A physical robot that mimics the user's posture and uses an OLED display to show emotional expressions based on how well you are sitting.
