# Posture Pet: Devpost Submission

## Inspiration

The inspiration for this project came from a running joke in our group. One of our members, Aman, is notorious for having terrible posture whenever we sit down to work on homework together. When we started brainstorming for the hackathon, we knew we wanted to build something in the wearable technology space that could actually solve a problem in our daily lives. Aman's posture was the perfect target, so we decided to build a system that wouldn't just track posture, but would make us actually care about fixing it.

## What it does

Posture Pet is a hybrid hardware-AI system that monitors your body alignment and reflects your physical state through a robotic companion. The Pet is a three-block robot that uses two servo axes to physically mimic the user's slouching or leaning. It features an OLED display for eyes that change expressions based on how the user is sitting—becoming happy when you sit upright and showing distress when you slouch.

The system uses a custom wearable array of three MPU-6500 IMU sensors placed at the base of the neck and on both shoulders. This data is transmitted via BLE to an Arduino Uno Q, where an AI model classifies the posture. Simultaneously, a PC-based vision model tracks the user and sends its own classification results to the Uno Q via UDP over WiFi. This allows the system to cross-verify the sensor data against a visual ground truth in real-time.

## How we built it

We built the wearable component using an ESP32-S3 and three MPU-6500 sensors. We implemented a Mahony fusion filter directly on the ESP32 to convert raw motion data into stable quaternions, which are sent over BLE to the Arduino Uno Q. 

The Uno Q acts as the brain of the system. We utilized its dual-architecture by running a TensorFlow Lite inference engine on the Qualcomm Linux core to process the incoming sensor data. For the vision side, we used a PC running a MediaPipe-based model. The PC sends classification packets to the Uno Q using the UDP protocol over WiFi to ensure low-latency communication. The Uno Q compares the two sources and bridges the final decision to the STM32 microcontroller, which handles the real-time movement of the servos and the OLED eye expressions.

## Challenges we ran into

The hardware was a massive hurdle. Soldering three separate IMUs with custom I2C addressing while maintaining bus stability was a constant headache. We also faced significant difficulties with the Arduino Uno Q. Because it is such a new product, there was a major lack of community support and documentation. We spent a lot of time just figuring out the basics of how the Qualcomm and STM32 cores communicate through the bridge, which felt like we were working in the dark at times. Managing the networking between the PC and the Uno Q using UDP also required a lot of trial and error to handle packet drops and latency on the local network.

## Accomplishments that we're proud of

We are particularly proud of getting the AI inference model running smoothly on the Uno Q. Training a model on motion data that can accurately distinguish between body types and sitting styles was difficult, and seeing it make real-time decisions that move the robot is rewarding. We also managed to implement a complex 60-feature extraction engine (using FFT and temporal stats) that makes the system much more robust than a simple threshold-based tracker.

## What we learned

We learned a lot about the realities of working with bleeding-edge hardware. When the documentation doesn't exist, you have to become very comfortable reading source code and experimenting with low-level protocols. We also gained a deep understanding of sensor fusion and how to handle data synchronization between three different processing layers (ESP32, Qualcomm Linux, and STM32) and external network sources.

## What's next for Posture Pet

We want to expand the Pet's health metrics beyond just physical posture. Our next step is to integrate an additional ESP32 node equipped with gas sensors to monitor indoor air quality. In this version, the Pet's survival wouldn't just depend on your back being straight, but also on the environment being healthy. If the CO2 levels get too high, the Pet would start to show distress, prompting the user to open a window—adding another layer to how the companion keeps its owner healthy.
