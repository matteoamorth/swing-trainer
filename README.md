# ⛳ GolfTrainer: Improve Your Golf Skills with Multisensory Feedback

## 🏌️ Overview

GolfTrainer is a smart golf club designed to enhance golf training through multisensory feedback, including visual, auditory, and haptic cues. By integrating advanced sensors and real-time feedback mechanisms, this system helps golfers refine their swing mechanics and improve overall performance.

## 🎯 Features

- **Real-time multisensory feedback**: Provides visual, auditory, and haptic cues to guide users in improving their swing.
- **Advanced sensor integration**: Utilizes a BNO055 motion sensor, ribbon sensors, Hall effect sensors, and a vibration motor to capture and analyze swing data.
- **User-friendly interface**: Displays feedback in a clear and intuitive manner to help both novice and expert golfers.
- **Experimental validation**: Tested through a pilot study comparing multisensory feedback to traditional visual-only methods.

## 🛠️ System Components

- **Microcontroller**: Teensy 4.1
- **Sensors**:
  - BNO055 motion sensor (for orientation tracking)
  - Ribbon sensor (for hand position tracking)
  - Hall effect sensor (for ball position detection)
- **Feedback mechanisms**:
  - 💡 LEDs for visual feedback
  - 🎵 Vibration motor for haptic feedback
  - 🔊 Pure Data (Pd) for audio feedback
- **Software**:
  - 🖥️ Processing (for real-time visualization)
  - 🎼 Pure Data (for sound generation)
  
## 🚀 Installation & Usage

1. **Hardware Setup**:
   - Connect the sensors to the Teensy 4.1 microcontroller as per the wiring diagram.
   - Mount the components onto the golf club.
2. **Software Setup**:
   - Install the required libraries for the BNO055 sensor.
   - Load the Arduino firmware onto the Teensy 4.1.
   - Run the Processing script for real-time visualization.
   - Execute the Pure Data patch for audio feedback.
3. **Using the GolfTrainer**:
   - Power on the system and calibrate the sensors.
   - Practice swings while receiving real-time feedback.
   - Adjust technique based on visual, auditory, and haptic cues.

## ⚠️ Compatibility Disclaimer
GolfTrainer is compatible with Arduino; however, the available pins and interrupt-capable pins must be modified to ensure proper functionality. Users should review the pin configurations and adjust accordingly when integrating with different Arduino models.

## 📊 Evaluation & Results
A pilot study was conducted to assess the system's effectiveness. Participants tested three feedback conditions:
1. **Visual only** 👀
2. **Visual + Sound** 🔊
3. **Visual + Sound + Haptics** 🎵💡

Findings showed that multisensory feedback enhances training efficacy, especially for beginners, by improving proprioceptive awareness and skill acquisition.

## 🔧 Future Improvements

- Refining haptic feedback calibration.
- Enhancing Hall effect sensor stability.
- Expanding study with a larger participant pool.
- Implementing adaptive feedback based on user expertise.

## 👥 Contributors
Special thanks to [Samu01Tech](https://github.com/Samu01Tech) for Audio processing, dashboard implementation, IMU support, data analysis guidelines.

## 🔗 Resources
Visit the project website for additional resources: [GolfTrainer Website](https://golftrainer.netlify.app/)
