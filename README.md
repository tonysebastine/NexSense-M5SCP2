# NexSense M5-DoorSensor

A professional-grade, battery-optimized door sensor firmware for the **M5StickC Plus 2**. 

This firmware utilizes the internal IMU (MPU6886) to detect door orientation changes and broadcasts encrypted BLE advertisements to alert your home automation system.

## ✨ Features

- **🚀 Ultra-Low Power**:
  - **Dynamic Frequency Scaling (DFS)**: CPU scales from 40MHz up to 240MHz.
  - **Adaptive Light Sleep**: High-efficiency sleep cycles (up to 1000ms polling).
  - **Sensor Optimization**: MPU6886 operates in 10Hz low-power mode when idle.
- **🛡️ Secure Communication**:
  - **AES-128 CCM Authentication**: Prevents spoofing and replay attacks.
  - **Encrypted BLE Broadcasts**: Compatible with standard BLE scanners/receivers.
- **🎯 Precise Detection**:
  - **6-Axis Vector Analysis**: Highly reliable door state detection.
  - **Gyro-Assisted Calibration**: Smooth, stable setup process.
- **💻 Modern UI**:
  - **"Cyber-Deck" Aesthetic**: High-contrast status display.
  - **Silent Operation**: Minimal LED/Buzzer usage (reserved for system alerts).

## 🛠️ Hardware Requirements

- **Device**: M5StickC Plus 2 (ESP32-PICO-V3-02)
- **Sensor**: Internal MPU6886
- **Power**: Internal 200mAh Battery (USB-C rechargeable)

## 📦 Installation

1. **Install ESP-IDF**: Ensure you have ESP-IDF v5.5.2 or later installed.
2. **Clone the Repo**:
   ```bash
   git clone https://github.com/tonysebastine/NexSense-M5SCP2.git
   cd NexSense-M5SCP2
   ```
3. **Build & Flash**:
   ```bash
   idf.py build
   idf.py -p [PORT] flash monitor
   ```

## 🎮 Usage

- **Boot**: Shows status, calibration state, and device ID.
- **Calibration**:
  - Hold **Button A** (Front) for 2 seconds to enter calibration.
  - Follow on-screen prompts for **OPEN** and **CLOSED** positions.
- **Wake Display**: Short press **Button B** (Side) to view current status.
- **Low Battery**: The red LED will double-blink every 10 seconds when battery is low.

## 📄 License

MIT License. See [LICENSE](LICENSE) for details.
