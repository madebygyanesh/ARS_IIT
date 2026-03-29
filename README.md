# Vibration Measurement System Using MPU6050 & Dual ESP32 Architecture
### Project Code: ARS_IIT

This project implements a wireless vibration monitoring and relay control system using a dual ESP32 architecture. It features a specialized Worker Node for real-time sensing and a Display Node for advanced data visualization and industrial HMI control, communicating via the low-latency **ESP-NOW** protocol.

---

## 🏛️ Project Information

- **Institution:** Anjuman-I-Islam’s Abdul Razzak Kalsekar Polytechnic, Panvel
- **Department:** Automation & Robotics
- **Location:** Panvel, Maharashtra, India
- **Date:** March 2026

### 👨‍🎓 Submitted By:
- Gyanesh Maurya
- Mohammad Ibaad Nadkar
- Md. Ayan Ansari
- Huzaifa Adi

### 👨‍🏫 Project Guides:
- Mr. Imran Rajwani
- Mrs. Naema Shaikh

---

## 🚀 System Specifications
- **Microcontrollers:** ESP32-C3 Mini (Worker) + ESP32 (Display)
- **Sensor:** MPU6050 / MPU6500 (6-Axis IMU)
- **Communication:** ESP-NOW (2.4 GHz, P2P)
- **Display:** 320x240 TFT + XPT2046 Touch
- **Source Code:** [github.com/madebygyanesh/ARS_IIT](https://github.com/madebygyanesh/ARS_IIT)

---

## 🛠️ Hardware Setup

### 1. Worker Node (Sensor Board)
- **Microcontroller:** ESP32-C3 Mini
- **Sensor:** MPU6050 / MPU6500 (6-Axis IMU)
- **Output:** 4-Channel Relay Module (Active-LOW logic)
- **I2C Pins:** SDA → GPIO 8, SCL → GPIO 9
- **Relay Pins:** GPIO 2, 3, 4, 5

### 2. Display Node (Controller Board)
- **Microcontroller:** ESP32 DevKit V1 (38-pin)
- **Display:** 2.8" TFT LCD (320x240, ILI9341 Driver)
- **Touch:** XPT2046 Resistive Touch Controller
- **SPI Pins (TFT):** CS: 14, DC: 27, RST: 26, MOSI: 13, CLK: 12
- **SPI Pins (Touch):** CS: 33, IRQ: 36, MOSI: 32, MISO: 39, CLK: 25

---

## 📂 Repository Structure
```text
.
├── Circuit_Diagram/         # Hardware wiring diagrams (Worker & Display)
├── C3_Worker/               # Source code for ESP32-C3 Worker node
├── ESP32_TFT_DISPLAY/       # Source code for ESP32 Display node
├── libraries/               # Required pre-configured Arduino libraries
└── README.md                # Project documentation
```

---

## 💻 Software Setup & Flashing Guide

### 1. Prerequisites
- **Arduino IDE 2.x**
- **ESP32 Board Package:** Add the following URL to your Board Manager:
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

### 2. Library Installation
All necessary external libraries are included in the `/libraries` folder of this repository:
- `TFT_eSPI` (pre-configured for this project's pinout)
- `XPT2046_Touchscreen`

1. Locate your Arduino libraries folder (usually `Documents/Arduino/libraries`).
2. Copy the folders from this repository's `libraries/` directory into your local libraries folder.

### 3. Step-by-Step Flashing

**Step 1: Flash the Display Node first.**
1. Open `ESP32_TFT_DISPLAY/ESP32_TFT_DISPLAY/ESP32_TFT_DISPLAY.ino`.
2. Connect your **ESP32**.
3. Select Board: **ESP32 Dev Module**.
4. Click **Upload** and open the Serial Monitor at **115200 baud**.
5. On first boot, follow the on-screen **9-point touch calibration**.
6. Navigate to the **Connection Screen** and note the **"My MAC"** address displayed.

**Step 2: Configure and flash the Worker Node.**
1. Open `C3_Worker/Worker/Worker.ino`.
2. Locate the `displayMAC` array at the top of the file:
   ```cpp
   // Replace with the MAC address shown on the Display's Connection screen
   uint8_t displayMAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
   ```
3. Update it with the MAC address you noted from the Display Node.
4. Connect your **ESP32-C3**.
5. Select Board: **ESP32C3 Dev Module** (Set USB CDC on Boot: **Enabled**).
6. Click **Upload**.
7. On boot, the worker will auto-calibrate — **keep the sensor flat and still for 5 seconds**.
