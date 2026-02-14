# SPEC-1-M5-DoorSensor

## Background

The M5-DoorSensor is a security-grade, battery-powered door monitoring device built exclusively on the M5StickC Plus 2 (ESP32-PICO-V3-02 + MPU6886 + ST7789V2 display). The device operates in a LAN-only environment and supports:

* BLE as the primary push-based communication channel
* Optional WiFi with equivalent push semantics
* Multiple listeners (PC + phones)
* High reliability (OPEN events must not be missed)
* No additional hardware sensors

The device must operate solely from the internal LiPo battery and be scalable to large deployments.

The display is diagnostic-only and must not interfere with sensing, communication timing, or power budget.

---

## 2. Hardware Specifications

**Target Device:** M5StickC Plus 2 (ESP32-PICO-V3-02)

| Component | Specification | Notes |
| :--- | :--- | :--- |
| **MCU** | ESP32-PICO-V3-02 | Dual-core 240MHz, 2MB PSRAM, 8MB Flash, WiFi/BLE |
| **Display** | 1.14" TFT LCD (135x240) | ST7789V2 Driver (SPI) |
| **IMU** | MPU6886 | 6-Axis (Accel + Gyro) |
| **Battery** | 200mAh LiPo @ 3.7V | Voltage divider on GPIO 38 |
| **Buttons** | 3 x Tactile (A, B, C) | GPIO 37 (A), GPIO 39 (B), Power/C |
| **LED** | Red (GPIO 19), Green (Power) | Red LED shared with IR |
| **RTC** | BM8563 | I2C Real-time clock |
| **Buzzer** | Passive Piezo | GPIO 2 |
| **Mic** | SPM1423 | PDM Microphone (GPIO 0/34) |
| **USB** | Type-C (CH9102) | Programming & Charging |
| **Expansion** | GROVE Port | I2C + I/O + UART |

### Key Improvements over M5StickC Plus
* **Chip:** Upgraded to ESP32-PICO-V3-02 (from D4)
* **Battery:** Increased to 200mAh (from 120mAh)
* **Memory:** Increased to 8MB Flash + 2MB PSRAM (from 4MB Flash)
* **UART Chip:** CH9102
* **Button Count:** 3 (from 2)

---

## Requirements

### Must Have (M)

* IMU-based OPEN/CLOSED detection (no external sensors)
* Calibration for BOTH OPEN and CLOSED reference positions
* Security-grade reliability (no missed OPEN events)
* Push-based event model
* BLE encrypted broadcast (primary)
* Optional WiFi push mode
* Event persistence across sleep cycles
* Monotonic event ID
* Heartbeat mechanism
* Low battery detection
* Unique device ID per unit
* Battery-powered operation (optimized sleep strategy)

### Should Have (S)

* Encrypted BLE payload (AES-CCM)
* WiFi MQTT QoS1 confirmation
* BLE provisioning mode
* Configurable thresholds and heartbeat interval
* OTA updates (WiFi mode)

### Could Have (C)

* Home Assistant auto-discovery (WiFi)
* Local ring buffer for event logging
* Tamper/motion anomaly detection

### Won’t Have (W)

* External sensors
* Cloud connectivity
* Continuous display usage

---

## Method

### 1. System Architecture

Modules:

1. Sensor Engine (IMU sampling + filtering)
2. State Engine (decision + hysteresis + debounce)
3. Event Engine (ID generation + persistence)
4. BLE Broadcaster (primary channel)
5. Optional WiFi Publisher
6. Power Manager (sleep state machine)
7. Provisioning Service (BLE GATT)
8. Display Module (isolated diagnostic layer)

Strict rule:
Display runs on lowest priority task and never blocks sensing or transmission.

---

### 2. Door Detection Algorithm (Security-Grade)

#### 2.1 Calibration Structure

Stored in NVS and mirrored in RTC:

* open_ref[3]
* closed_ref[3]
* threshold
* calibration_timestamp

#### 2.2 Sampling Strategy

Wake interval: 200 ms (configurable)

Steps:

1. Wake from Light Sleep
2. Delay 5 ms (IMU stabilization)
3. Discard first sample
4. Collect 3 samples @ 10 ms interval
5. Average and normalize vector

#### 2.3 State Decision

Compute Euclidean distance to both references.

If:

* d_open < threshold → OPEN
* d_closed < threshold → CLOSED
* else → MOVING

Security enhancements:

* Require 3 consecutive confirmations
* 300 ms stabilization window before commit
* Hysteresis: threshold_open != threshold_closed

---

### 3. Event Model

Event Structure (binary packed ≤ 24 bytes):

* DeviceID (4B)
* EventID (4B)
* EventType (1B)
* Battery_mV (2B)
* Timestamp (4B)
* Nonce (4B)
* AuthTag (8B truncated)

Event Types:

* 0x01 OPEN
* 0x02 CLOSED
* 0x03 HEARTBEAT
* 0x04 BATTERY_LOW

Event Flow:

1. Event generated
2. Stored in RTC buffer
3. Checkpointed to NVS every 100 events or 1 hour
4. Transmitted

WiFi mode waits for ACK before clearing event.
BLE mode uses encrypted broadcast (stateless).

---

### 4. BLE Architecture (Primary)

Mode: Non-connectable encrypted advertisement

Behavior:

* On event → 300 ms burst @ 100 ms interval
* Heartbeat → every 5 minutes

Encryption:

* AES-128 CCM
* Pre-shared network key
* Rolling nonce

Advantages:

* Unlimited listeners
* No connection limits
* Low latency (<300 ms typical)

Limitation:
Payload must fit within 31-byte legacy advertisement.

---

### 5. Optional WiFi Mode

Disabled by default.

On event:

1. Enable WiFi
2. Connect to AP
3. Publish via MQTT QoS1
4. Wait for PUBACK
5. Disconnect WiFi

WiFi also enables OTA.

BLE and WiFi never active simultaneously.

---

### 6. Power Strategy

Default Mode: Light Sleep

State Machine:

LightSleep → Wake → Measure → Evaluate →
If Event → Transmit → LightSleep
Else → LightSleep

Power Rules:

* Display always OFF unless button pressed
* WiFi radio disabled unless transmitting
* BLE controller enabled only during burst
* IMU powered only during sampling

Target:

* BLE-only mode: 3+ months
* WiFi-enabled mode: 1–2 months

Deep Sleep may be introduced after stability validation.

---

### 7. Persistence Strategy

RTC_DATA_ATTR:

* Last state
* Event counter
* Last event pending

NVS:

* Calibration
* Network key
* WiFi credentials
* Event counter checkpoint

Flash writes minimized via batching.

---

### 8. Display Subsystem Rules (ST7789V2)

* Full reinitialization after deep sleep
* Backlight forced LOW on boot
* No IMU sampling during SPI updates
* No framebuffer allocation >10 KB
* Display never auto-wakes on event

Used only for:

* Calibration confirmation
* Battery level
* IP display (WiFi mode)

---

## Implementation Plan

Phase 1 – Stable Sensing Core

* IMU sampling + filtering
* Dual-reference calibration
* Hysteresis + debounce
* 7-day stability test

Phase 2 – Secure BLE Broadcast

* Binary packed payload
* AES-CCM encryption
* Multi-device listener validation

Phase 3 – Event Persistence Hardening

* RTC + NVS checkpointing
* Brownout handling
* Cold boot recovery

Phase 4 – Optional WiFi

* MQTT QoS1
* ACK validation
* OTA update support

Phase 5 – Power Optimization

* Current profiling
* Sleep tuning
* Battery endurance test

---

## Milestones

1. No false toggles for 72 hours
2. 0 missed OPEN events in stress test
3. Stable BLE reception at 10 devices
4. 30-day battery endurance validation
5. OTA update successful over WiFi

---

## Gathering Results

Validation Metrics:

* Event latency < 300 ms (BLE)
* No duplicate EventIDs
* No flash corruption after 1000 events
* Battery drain < 1% per day (BLE-only idle)
* Stable operation with 20+ simultaneous devices

If approved, next step options:

1. Produce exact ESP-IDF project structure
2. Define AES key management model
3. Simulate worst-case power budget
4. Design listener-side PC application architecture
