# 🚁 ESP32 Flight Controller

Advanced quadcopter flight controller firmware featuring comprehensive sensor integration, low‑latency RF control, high‑precision telemetry (dev), and extensible stabilization architecture.

[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Framework](https://img.shields.io/badge/framework-Arduino-green.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-Dev%20Stabilization%20Phase-orange.svg)](docs/PROJECT_PROGRESS.md)

---

## 🆕 August 18, 2025 – Latest Development Highlights

| Area                | Update                                                                                                          | Impact                                      |
| ------------------- | --------------------------------------------------------------------------------------------------------------- | ------------------------------------------- |
| Control Latency     | Radio polling tightened to ~2ms (≈500Hz) + NRF24 @1Mbps                                                         | Lower stick→motor delay, reduced jitter     |
| IMU Orientation     | 90° CW Z-axis rotation compensated (axis remap + bias transform)                                                | Correct roll/pitch alignment, drift removed |
| GPS Precision (Dev) | latitudeE7 / longitudeE7 (int32 *1e7) replacing int16 *100                                                      | ~1 cm positional resolution (dev builds)    |
| PID Controller      | Integral gating, leak, derivative filtering, setpoint smoothing, yaw feed-forward, optional cascaded Angle→Rate | Smoother attitude & crisper yaw             |
| Mixer               | Unified dual‑sided dynamic scaling across RF & WiFi builds                                                      | Identical tuning cross-build                |
| Logging             | Clarified status print throttling vs high‑rate loops                                                            | Prevents false latency conclusions          |
| Equivalence Audit   | Post-PID PWM parity verified between transport variants                                                         | Cross-platform confidence                   |

Temporary Dev vs Stable Telemetry divergence:

| Field        | Stable (legacy)      | Development (Aug 18)       |
| ------------ | -------------------- | -------------------------- |
| Latitude     | int16 \*100 (≈0.01°) | int32 latitudeE7 (° \*1e7) |
| Longitude    | int16 \*100          | int32 longitudeE7          |
| Packet Size  | 22 bytes             | 30 bytes                   |
| Version Byte | (none)               | Planned (before promotion) |

Stable retains legacy packet until a version byte enables backward-compatible upgrade.

See also: `docs/PROJECT_PROGRESS.md` & `docs/CURRENT_STATUS_AUGUST_18.md`.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#✨-features)
- [Project Structure](#project-structure)
- [Hardware Specifications](#-hardware-specifications)
- [Pin Configuration](#-pin-configuration)
- [Communication Protocols](#-communication-protocols)
- [Power Management](#-power-management)
- [Status Indication](#-status-indication-system)
- [System Architecture](#-system-architecture)
- [Flight Control Modes](#-flight-control-modes)
- [Installation](#-installation)
- [Usage](#-usage)
- [Development Status](#-development-status)
- [Directory Navigation](#-directory-navigation)
- [Contributing](#-contributing)
- [License](#-license)

## Overview

ESP32-based quadcopter controller featuring multi-rate FreeRTOS tasking, advanced dynamic motor mixing, modular telemetry, and progressive stabilization features suitable for experimentation and extension.

## Project Structure

```
esp32-flight-controller/
├── firmware/                 # Stable & development firmware
│   ├── stable/               # Production-ready (legacy telemetry packet)
│   │   ├── drone/
│   │   └── remote/
│   └── development/          # Active development (high precision GPS, advanced PID)
│       ├── drone/
│       └── remote/
├── docs/                     # Technical & status documentation
├── examples/                 # Sensor / component tests
├── tools/                    # Utilities (web dashboard, etc.)
├── hardware/                 # Hardware references (in progress)
├── archive/                  # Legacy & deprecated versions
├── src/                      # PlatformIO src (main.cpp)
└── platformio.ini            # Build configuration
```

## ✨ Features

### 🎮 RF Remote Control System

- Dual analog joystick inputs (throttle/yaw + pitch/roll)
- X-configuration motor mixing with dual‑sided dynamic scaling (no pre-reserved headroom)
- ~5Hz remote transmission; firmware side polls radio ~500Hz for ultra‑low latency application
- Toggle-based arming & flight mode selection
- ACK payload telemetry return path

### 🧠 Advanced Flight Control

- 50Hz PID + motor loop, 100Hz IMU sampling, ~500Hz radio polling
- Immediate ESC calibration & enforced PWM range
- Safety: arming interlocks, control timeout, sensor freshness checks
- PID features: integral gating (throttle-based), integrator leak, derivative filtering, setpoint smoothing, yaw feed-forward, optional cascaded Angle→Rate (experimental)
- Unified mixer: Full 1000–2000 µs throttle span; dual-scale factors (s_pos / s_neg) clamp only the saturated side
- Altitude correction overlay (prototype): ±300 µs additive window auto-scaled in mixer
- Manual + Stabilized modes (Altitude Hold experimental)

### 📡 Comprehensive Sensor Suite

- Environmental: Pressure, humidity, temperature, air quality
- Navigation: GPS (high precision in dev), IMU (90° rotation compensated), barometric altitude
- Obstacle: Up to 4x VL53L0X via I2C multiplexer
- Light & UV sensing
- Battery voltage monitoring

### 🌐 Connectivity & Telemetry

- NRF24L01+ PA+LNA (1 Mbps dev / range-focused lower rates stable)
- WiFi (dashboard / alternate control experiments)
- Telemetry: legacy int16\*100 or dev high-precision latitudeE7/longitudeE7
- Planned: versioned telemetry framing

## 🧭 Flight Mode System

- Manual Mode: Direct joystick inputs → mixer
- Stabilized Mode: PID-stabilized roll/pitch/yaw through identical mixer path
- Altitude Hold: Barometric additive correction (prototype)
- Seamless in-flight mode switching

## 🔧 Hardware Specifications

| Component       | Model                   | Purpose                |
| --------------- | ----------------------- | ---------------------- |
| Microcontroller | ESP32 (dual-core)       | Flight control & tasks |
| RF Module       | NRF24L01+ PA+LNA        | Control & telemetry    |
| GPS             | NEO-6M                  | Positioning            |
| IMU             | MPU6050                 | Attitude sensing       |
| Baro/Env        | BME280 / AHT21 / ENS160 | Altitude & environment |
| Light           | BH1750                  | Ambient light          |
| UV              | GUVA-S12SD              | UV index               |
| Distance        | VL53L0X (multiplexed)   | Obstacle ranging       |

## 📌 Pin Configuration (Excerpt)

See prior revisions for full mapping; representative connections retained (SPI: CE4 CSN5 SCK18 MOSI23 MISO19, I2C: SDA21 SCL22, GPS UART RX16).

## 📡 Communication Protocols

- NRF24L01+: Channel 76, dynamic poll architecture; dev uses 1 Mbps for latency
- I2C: Standard 100kHz (multiplexed for VL53L0X set)
- WiFi: Station mode (dashboard/testing)

## 🔋 Power Management

- 3S LiPo monitored via precision divider
- Filtered 3.3V rail for RF module and sensors
- ESC supply isolated from logic noise paths

## 🔔 Status Indication System

- Navigation LEDs (port/starboard + orientation)
- White status LEDs (arming / mode)
- Buzzer alerts (boot, errors, arming confirmation)

## 🏗️ System Architecture

### FreeRTOS Task Structure (Representative – Dev Build)

| Task              | Core | Priority | Frequency                   | Purpose                     |
| ----------------- | ---- | -------- | --------------------------- | --------------------------- |
| PID / Motor Loop  | 1    | 4 (High) | 50Hz                        | Stabilization + ESC PWM     |
| Radio Poll        | 1    | 3        | ~500Hz                      | Low-latency packet handling |
| IMU / Sensor Read | 0    | 2        | 100Hz IMU / mixed env rates | Orientation & environment   |
| Status Updates    | 0    | 1        | 0.1–1Hz                     | Throttled diagnostics       |

### Flight Control Logic (Conceptual)

```
Joysticks → Radio → (Poll) → Control Task → PID (Stabilized) → Dynamic Mixer → ESC PWM
                                 ↑                │
                              Sensors (IMU, Baro, GPS dev)
```

### Current Implementation

- Manual Mode: Direct joystick → mixer (dynamic scaling)
- Stabilized Mode: PID outputs share identical mixer for consistent feel
- Altitude Hold (prototype): Barometric ±300 µs additive throttle offset
- Dynamic Scaling (X config):
  - Raw mixes: M1 = -kR R - kP P + kY Y; M2 = -kR R + kP P - kY Y; M3 = +kR R - kP P - kY Y; M4 = +kR R + kP P + kY Y
  - posMax / negMin → s_pos = min(1, (PWM_max - T)/posMax), s_neg = min(1, (T - PWM_min)/-negMin)
  - Side-specific scaling preserves lift & proportionality

### Current PID & Stabilization Features (Dev)

- Complementary attitude fusion (Kalman optional)
- Roll/Pitch/Yaw PID with throttle-gated integral & leak
- Yaw feed-forward for improved yaw authority
- Optional cascaded Angle→Rate inner loop (experimental)
- Altitude hold (barometric additive correction – prototype)

### Planned Enhancements

- Telemetry versioning (promote high-precision GPS to stable)
- Yaw drift mitigation (magnetometer / adaptive bias)
- Position / altitude hybrid hold (baro + GPS fusion)
- High-rate lightweight debug streaming channel

## 🛠️ Installation

1. Install PlatformIO (preferred) or Arduino IDE
2. Clone repo & open root folder
3. Select desired firmware (stable vs development) under `firmware/`
4. Build & upload with PlatformIO tasks or CLI
5. Verify radio & sensor output over serial before arming

## 🚀 Usage

1. Power system (ESCs calibrate automatically)
2. Connect remote; confirm telemetry returns
3. Arm via SW1 (throttle low + toggle)
4. Fly in Manual or Stabilized; engage experimental altitude hold if enabled

Remote Controls:

- Left Stick: Throttle (Y) / Yaw (X)
- Right Stick: Pitch (Y) / Roll (X)
- SW1: ARM / DISARM
- SW2: Flight Mode (Manual / Stabilized)

Safety:

- Always arm on a level, clear surface
- Observe control timeout; system disarms on link loss
- Monitor battery to avoid over-discharge

## 📊 Development Status

### ✅ Completed Features

- Low-latency RF control path & mixer parity
- Unified dynamic scaling motor mixer
- Sensor integration (IMU, baro, GPS, env, light, ToF, battery)
- ESC calibration & safety systems
- High-precision GPS telemetry (dev builds)
- Advanced PID feature set (dev)

### 🔄 In Progress

- Telemetry versioning & GPS precision promotion
- Yaw drift handling & heading stabilization
- Position / altitude hold research

### 📋 Planned Features

- GPS waypoint navigation (after position hold)
- Enhanced telemetry dashboard & streaming
- Autonomous flight capabilities (long-term)
- Mobile / companion app integration

For detailed progress tracking see `docs/PROJECT_PROGRESS.md`.

## 🗂️ Directory Navigation

- **🚁 firmware/** – Stable & development firmware
  - ✅ stable/drone/ – Production-ready drone firmware
  - ✅ stable/remote/ – Production-ready remote firmware
  - 🔬 development/drone/ – Active dev (precision GPS, PID)
  - 🔬 development/remote/ – Experimental features
- **📚 docs/** – Technical documentation & status snapshots
- **🔬 examples/** – Sensor & component test programs
- **🛠️ tools/** – Utilities (e.g., web dashboard)
- **📦 archive/** – Legacy & deprecated implementations

## 🤝 Contributing

Contributions welcome. Please:

- Follow Arduino / ESP32 style conventions
- Test thoroughly
- Update documentation for user-facing changes
- Maintain backward compatibility where feasible

## 📄 License

MIT License – see `LICENSE`.

## 🙏 Acknowledgments

- ESP32 & Arduino communities
- FreeRTOS ecosystem
- Open-source drone development community

---

**⚠️ Safety Notice**: Experimental flight controller software. Test in controlled environments. Keep clear of props. Proceed responsibly.
