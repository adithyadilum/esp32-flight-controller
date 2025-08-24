# 🚁 ESP32 Flight Controller

**Advanced quadcopter flight controller firmware featuring comprehensive sensor integration, RF remote control, and real-time telemetry systems.**

[![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Framework](https://img.shields.io/badge/framework-Arduino-green.svg)](https://www.arduino.cc/)
[![License](htt- **🚁 [firmware/](firmware/)** - Stable and development firmware versions

- **✅ [stable/drone/](firmware/stable/drone/)** - Production-ready drone firmware
- **✅ [stable/remote/](firmware/stable/remote/)** - Production-ready remote firmware
- **🔬 [development/drone/](firmware/development/drone/)** - PID integration development
- **🔬 [development/remote/](firmware/development/remote/)** - Enhanced features developmentimg.shields.io/badge/license-MIT-brightgreen.svg)](LICENSE)
  [![Status](https://img.shields.io/badge/status-Motor%20Control%20Operational-success.svg)](docs/PROJECT_PROGRESS.md)

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Hardware Specifications](#hardware-specifications)
- [Pin Configuration](#pin-configuration)
- [Communication Protocols](#communication-protocols)
- [Installation](#installation)
- [Usage](#usage)
- [System Architecture](#system-architecture)
- [Contributing](#contributing)
- [License](#license)

## 🔍 Overview

This project implements a sophisticated flight controller for a custom quadcopter using the ESP32 microcontroller. The system features advanced sensor fusion, RF remote control, comprehensive telemetry, and FreeRTOS-based multi-threading architecture for real-time flight control operations.

## 📁 Project Structure

```
esp32-flight-controller/
├── 📁 firmware/              # Firmware organized by stability
│   ├── 📁 stable/            # Production-ready firmware
│   │   ├── 📁 drone/         # Stable drone firmware
│   │   │   └── droneFreeRTOS.ino # ✅ Stable production firmware
│   │   └── 📁 remote/        # Stable remote firmware
│   │       ├── remoteControllerStable.ino  # ✅ Stable remote controller
│   │       └── FastControlRemote.ino       # Alternative stable remote
│   └── 📁 development/       # Development firmware
│       ├── 📁 drone/         # Development drone firmware
│       │   └── droneFreeRTOS.ino # 🔬 Development version for PID integration
│       └── 📁 remote/        # Development remote firmware
│           ├── remoteControllerStable.ino  # 🔬 Development remote
│           └── FastControlRemote.ino       # Alternative development remote
│
├── 📁 docs/                  # Project documentation
│   ├── PROJECT_PROGRESS.md   # Development progress tracking
│   ├── PIDControl.md         # PID control documentation
│   └── FLIGHT_MODE_CONTROL.md # Flight mode switching system guide
│
├── 📁 examples/              # Test programs and examples
│   ├── 📁 sensor_tests/      # Individual sensor test programs
│   │   ├── BMP280.cpp        # Environmental sensor testing
│   │   ├── MPU6050.cpp       # IMU sensor testing
│   │   ├── gps_test.cpp      # GPS module testing
│   │   └── ...
│   └── 📁 component_tests/   # System component testing
│       ├── motor_test.cpp    # ESC/motor testing
│       └── Wifi Control.cpp  # WiFi testing
│
├── 📁 tools/                 # Development utilities
│   └── web_dashboard.cpp     # Web-based control interface
│
├── 📁 hardware/              # Hardware specs and diagrams (planned)
│
├── 📁 archive/               # Legacy versions and deprecated code
│   └── 📁 legacy_versions/   # Previous firmware implementations
│
├── 📁 lib/                   # Project-specific libraries
├── 📁 include/               # Header files
├── 📁 src/                   # Source files (main.cpp)
└── platformio.ini            # Build configuration
```

## ✨ Features

### 🎮 RF Remote Control System

- **Direct joystick control** with dual analog joysticks
- **X-configuration motor mixing** with dynamic saturation scaling (no pre-reserved headroom)
- **5Hz control transmission** for responsive real-time control
- **Toggle switch arming system** with flight mode switching (Stabilized/Manual)
- **Flight mode control**: Stabilized mode with PID control and Manual mode for direct control
- **ACK payload telemetry** for live sensor feedback

### 🧠 Advanced Flight Control

- **FreeRTOS multi-threading** with dedicated motor control task
- **50Hz motor updates** for smooth ESC control
- **Immediate ESC calibration** on power-up
- **Safety systems** including control timeout and arming protection
- **Unified mixer algorithm**: Manual and PID modes both use full 1000–2000 µs throttle span; differential roll/pitch/yaw corrections are dynamically scaled only when they would saturate an ESC.
- **No pre-reserved headroom**: Maximum lift always available; mixer computes per-cycle positive/negative scaling factors (s_pos / s_neg) to keep authority while preventing clipping.
- **Altitude correction overlay**: Applied after throttle mapping (±300 µs window) and included in saturation scaling automatically.
- **Dual flight modes**: Stabilized (PID-assisted) and Manual (direct control) modes

### 📡 Comprehensive Sensor Suite

- **Environmental monitoring**: Temperature, humidity, pressure, air quality
- **Navigation sensors**: GPS positioning, IMU orientation, altitude tracking
- **Obstacle detection**: 4x Time-of-Flight sensors with I2C multiplexing
- **Light monitoring**: UV index and ambient light measurement
- **Battery monitoring**: Real-time voltage and percentage tracking

### 🌐 Connectivity & Telemetry

- **NRF24L01+ PA+LNA** for long-range RF communication
- **WiFi connectivity** for web dashboard access
- **Real-time telemetry** with 2-decimal precision altitude tracking
- **Data logging** with Firebase integration
- **Web-based control interface** for testing and configuration

## �️ Flight Mode System

The drone features a dual flight mode system for enhanced control flexibility:

### Flight Mode Controls

- **Toggle 1:** ARM/DISARM (primary safety control)
- **Toggle 2:** Flight Mode Selection (Stabilized/Manual)

### Available Flight Modes

#### 🎯 Manual Mode (Toggle 2 OFF)

- **Direct stick control** with no stabilization assistance
- **PID controllers disabled** for maximum control authority
- **Best for:** Experienced pilots, advanced maneuvers, acrobatic flight

#### 🛡️ Stabilized Mode (Toggle 2 ON)

- **PID-assisted flight** with automatic self-leveling
- **Roll, pitch, and yaw stabilization** for smooth control
- **Best for:** New pilots, stable photography, hover operations

### Real-time Mode Switching

- Switch between modes **during flight** for optimal control
- Seamless transitions with no motor interruption
- Current mode displayed in **telemetry output**
- Mode changes **logged to serial console** for monitoring

**📖 Complete Documentation:** See [FLIGHT_MODE_CONTROL.md](docs/FLIGHT_MODE_CONTROL.md) for detailed operation guide

## �🔧 Hardware Specifications

### 🎯 Main Controller

| Component           | Model                      | Purpose                                          |
| ------------------- | -------------------------- | ------------------------------------------------ |
| **Microcontroller** | ESP32 NodeMCU ESP-WROOM-32 | Main flight controller with dual-core processing |

### 📡 Communication Modules

| Component     | Model            | Interface | Purpose                               |
| ------------- | ---------------- | --------- | ------------------------------------- |
| **RF Module** | NRF24L01+ PA+LNA | SPI       | Long-range remote control & telemetry |
| **GPS**       | NEO-6M           | UART      | Position tracking and navigation      |
| **WiFi**      | Built-in ESP32   | -         | Web dashboard and configuration       |

### 🔬 Sensor Array

| Sensor              | Model          | Interface           | Measurement                      |
| ------------------- | -------------- | ------------------- | -------------------------------- |
| **IMU**             | MPU6050        | I2C (0x68)          | Gyroscope + Accelerometer        |
| **Environmental**   | BME280         | I2C (0x76/0x77)     | Pressure, temperature, humidity  |
| **Air Quality**     | ENS160 + AHT21 | I2C                 | CO₂, TVOC, temperature, humidity |
| **Light**           | BH1750         | I2C                 | Ambient light intensity          |
| **UV Sensor**       | GUVA-S12SD     | Analog              | UV index monitoring              |
| **Distance**        | 4x VL53L0X     | I2C via Multiplexer | Obstacle detection               |
| **I2C Multiplexer** | PCA9548A       | I2C (0x70)          | Sensor channel switching         |

### ⚡ Power & Control

| Component   | Purpose             | Specifications                    |
| ----------- | ------------------- | --------------------------------- |
| **ESCs**    | Motor speed control | 4x Brushless ESC with PWM control |
| **Motors**  | Propulsion          | EMAX 980KV brushless motors       |
| **Battery** | Power supply        | 3S LiPo with voltage monitoring   |
| **LEDs**    | Navigation lights   | RGB status indicators             |
| **Buzzer**  | Audio feedback      | Status and alert sounds           |

## 📌 Pin Configuration

### 🎮 Remote Controller Pins

| Function            | Component | GPIO Pin | Notes                           |
| ------------------- | --------- | -------- | ------------------------------- |
| **Joystick 1 X**    | Analog    | GPIO 39  | Primary control stick           |
| **Joystick 1 Y**    | Analog    | GPIO 36  | Primary control stick           |
| **Joystick 1 BTN**  | Digital   | GPIO 33  | Button (not functioning well)   |
| **Joystick 2 X**    | Analog    | GPIO 34  | Secondary control stick         |
| **Joystick 2 Y**    | Analog    | GPIO 35  | Secondary control stick         |
| **Joystick 2 BTN**  | Digital   | GPIO 32  | Button input                    |
| **Toggle Switch 1** | Digital   | GPIO 27  | ARM/DISARM control              |
| **Toggle Switch 2** | Digital   | GPIO 14  | Flight mode (Stabilized/Manual) |

### 🚁 Drone Controller Pins

#### Motor Control (ESC PWM Outputs)

| Motor       | Position          | GPIO Pin | ESC Connection   |
| ----------- | ----------------- | -------- | ---------------- |
| **Motor 1** | Front Right (CCW) | GPIO 13  | ESC1 signal wire |
| **Motor 2** | Back Right (CW)   | GPIO 12  | ESC2 signal wire |
| **Motor 3** | Front Left (CW)   | GPIO 14  | ESC3 signal wire |
| **Motor 4** | Back Left (CCW)   | GPIO 27  | ESC4 signal wire |

#### Communication Interfaces

| Interface           | Component | GPIO Pins                                  | Configuration                            |
| ------------------- | --------- | ------------------------------------------ | ---------------------------------------- |
| **SPI (NRF24L01+)** | RF Module | CE: 4, CSN: 5, SCK: 18, MOSI: 23, MISO: 19 | Channel 76 (WiFi interference avoidance) |
| **I2C Bus**         | Sensors   | SDA: 21, SCL: 22                           | Shared bus with pull-up resistors        |
| **UART**            | GPS       | RX: 16                                     | GPS data reception                       |

#### Sensor Connections

| Sensor Type         | GPIO Pin | Interface  | Purpose               |
| ------------------- | -------- | ---------- | --------------------- |
| **UV Sensor**       | GPIO 36  | Analog ADC | UV index measurement  |
| **Battery Monitor** | GPIO 35  | Analog ADC | Voltage divider input |

#### Status Indicators

| LED Position    | Color | GPIO Pin | Function         |
| --------------- | ----- | -------- | ---------------- |
| **Front Right** | Green | GPIO 32  | Navigation light |
| **Back Right**  | Green | GPIO 33  | Navigation light |
| **Back Left**   | Red   | GPIO 25  | Navigation light |
| **Front Left**  | Red   | GPIO 02  | Navigation light |
| **Back Center** | White | GPIO 15  | Status indicator |
| **Buzzer**      | Audio | GPIO 26  | Alert sounds     |

## 📡 Communication Protocols

### 📻 NRF24L01+ RF Communication

- **Frequency**: 2.4GHz, Channel 76 (WiFi interference avoidance)
- **Data Rate**: 250KBPS for maximum range
- **Power Level**: RF24_PA_HIGH for extended range
- **Control Rate**: 5Hz transmission (200ms intervals)
- **Protocol**: ACK payload for bidirectional communication
- **Range**: Extended range with PA+LNA amplifier

### 🔄 I2C Sensor Bus

- **Bus Speed**: Standard 100kHz
- **Pull-up Resistors**: 4.7kΩ on SDA/SCL lines
- **Multiplexing**: PCA9548A for multiple VL53L0X sensors
- **Device Addresses**:
  - PCA9548A: 0x70
  - BME280: 0x76/0x77
  - MPU6050: 0x68

### 🌐 WiFi Connectivity

- **Mode**: Station mode for web dashboard
- **Security**: WPA2 encryption
- **Purpose**: Configuration interface and telemetry viewing

## 🔋 Power Management

### Battery Monitoring System

The system monitors a 3S LiPo battery using a precision voltage divider:

**Voltage Divider Configuration:**

- **R1 (Top)**: 30kΩ (3x 10kΩ in series)
- **R2 (Bottom)**: 7.5kΩ (parallel combination of 30kΩ and 10kΩ)
- **Divider Ratio**: 0.2 (scales 12.6V max to 2.52V)

**ADC Conversion:**

```cpp
float vOut = (adcReading / 4095.0) * 3.3;  // ESP32 12-bit ADC
float batteryVoltage = vOut * 5.0;         // Scale back to actual voltage
```

### Power Distribution

- **5V Rail**: Regulated supply for motors and high-power components
- **3.3V Rail**: ESP32 and sensor power with dedicated filtering
- **NRF24L01**: Dedicated filtered 3.3V supply (not from ESP32)

## � Status Indication System

### Navigation Lights

| Position        | Color | Behavior  | Purpose              |
| --------------- | ----- | --------- | -------------------- |
| **Front Left**  | Red   | Always ON | Port navigation      |
| **Front Right** | Green | Always ON | Starboard navigation |
| **Back Left**   | Red   | Always ON | Port navigation      |
| **Back Right**  | Green | Always ON | Starboard navigation |

### Status Lights

| Position         | Color | Behavior   | Indication         |
| ---------------- | ----- | ---------- | ------------------ |
| **Front Center** | White | Slow blink | Waiting to arm     |
| **Front Center** | White | Solid      | Flight mode active |
| **Back Center**  | White | Variable   | System status      |

### Audio Feedback

- **Buzzer**: Status alerts and system notifications
- **Boot Sequence**: Confirmation sounds during initialization
- **Error Alerts**: Audio warnings for system issues

## 🏗️ System Architecture

### FreeRTOS Task Structure

| Task                 | Core   | Priority | Frequency | Purpose                       |
| -------------------- | ------ | -------- | --------- | ----------------------------- |
| **Motor Control**    | Core 1 | 4 (High) | 50Hz      | ESC PWM updates               |
| **RF Communication** | Core 0 | 3        | 5Hz       | Remote control data           |
| **Sensor Reading**   | Core 0 | 2        | 10Hz      | IMU and environmental sensors |
| **Status Updates**   | Core 0 | 1        | 1Hz       | LED and buzzer control        |

### Flight Control Logic

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RF Remote     │───▶│  Flight Control  │───▶│   Motor Mixing  │
│   Joystick      │    │   Processing     │    │  (X-Config)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │                        │
                                ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Sensor Data   │───▶│  PID Control     │    │   ESC Control   │
│  (IMU, Altitude)│    │   (Future)       │    │   (50Hz PWM)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🎮 Flight Control Modes

### Current Implementation

- **Manual Mode**: Direct joystick → mixer (full range) with dynamic saturation scaling
- **Stabilized Mode**: PID outputs normalized by mixer gains (roll/pitch/yaw) → same mixer path for identical feel and authority
- **Altitude Hold (in progress)**: Barometric altitude PID adds a symmetric ±300 µs base shift post throttle mapping
- **Safety Systems**: Arming sequence and emergency stop
- **Dynamic Scaling Details**:
  - Raw mixes: M1 = -kR R - kP P + kY Y, M2 = -kR R + kP P - kY Y, M3 = +kR R - kP P - kY Y, M4 = +kR R + kP P + kY Y
  - Compute posMax / negMin among mixes
  - s_pos = min(1, (PWM_max - T) / posMax) ; s_neg = min(1, (T - PWM_min) / -negMin)
  - Final motor = T + scaled mix (positive uses s_pos, negative uses s_neg)
  - Ensures full available thrust while proportionally shrinking only the saturated side

### Planned Features (PID Integration)

- **Stabilized Mode**: Automatic leveling with manual control
- **Altitude Hold**: Barometric altitude maintenance
- **Position Hold**: GPS-based position stabilization
- **Auto-Level**: Automatic return to level flight

## 🛠️ Installation

### Prerequisites

- **PlatformIO** or **Arduino IDE**
- **ESP32 Board Package**
- Required libraries (see `platformio.ini`)

### Hardware Setup

1. **Wire connections** according to pin configuration table
2. **Power supply** with proper voltage regulation
3. **Antenna mounting** for NRF24L01+ module
4. **Propeller installation** with correct rotation direction

### Software Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/esp32-flight-controller.git

# Navigate to project directory
cd esp32-flight-controller

# For stable drone firmware (production use)
cd firmware/stable/drone
# Upload droneFreeRTOS.ino using PlatformIO or Arduino IDE

# For stable remote controller firmware (production use)
cd ../remote
# Upload remoteControllerStable.ino using PlatformIO or Arduino IDE

# For development versions (experimental features)
cd ../../development/drone
# Upload development firmware for PID integration work

# Return to project root for PlatformIO builds
cd ../../
pio run --target upload
```

### Quick Start Guide

1. **Hardware Assembly** - Follow pin configuration tables for wiring
2. **Stable Firmware** - Use `firmware/stable/*` for production/flight operations
3. **Development Firmware** - Use `firmware/development/*` for PID implementation and testing
4. **Component Testing** - Use examples in `examples/` for individual component verification
5. **System Integration** - Test complete system using tools in `tools/` directory
6. **Documentation** - Refer to `docs/` for detailed progress and technical information## 🚀 Usage

### Initial Setup

1. **Power on** the system - ESCs will auto-calibrate
2. **Connect remote** - Verify RF communication link
3. **Arm system** - Use toggle switch SW1
4. **Test controls** - Verify joystick response

### Remote Control Operation

- **Left Joystick**: Throttle (Y-axis) and Yaw (X-axis)
- **Right Joystick**: Pitch (Y-axis) and Roll (X-axis)
- **SW1 Toggle**: ARM/DISARM system
- **SW2 Toggle**: Emergency stop (immediate motor shutdown)

### Safety Procedures

- **Always arm on level surface** with clear propeller area
- **Keep emergency stop accessible** during operation
- **Monitor battery voltage** to prevent over-discharge
- **Respect control timeout** - system disarms after 1 second of lost communication

## 📊 Development Status

### ✅ Completed Features

- RF remote control system with joystick integration
- FreeRTOS multi-threading architecture
- Comprehensive sensor integration
- ESC control with auto-calibration
- Safety systems and status indicators
- Real-time telemetry transmission

### 🔄 In Progress

- PID stabilization controller implementation
- MPU6050 sensor fusion algorithms
- Advanced flight modes

### 📋 Planned Features

- Autonomous flight capabilities
- GPS waypoint navigation
- Advanced telemetry dashboard
- Mobile app integration

For detailed progress tracking, see [docs/PROJECT_PROGRESS.md](docs/PROJECT_PROGRESS.md).

## 🗂️ Directory Navigation

- **🚁 [firmware/](firmware/)** - Stable and development firmware versions
  - **✅ [stable_drone/](firmware/stable_drone/)** - Production-ready drone firmware
  - **✅ [stable_remote/](firmware/stable_remote/)** - Production-ready remote firmware
  - **🔬 [development_drone/](firmware/development_drone/)** - PID integration development
  - **🔬 [development_remote/](firmware/development_remote/)** - Enhanced features development
- **📚 [docs/](docs/)** - Complete project documentation and progress tracking
- **🔬 [examples/](examples/)** - Sensor tests and component verification programs
- **🛠️ [tools/](tools/)** - Development utilities and web dashboard
- **🔌 [hardware/](hardware/)** - Hardware specifications and diagrams (planned)
- **📦 [archive/](archive/)** - Legacy versions and deprecated implementations## 🤝 Contributing

Contributions are welcome! Please read our contributing guidelines and submit pull requests for any improvements.

### Development Guidelines

- Follow Arduino/ESP32 coding standards
- Test thoroughly before submitting
- Update documentation for new features
- Maintain backward compatibility when possible

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ESP32 community for excellent documentation
- Arduino ecosystem for comprehensive libraries
- FreeRTOS for real-time operating system capabilities
- Open-source drone community for inspiration and guidance

---

**⚠️ Safety Notice**: This is experimental flight controller software. Always follow proper safety procedures when testing with live motors and propellers. Test in controlled environments and never operate near people or property.

```

```
