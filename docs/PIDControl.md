## ✈️ ESP32 Weather Drone – Flight Controller Overview

This project implements a PID-based flight controller for a custom ESP32-powered weather drone, designed to stabilize flight, handle sensor data, and provide real-time telemetry. The project is structured around FreeRTOS for multitasking and uses WiFi for initial control/testing before transitioning to NRF24L01.

---

## ⚙️ Motor Pin Allocation (PWM Outputs)

| GPIO Pin | Motor Position | Rotation Direction      | Propeller Type | Status    |
| -------- | -------------- | ----------------------- | -------------- | --------- |
| GPIO 27  | Back Left      | Counter-Clockwise (CCW) | CCW            | ✅ Active |
| GPIO 14  | Front Left     | Clockwise (CW)          | CW             | ✅ Active |
| GPIO 12  | Back Right     | Clockwise (CW)          | CW             | ✅ Active |
| GPIO 13  | Front Right    | Counter-Clockwise (CCW) | CCW            | ✅ Active |

> ✅ **Current Status**: Motors are fully operational with RF remote control. ESC calibration occurs automatically on power-on. Toggle switch 1 arms/disarms motors, toggle switch 2 provides emergency stop.

---

## 🧠 Control Architecture

- **Framework**: FreeRTOS with task-based scheduling ✅
- **Servo Control**: Uses `ESP32Servo` library for PWM motor control ✅
- **RF Communication**: NRF24L01 at 5Hz with ACK payload telemetry ✅
- **Motor Control**: X-configuration mixing with dynamic saturation scaling (no pre-reserved headroom) ✅
- **PID Controller**: `SimplePID` planned for implementation:
  - Roll stabilization (next priority)
  - Pitch stabilization (next priority)
  - Yaw control (optional)
- **Sensor Integration**: MPU6050 gyro/accelerometer integration needed

---

## 📡 Communication Strategy

- ✅ **Phase 1 Complete**: Control & telemetry via WiFi dashboard
- ✅ **Phase 2 Complete**: **RF remote control** via NRF24L01 module
- ✅ **Phase 3 Complete**: **Telemetry transmission** to remote using **NRF24L01 acknowledgment payloads**
- ⬜ **Phase 4 Next**: **PID stabilization** with RF remote parameter tuning

> 🛰️ **Current Status**: Full RF communication system operational! Remote sends control commands at 5Hz, drone responds with real-time telemetry via ACK payloads. System includes comprehensive sensor data, motor arming controls, and emergency stop functionality.

---

## ⬆️ Altitude / Base Throttle Strategy

- **Lift-off Threshold**: Motors begin lifting frame at ~1550–1600 µs (battery dependent)
- **Base Throttle Mapping**: User throttle always maps full idle→max range (no pre-reserved correction margin).
- **Dynamic Mixer Scaling**: Differential roll/pitch/yaw corrections are scaled only when they would saturate an ESC output.
- **Altitude Correction Overlay**: Altitude PID (0..1000) is centered to (−1..+1) and mapped to ±300 µs, added after base throttle mapping, then clamped.
- **Altitude Source**:
  - Pre GPS lock: Barometric (BME280)
  - After reliable GPS (≥4 sats): GPS altitude blending planned

---

## 🔍 Sensor Suite

| Sensor Module      | Function                                      |
| ------------------ | --------------------------------------------- |
| **MPU6050**        | Gyro + Accelerometer (orientation)            |
| **BME280**         | Pressure + Temp + Humidity                    |
| **VL53L0X x4**     | Short-range ToF sensors for obstacle distance |
| **ENS160 + AHT21** | Air quality (eCO₂, TVOC) + Temp/Humidity      |
| **GPS Module**     | Latitude, Longitude, Altitude, Speed          |
| **PCA9548A MUX**   | I2C multiplexer for sensor management         |

---

## 🛠️ Development Notes

- ✅ **ESC Control**: Motors respond to joystick inputs with X-configuration mixing and per-cycle dynamic scaling
- ✅ **RF Communication**: 5Hz control loop with telemetry feedback operational
- ✅ **Joystick Sensitivity**: Tuned to ±3000 range for precise flight control
- ✅ **Safety Systems**: Toggle switch arming, emergency stop, control timeout protection
- ⬜ **Next Phase**: Begin PID tuning with P term → then add D → finally add I
- ⬜ **PID Testing**: Real-time parameter adjustment via RF remote controls
- ⬜ **Stabilization**: Implement MPU6050 feedback for roll/pitch/yaw control
- ⬜ **Flight Testing**: Progressive testing from basic lift to full stabilization

---

## 📅 Progress Tracking

- ✅ WiFi control tested
- ✅ **FreeRTOS drone implementation complete** (droneFreeRTOS.ino)
- ✅ **RF remote control implemented** (remoteControllerStable.ino)
- ✅ **ESC control with joystick inputs** - Motors respond to remote commands
- ✅ **5Hz RF24 communication** - Stable control at 200ms intervals
- ✅ **Toggle switch arming system** - SW1=ARM, SW2=EMERGENCY_STOP
- ✅ **Comprehensive telemetry system** - BME280, GPS, air quality sensors
- ✅ **Joystick sensitivity tuning** - ±3000 range for precise control
- ✅ **ESC calibration on power-on** - Immediate MAX→MIN calibration sequence
- ⬜ **Implement PID stabilization** - Next critical milestone (mixer path already unified)

---

## 🔄 Unified Scaled Motor Mixer (Technical Detail)

Mix equations (normalized R,P,Y in [-1,1]):

```
M1 = -kR*R - kP*P + kY*Y   (Front Right CCW)
M2 = -kR*R + kP*P - kY*Y   (Back  Right CW)
M3 =  kR*R - kP*P - kY*Y   (Front Left  CW)
M4 =  kR*R + kP*P + kY*Y   (Back  Left  CCW)
```

Scaling:

1. posMax = max positive mix; negMin = most negative mix
2. s_pos = min(1, (PWM_max - T)/posMax) if posMax>0
3. s_neg = min(1, (T - PWM_min)/(-negMin)) if negMin<0
4. Each motor = T + (m>=0 ? s_pos*m : s_neg*m)
5. Clamp to [PWM_min, PWM_max]

PID normalization:

```
Rn = clamp( rollPID / kR, -1, 1 )
Pn = clamp( pitchPID / kP, -1, 1 )
Yn = clamp( yawPID / kY, -1, 1 )
```

Advantages:

- Full thrust retained until real saturation
- Proportional axis balance preserved (no artificial clipping)
- Eliminates guess-based headroom reservation

Future Enhancements:

- Telemetry export of s_pos / s_neg and raw mixes
- Adaptive gain scheduling w.r.t battery voltage
- Optional minimal reserved authority band for aggressive acro tuning
- ⬜ **PID tuning with RF remote** - Real-time PID parameter adjustment
- ⬜ **Add MPU6050 integration** - Gyro/accelerometer for stabilization
- ⬜ **Add dynamic altitude hold and GPS switching logic**
