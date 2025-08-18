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
- **RF Communication**: NRF24L01 optimized – control polling ~500Hz (radio loop), telemetry ACK updates throttled ✅
- **Motor Control**: Unified X-configuration mixer with dynamic dual-side scaling (no fixed headroom) ✅
- **PID Controller**: Advanced custom controller (not SimplePID) implemented with:
  - Output limits (Roll/Pitch ±400 µs, Yaw ±200 µs)
  - Integral gating (throttle-based) & leak (bias bleed)
  - Derivative input low-pass filtering
  - Setpoint smoothing & slew limiting (roll/pitch angle, yaw rate)
  - Optional cascaded Angle→Rate loop (disabled by default)
  - Yaw feed-forward term
- **Sensor Integration**: MPU6050 fused via complementary filter (Kalman optional) with 90° CW Z-axis rotation compensation ✅

---

## 📡 Communication Strategy

- ✅ **Phase 1 Complete**: Control & telemetry via WiFi dashboard
- ✅ **Phase 2 Complete**: RF remote control (low-latency path tuned)
- ✅ **Phase 3 Complete**: Telemetry via NRF24 ACK payloads (legacy 22B stable / 30B dev)
- ✅ **Phase 4**: PID stabilization & advanced control features active (development & WiFi builds unified)

> 🛰️ **Current Status**: Full RF communication system operational! Remote sends control commands at 5Hz, drone responds with real-time telemetry via ACK payloads. System includes comprehensive sensor data, motor arming controls, and emergency stop functionality.

---

## ⬆️ Altitude / Base Throttle & Mixer Strategy

- **Lift-off Threshold**: ~1550–1600 µs (pack & weight dependent)
- **Full-Range Base**: Always map throttle directly to 1000–2000 µs; no static authority reservation.
- **Dynamic Dual Scaling**: Mixer computes raw mixes then applies s_pos / s_neg scaling to prevent saturation symmetrically.
- **Altitude Hold (dev)**: Adds centered correction (±300 µs window) post base mapping (future blending with GPS altitude).
- **Altitude Source Roadmap**: Baro primary now; GPS fusion planned after stable yaw heading solution.

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

## 🛠️ Development Notes (Updated Aug 18, 2025)

- ✅ **ESC Control**: Motors respond to joystick inputs with X-configuration mixing and per-cycle dynamic scaling
- ✅ **RF Communication**: 5Hz control loop with telemetry feedback operational
- ✅ **Joystick Sensitivity**: Tuned to ±3000 range for precise flight control
- ✅ **Safety Systems**: Toggle switch arming, emergency stop, control timeout protection
- ✅ Advanced PID (P+I+D with gating, leak, filtering) implemented & active
- ✅ IMU rotation (90° CW about Z) compensated in both RF & WiFi builds
- ✅ GPS precision upgrade (dev build latitudeE7 / longitudeE7 int32 scaling \*1e7)
- ✅ Mixer & PID output normalization identical across builds (validated equivalence)
- ✅ Low-latency radio loop tuning (≈500Hz polling)
- 🔄 Yaw absolute heading & magnetometer integration (pending)
- 🔄 Telemetry versioning (introduce packet schema version byte) (pending)
- 🔄 High-rate lightweight debug stream toggle (pending)

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
- ✅ Implement PID stabilization with advanced controller features
- ✅ Cross-build IMU & mixer parity
- 🔄 Yaw drift mitigation improvements

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

- Telemetry export of s_pos / s_neg and raw mixes (diagnostics)
- Adaptive gain scheduling vs battery sag
- Optional minimal reserved authority reserve for aggressive acro
- Packet version byte for dev/stable compatibility
- Yaw heading fusion & magnetometer integration
- Altitude fusion (baro + GPS + future ToF blend)

---

## 🆕 IMU Rotation & Calibration Handling

The MPU6050 board was rotated 90° clockwise about the Z-axis. The firmware applies a deterministic remap before fusion and transforms saved calibration offsets into the body frame:

```
bodyX = sensorY
bodyY = -sensorX
bodyZ = sensorZ
rollRate  = gyroY
pitchRate = -gyroX
yawRate   = gyroZ
```

Calibration samples are captured in sensor frame; offsets are projected into body frame prior to subtraction. This prevents introducing artificial bias after rotation. Complementary (default) and optional Kalman filters are seeded after calibration to eliminate startup bias.

---

## 🆕 High-Precision GPS Telemetry (Development Branch)

Legacy telemetry used int16 latitude/longitude *100 (≈1e-2°). Development build upgrades to int32 latitudeE7/longitudeE7 (degrees *1e7) reaching centimeter-scale resolution. Stable branch left unchanged to avoid incompatibility with existing remote firmware until a packet versioning mechanism is introduced.

---

## 🧪 PID Output & Mixer Equivalence Across Builds

Code audit confirmed that, under identical constants (MIX_KR/KP/KY, PID limits, yaw feed-forward), both RF (`droneFreeRTOS.ino`) and WiFi (`flight_controller-v2.cpp`) builds produce identical post-PID motor PWM outputs. Any divergence stems only from enabled optional features (cascaded mode, yaw FF differences, altitude correction) or differing runtime tunables.
