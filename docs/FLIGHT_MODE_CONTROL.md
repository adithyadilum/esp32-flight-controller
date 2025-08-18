# 🚁 Flight Mode Control System

**Last Updated:** August 18, 2025  
**System Status:** ✅ FULLY OPERATIONAL (with enhanced PID gating & mixer parity)

---

## Overview

The ESP32 Flight Controller features a dual flight mode system that allows pilots to switch between manual control and stabilized flight assistance in real-time. This system provides flexibility for both experienced pilots who want direct control and those who prefer assisted flight.

### Aug 18 Addendum

- Integral portion of attitude PIDs now automatically gates below a throttle threshold to avoid integrator windup while allowing immediate P/D authority at takeoff.
- Unified scaled mixer logic is identical across RF and WiFi builds → consistent behavior when switching platforms.
- Yaw feed-forward introduced (small proportional assist) improving yaw responsiveness without destabilizing derivative behavior.

## Flight Mode Controls

### Toggle Switch Assignment

- **Toggle 1 (GPIO 27):** ARM/DISARM Control
- **Toggle 2 (GPIO 14):** Flight Mode Selection

### Flight Mode States

| Toggle 2 Position | Flight Mode         | PID Controllers | Description                                       |
| ----------------- | ------------------- | --------------- | ------------------------------------------------- |
| **OFF (0)**       | **Manual Mode**     | Disabled        | Direct stick control, no stabilization assistance |
| **ON (1)**        | **Stabilized Mode** | Enabled         | PID-assisted flight with automatic leveling       |

## Flight Mode Descriptions

### 🎯 Manual Mode (Toggle 2 OFF)

- **Control Type:** Direct joystick-to-motor mapping
- **PID System:** Disabled
- **Best For:**
  - Experienced pilots
  - Advanced maneuvers
  - Acrobatic flight
  - Maximum control authority
- **Characteristics:**
  - No automatic leveling
  - Direct response to stick inputs
  - Full pilot responsibility for stability
  - Higher skill requirement

### 🛡️ Stabilized Mode (Toggle 2 ON)

- **Control Type:** PID-assisted stabilization
- **PID System:** Enabled (Roll, Pitch, Yaw control)
- **Best For:**
  - New pilots
  - Stable photography/videography
  - Hover operations
  - Windy conditions
- **Characteristics:**
  - Automatic self-leveling
  - Smoother flight characteristics
  - Easier to fly and control
  - Reduced pilot workload

## Operation Sequence

### 1. Pre-Flight Setup

1. Power on remote controller
2. Power on drone
3. Verify RF communication established
4. Select desired flight mode with Toggle 2
5. ARM motors with Toggle 1

### 2. Flight Mode Selection

- **Before Arming:** Set Toggle 2 to desired initial flight mode
- **During Flight:** Toggle 2 can be switched at any time for real-time mode changes

### 3. In-Flight Mode Switching

```
Stabilized Mode → Manual Mode:
- Toggle 2: ON → OFF
- PID controllers disable
- Control transitions to direct mode
- Serial output: "🎯 FLIGHT MODE: MANUAL (PID OFF)"

Manual Mode → Stabilized Mode:
- Toggle 2: OFF → ON
- PID controllers enable
- Automatic leveling begins
- Serial output: "🛡️ FLIGHT MODE: STABILIZED (PID ON)"
```

## Safety Features

### Primary Safety System

- **Toggle 1 (ARM/DISARM)** remains the primary safety control
- Disarming immediately stops all motors regardless of flight mode
- 1-second control timeout safety system active in both modes

### Flight Mode Safety

- Mode switching is smooth with no motor interruption
- PID controllers gracefully enable/disable during transitions
- Current flight mode always displayed in telemetry output
- Mode changes logged to serial console for monitoring

## Technical Implementation

### Code Architecture

```cpp
// Flight mode variables
volatile bool motorsArmed = false;          // Primary safety control
volatile bool stabilizedMode = false;       // Flight mode status
bool pidEnabled = false;                    // PID controller state

// Mode switching logic in calculateMotorSpeeds()
if (receivedControl.toggle2 == 1) {
    currentFlightMode = FLIGHT_MODE_STABILIZE;
    pidEnabled = true;
    stabilizedMode = true;
} else {
    currentFlightMode = FLIGHT_MODE_MANUAL;
    pidEnabled = false;
    stabilizedMode = false;
}
```

### Communication Protocol

- Flight mode state transmitted from remote via Toggle 2
- 5Hz control packet transmission rate
- Flight mode changes detected and logged in real-time
- Status information included in telemetry feedback

## Status Display

### Serial Monitor Output

```
Motors: ARMED [STABILIZED]  - Stabilized mode active
Motors: ARMED [MANUAL]      - Manual mode active
Motors: DISARMED           - Motors disarmed
```

### Flight Mode Change Messages

```
🔓 MOTORS ARMED - STABILIZED MODE ACTIVE (PID ON)
🔓 MOTORS ARMED - MANUAL MODE ACTIVE (PID OFF)
🛡️ FLIGHT MODE: STABILIZED (PID ON)
🎯 FLIGHT MODE: MANUAL (PID OFF)
```

## Troubleshooting

### Common Issues

#### Mode Not Switching

- **Cause:** RF communication issues
- **Solution:** Check radio connection and signal strength
- **Check:** Toggle 2 position on remote controller

#### Unstable Flight in Manual Mode

- **Cause:** Normal behavior - no stabilization active
- **Solution:** Switch to Stabilized mode or improve piloting skills
- **Note:** Manual mode requires active pilot input for stability

#### Too Stable in Manual Mode

- **Cause:** Still in Stabilized mode
- **Solution:** Ensure Toggle 2 is OFF position
- **Check:** Serial monitor for current flight mode confirmation

### Diagnostic Commands

- Monitor serial output for flight mode status
- Check `stabilizedMode` variable state
- Verify `pidEnabled` state matches expected mode
- Confirm toggle switch positions on remote

## Advantages Over Emergency Stop System

### Previous Emergency Stop Issues

- ❌ Dangerous power loss during flight
- ❌ No recovery possible after activation
- ❌ Limited pilot control options
- ❌ Binary on/off operation only

### New Flight Mode Benefits

- ✅ Safe operation with continuous motor control
- ✅ Real-time mode switching capability
- ✅ Enhanced pilot control flexibility
- ✅ Smooth transitions between assistance levels
- ✅ Maintains flight capability in both modes
- ✅ Better training progression (manual → stabilized)

## Conclusion

The Flight Mode Control System provides pilots with the flexibility to choose their level of assistance while maintaining safety through the primary ARM/DISARM system. This implementation offers both the direct control experienced pilots desire and the stability assistance that makes drone operation accessible to newer pilots.
