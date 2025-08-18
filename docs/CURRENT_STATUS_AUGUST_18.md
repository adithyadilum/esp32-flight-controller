# 🚁 Current System Status - August 18, 2025

## Summary

Development branch focuses on control latency reduction, precision telemetry, IMU orientation correction after hardware rotation, and advanced PID behavior refinements. Stable branch (v4.0) intentionally unchanged for GPS packet format to preserve remote compatibility.

## Key Updates Since Aug 7

| Area                | Change                                                                | Impact                                         |
| ------------------- | --------------------------------------------------------------------- | ---------------------------------------------- |
| RF Control Latency  | Polling raised to ~500Hz, NRF24 1Mbps, retries trimmed                | Faster stick→motor response                    |
| GPS Precision (Dev) | int16 *100 → int32 *1e7 (latitudeE7/longitudeE7)                      | ~1cm resolution in dev telemetry               |
| IMU Rotation        | 90° CW Z-axis remap + calibration transform                           | Correct roll/pitch orientation & drift removal |
| PID Controller      | Integral gating, leak, yaw feed-forward, optional cascaded angle→rate | More stable, responsive control                |
| Mixer               | Unified scaled mixer across builds                                    | Consistent tuning cross-platform               |
| Logging             | Clarified slow status print vs fast control loops                     | Prevents misinterpretation of responsiveness   |

## Loop Frequencies (Development)

- Radio poll: ~500Hz
- IMU task: 100Hz
- PID loop: 50Hz
- Motor output: 50Hz (driven by PID loop / control path)
- Telemetry ACK updates: ~1Hz (configurable)
- Status logging: 0.1Hz (throttled)

## Telemetry Packets

- Stable: 22B legacy (lat/long \*100 int16)
- Dev: 30B extended (latitudeE7 / longitudeE7 int32)
- Next: Introduce version byte for schema negotiation (planned)

## IMU Mapping (Post-Rotation)

```
bodyX = sensorY
bodyY = -sensorX
bodyZ = sensorZ
rollRate  = gyroY
pitchRate = -gyroX
yawRate   = gyroZ
```

Calibration offsets captured in sensor frame → transformed into body frame before bias subtraction.

## PID Enhancements

- Output limits: Roll/Pitch ±400 µs, Yaw ±200 µs
- Throttle-gated integral (integral only above ~35% throttle)
- iTermLeak prevents residual bias accumulation
- Derivative low-pass on input (derivativeAlpha)
- Setpoint smoothing + slew limiting (roll/pitch angle, yaw rate)
- Yaw feed-forward: adds proportional assist for crisp yaw
- Optional cascaded Angle→Rate path (disabled by default)

## Equivalence Validation

Confirmed identical post-PID PWM outputs between WiFi and RF builds under matched constants (mixer gains, PID gains, feed-forward) and same mode (no altitude adjustment / cascaded disabled).

## Pending / Planned

- Telemetry version byte & backward compatibility layer
- Yaw drift mitigation (magnetometer or enhanced gyro bias adaptation)
- High-rate debug attitude stream toggle
- Baro + GPS altitude fusion refinements
- Position hold (after yaw heading solution)

## Risk / Watch List

| Item                        | Risk                              | Mitigation                                             |
| --------------------------- | --------------------------------- | ------------------------------------------------------ |
| Divergent telemetry schemas | Remote parse failure              | Add version byte before promoting dev packet to stable |
| Yaw drift                   | Heading instability               | Implement magnetometer / adaptive bias                 |
| Overly low logging rate     | Debug difficulty                  | Add optional high-rate debug toggle                    |
| Integrator gating edge      | Slow I-term engagement at takeoff | Adjustable throttle gate parameter                     |

## Recommended Next Actions

1. Add telemetry version byte; update remote to branch parsing based on version.
2. Implement lightweight 25–50Hz debug stream (compact CSV) for flight log replay.
3. Begin yaw drift characterization to scope magnetometer requirement.
4. Prepare migration plan to promote high-precision GPS into stable after versioning in place.

---

Document generated from current development state (Aug 18, 2025).
