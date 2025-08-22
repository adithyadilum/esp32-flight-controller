/*
 * DEVELOPMENT VERSION - Drone with Enhanced Sensor Suite - FreeRTOS Version
 * 🔬 DEVELOPMENT FIRMWARE FOR PID STABILIZATION INTEGRATION
 *
 * Complete telemetry system with BME280, AHT21, GPS, and simulated advanced sensors
 * Compatible with remote control system (12-byte control, 22-byte telemetry)
 *
 * DEVELOPMENT FOCUS:
 * - PID stabilization controller implementation
 * - MPU6050 sensor fusion integration
 * - Advanced flight modes development
 * - Parameter tuning and optimization
 *
 * ⚠️ EXPERIMENTAL - FOR DEVELOPMENT USE ONLY
 * Use stable_drone/droneFreeRTOS.ino for production flights
 *
 * FreeRTOS Task Architecture:
 * - SensorTask: Reads all sensors (1Hz for environmental, 10Hz for GPS)
 * - RadioTask: Handles RF24 communication (now 500Hz polling for low-latency control reception, ~1Hz telemetry ack updates)
 * - StatusTask: Prints system status and diagnostics (0.1Hz)
 * - MotorTask: ESC control with PID integration (200Hz, was 50Hz)
 * - IMUTask: High-frequency IMU reading and calibration (200Hz)
 * - PIDTask: PID control loop for stabilization (200Hz, was 50Hz)
 *
 * NOTE: Task frequencies updated to match WiFi code for consistent behavior
 *
 * NOTE: PID control is active but PID values are not transmitted in telemetry
 * to keep telemetry focused on sensor data only.
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPS++.h>
#include <BH1750.h>
#include <ScioSense_ENS160.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5
#define BATTERY_PIN 35
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GUVA_PIN 36 // GUVA-S12SD UV sensor

// ESC Control Pins (PWM Outputs to Brushless Motors)
#define ESC1_PIN 13 // Front Right (CCW)
#define ESC2_PIN 12 // Back Right (CW)
#define ESC3_PIN 14 // Front Left (CW)
#define ESC4_PIN 27 // Back Left (CCW)

// ESC PWM Configuration
#define ESC_MIN_PULSE 1000 // Minimum pulse width (microseconds)
#define ESC_MAX_PULSE 2000 // Maximum pulse width (microseconds)
#define ESC_ARM_PULSE 1000 // Arming pulse width
#define ESC_FREQUENCY 50   // 50Hz PWM frequency for ESCs

// I2C addresses
#define BME280_ADDRESS 0x76
#define BH1750_ADDRESS 0x23
#define ENS160_ADDRESS 0x53
#define MPU6050_ADDRESS 0x68

// PID Control Configuration
#define PID_LOOP_FREQUENCY 200                      // Hz - increased from 50Hz for faster stabilization
#define PID_LOOP_PERIOD (1000 / PID_LOOP_FREQUENCY) // ms

// Extended PID behavior configuration
const float PID_THROTTLE_GATE = 0.35f;             // Attitude PID enable threshold (fraction of total throttle)
const float PID_ITERM_LEAK = 0.001f;               // Fractional leak applied each compute when integral allowed
const float PID_SETPOINT_SLEW_ROLL_PITCH = 120.0f; // deg/sec max change (roll/pitch)
const float PID_SETPOINT_SLEW_YAW_RATE = 720.0f;   // deg/sec^2 equivalent (rate change per second) for yaw rate transitions
const float YAW_FEED_FORWARD_GAIN = 0.10f;         // Feed-forward gain for yaw rate (portion of setpoint added to output)
const float MANUAL_AXIS_GAIN = 1.30f;              // Gain applied to manual roll/pitch (and yaw) stick inputs for stronger authority

// PID Output Limits
#define PID_ROLL_PITCH_MAX 400 // ±400 microseconds for roll/pitch correction
#define PID_YAW_MAX 200        // ±200 microseconds for yaw correction
#define PID_ALTITUDE_MAX 300   // ±300 microseconds for altitude correction

// ================================
// PID TUNING PARAMETERS - Easy Adjustment Section
// ================================
// Roll PID Gains (Angle Stabilization)
const double ROLL_KP = 2.5;  // Proportional gain - increase for faster response
const double ROLL_KI = 0.1;  // Integral gain - increase to eliminate steady-state error
const double ROLL_KD = 0.15; // Derivative gain - increase for more damping

// Pitch PID Gains (Angle Stabilization)
const double PITCH_KP = 2.5;  // Proportional gain - usually same as roll
const double PITCH_KI = 0.1;  // Integral gain - usually same as roll
const double PITCH_KD = 0.15; // Derivative gain - usually same as roll

// Yaw PID Gains (Rate Control)
const double YAW_KP = 1.0;  // Proportional gain - lower than roll/pitch
const double YAW_KI = 0.05; // Integral gain - very small for yaw
const double YAW_KD = 0.05; // Derivative gain - minimal for yaw

// Optional cascaded control (Angle -> Rate) gains for roll/pitch inner rate loops
const double ROLL_RATE_KP = 0.625;  // Inner rate PID Kp (deg/s -> μs)
const double ROLL_RATE_KI = 2.10;   // Inner rate PID Ki
const double ROLL_RATE_KD = 0.0088; // Inner rate PID Kd
const double PITCH_RATE_KP = ROLL_RATE_KP;
const double PITCH_RATE_KI = ROLL_RATE_KI;
const double PITCH_RATE_KD = ROLL_RATE_KD;

// Altitude PID Gains (Position Control)
const double ALT_KP = 1.5; // Proportional gain for altitude hold
const double ALT_KI = 0.2; // Integral gain for altitude hold
const double ALT_KD = 0.8; // Derivative gain for altitude hold

// Advanced PID Parameters
const double MAX_ANGLE = 30.0;     // Maximum roll/pitch angle (degrees)
const double MAX_YAW_RATE = 180.0; // Maximum yaw rate (degrees/second)
const double MAX_CLIMB_RATE = 2.0; // Maximum climb rate (m/s)

// Filter Coefficients (0.0 to 1.0, higher = more filtering)
const double ROLL_FILTER = 0.1;  // Derivative filter for roll
const double PITCH_FILTER = 0.1; // Derivative filter for pitch
const double YAW_FILTER = 0.15;  // Derivative filter for yaw (more filtering)
const double ALT_FILTER = 0.05;  // Derivative filter for altitude

// Setpoint Smoothing (0.0 to 1.0, lower = more smoothing)
const double ROLL_SMOOTH = 0.05;  // Setpoint smoothing for roll
const double PITCH_SMOOTH = 0.05; // Setpoint smoothing for pitch
const double YAW_SMOOTH = 0.1;    // Setpoint smoothing for yaw
const double ALT_SMOOTH = 0.02;   // Setpoint smoothing for altitude
// ================================

// Angle/rate constraints
const double MAX_ROLL_PITCH_RATE = 300.0; // deg/s limit for roll/pitch rate setpoints in cascaded mode

// Optional 1D Kalman filter parameters for roll/pitch fusion (when enabled)
const float KALMAN_RATE_STD = 4.0f; // deg/s process noise standard deviation
const float KALMAN_MEAS_STD = 3.0f; // deg measurement noise standard deviation

// Feature toggles (safe defaults: disabled)
static bool USE_KALMAN_ATTITUDE = false;     // If true, use 1D Kalman for roll/pitch instead of complementary filter
static bool USE_CASCADED_ANGLE_RATE = false; // If true, use angle outer loop -> rate inner loop for roll/pitch

// Task stack sizes
#define SENSOR_TASK_STACK 8192
#define RADIO_TASK_STACK 4096
#define STATUS_TASK_STACK 2048

// Task priorities
#define SENSOR_TASK_PRIORITY 2
#define RADIO_TASK_PRIORITY 3
#define STATUS_TASK_PRIORITY 1

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Initialize real sensors
Adafruit_BME280 bme280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;
BH1750 lightMeter;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // Prefer standard 0x53 address; will probe and retry in init
Adafruit_MPU6050 mpu6050;

// Track GPS altitude (meters) separately for fusion with barometric altitude
volatile float gpsAltitudeMeters = NAN;

// ESC Servo objects
Servo esc1, esc2, esc3, esc4;

// Control packet (12 bytes) - Added button states and toggle switches to match remote
struct ControlPacket
{
    int16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t joy1_btn; // Joystick 1 button state (0 = pressed, 1 = released)
    uint8_t joy2_btn; // Joystick 2 button state (0 = pressed, 1 = released)
    uint8_t toggle1;  // Toggle switch 1 state (0 = off, 1 = on)
    uint8_t toggle2;  // Toggle switch 2 state (0 = off, 1 = on)
};

// Enhanced telemetry packet (development) - now with high precision GPS (lat/lon * 1e7)
// NOTE: Size increased due to int32_t latitude/longitude; keep under NRF24 32-byte limit.
struct TelemetryPacket
{
    int16_t temperature;  // x100 - Real BME280 data
    uint16_t pressureX10; // x10 - Real BME280 data (one decimal, avoids 16-bit overflow)
    uint8_t humidity;     // % - Real AHT21 data
    uint16_t battery;     // mV - Real battery voltage
    int32_t latitudeE7;   // GPS latitude * 1e7 (high precision, ~1cm)
    int32_t longitudeE7;  // GPS longitude * 1e7
    uint8_t satellites;   // GPS satellite count
    uint8_t status;       // System status
    uint16_t lux;         // Light level in lux
    int16_t altitude;     // Altitude in centimeters from BME280 (for 2 decimal precision)
    uint16_t uvIndex;     // UV index x100 from GUVA sensor
    uint16_t eCO2;        // Equivalent CO2 in ppm from ENS160
    uint16_t TVOC;        // Total VOC in ppb from ENS160
};

// Enhanced PID Controller Structure with Advanced Features
struct PIDController
{
    double setpoint;
    double smoothedSetpoint; // For setpoint smoothing
    double input;
    double filteredInput; // For derivative filtering
    double output;
    double kp, ki, kd;
    double lastInput;
    double lastFilteredInput;
    double iTerm;
    double outputMin, outputMax;
    double iTermMin, iTermMax; // Separate integral limits for anti-windup
    unsigned long lastTime;
    bool enabled;
    bool allowIntegral; // Gating flag for integrator (throttle / on-ground suppression)

    // Cached last-loop terms for accurate diagnostics (avoid recomputation artifacts)
    double lastPTerm;
    double lastITerm;
    double lastDTerm;
    double lastDt; // seconds

    // Derivative filtering
    double derivativeAlpha; // Low-pass filter coefficient for derivative

    // Setpoint smoothing
    double setpointAlpha; // Smoothing factor for setpoint changes

    // Anti-windup flags
    bool wasOutputSaturated;
    bool wasIntegralSaturated;

    void initialize(double p, double i, double d, double minOut, double maxOut)
    {
        kp = p;
        ki = i;
        kd = d;
        outputMin = minOut;
        outputMax = maxOut;

        // Set integral limits to 50% of output range to prevent windup
        double range = outputMax - outputMin;
        iTermMin = outputMin + range * 0.25;
        iTermMax = outputMax - range * 0.25;

        lastInput = 0;
        lastFilteredInput = 0;
        filteredInput = 0;
        iTerm = 0;
        setpoint = 0;
        smoothedSetpoint = 0;
        lastTime = millis();
        enabled = false;
        wasOutputSaturated = false;
        wasIntegralSaturated = false;
        allowIntegral = true;
        lastPTerm = 0;
        lastITerm = 0;
        lastDTerm = 0;
        lastDt = 0;

        // Filter coefficients (higher = more filtering)
        derivativeAlpha = 0.1; // 10% new, 90% old for derivative
        setpointAlpha = 0.05;  // 5% new, 95% old for setpoint smoothing
    }

    double compute()
    {
        if (!enabled)
            return 0;

        unsigned long now = millis();
        double dt = (double)(now - lastTime) / 1000.0; // Convert to seconds for proper units

        // Only compute if sufficient time has passed (minimum 1ms)
        if (dt < 0.001)
            return output;

        // Setpoint smoothing - gradual transition to new setpoint
        smoothedSetpoint = smoothedSetpoint + setpointAlpha * (setpoint - smoothedSetpoint);

        // Input filtering for derivative term
        filteredInput = filteredInput + derivativeAlpha * (input - filteredInput);

        double error = smoothedSetpoint - input;

        // Proportional term
        double pTerm = kp * error;

        // Integral term with advanced anti-windup
        // Only accumulate integral if output is not saturated OR error is helping to reduce saturation
        bool shouldAccumulateIntegral = !wasOutputSaturated ||
                                        (wasOutputSaturated && ((error > 0 && output <= outputMin) ||
                                                                (error < 0 && output >= outputMax)));
        if (shouldAccumulateIntegral && allowIntegral && ki != 0.0)
        {
            iTerm += ki * error * dt;

            // Clamp integral term to prevent windup
            wasIntegralSaturated = false;
            if (iTerm > iTermMax)
            {
                iTerm = iTermMax;
                wasIntegralSaturated = true;
            }
            else if (iTerm < iTermMin)
            {
                iTerm = iTermMin;
                wasIntegralSaturated = true;
            }
        }

        // Derivative term with filtered input to reduce noise
        double dInput = 0;
        if (dt > 0)
        {
            dInput = (filteredInput - lastFilteredInput) / dt;
        }
        double dTerm = -kd * dInput; // Negative because we want to reduce rate of change (derivative-on-measurement)

        // Compute final output
        double rawOutput = pTerm + iTerm + dTerm;

        // Output saturation/clamping
        output = rawOutput;
        if (output > outputMax)
        {
            output = outputMax;
            wasOutputSaturated = true;
        }
        else if (output < outputMin)
        {
            output = outputMin;
            wasOutputSaturated = true;
        }
        else
        {
            wasOutputSaturated = false;
        }

        // Remember values for next iteration
        lastInput = input;
        lastFilteredInput = filteredInput;
        lastTime = now;

        // Store diagnostic terms
        lastPTerm = pTerm;
        lastITerm = iTerm;
        lastDTerm = dTerm;
        lastDt = dt;

        // iTerm leak: bleed accumulated bias slowly when enabled to prevent long-term windup memory
        if (allowIntegral && PID_ITERM_LEAK > 0.0f)
        {
            iTerm *= (1.0 - PID_ITERM_LEAK);
            lastITerm = iTerm; // keep diagnostics consistent
        }

        return output;
    }

    void reset()
    {
        iTerm = 0;
        lastInput = input;
        lastFilteredInput = input;
        filteredInput = input;
        smoothedSetpoint = setpoint;
        lastTime = millis();
        wasOutputSaturated = false;
        wasIntegralSaturated = false;
    }

    void setTunings(double p, double i, double d)
    {
        // Prevent negative gains
        kp = (p < 0) ? 0 : p;
        ki = (i < 0) ? 0 : i;
        kd = (d < 0) ? 0 : d;
    }

    void setOutputLimits(double min, double max)
    {
        if (min >= max)
            return; // Invalid limits

        outputMin = min;
        outputMax = max;

        // Update integral limits
        double range = outputMax - outputMin;
        iTermMin = outputMin + range * 0.25;
        iTermMax = outputMax - range * 0.25;

        // Clamp current values if needed
        if (iTerm > iTermMax)
            iTerm = iTermMax;
        else if (iTerm < iTermMin)
            iTerm = iTermMin;

        if (output > outputMax)
            output = outputMax;
        else if (output < outputMin)
            output = outputMin;
    }

    void setDerivativeFilter(double alpha)
    {
        derivativeAlpha = constrain(alpha, 0.01, 1.0); // Limit to reasonable range
    }

    void setSetpointSmoothing(double alpha)
    {
        setpointAlpha = constrain(alpha, 0.01, 1.0); // Limit to reasonable range
    }

    void setIntegralEnabled(bool enable, bool resetOnDisable = true)
    {
        allowIntegral = enable;
        if (!enable && resetOnDisable)
        {
            // Decay integral smoothly instead of hard zero to avoid step in output
            iTerm *= 0.5;
        }
    }

    void setIntegralLimits(double minPercent, double maxPercent)
    {
        double range = outputMax - outputMin;
        iTermMin = outputMin + range * constrain(minPercent, 0.0, 0.5);
        iTermMax = outputMax - range * constrain(maxPercent, 0.0, 0.5);
    }

    // Diagnostic functions
    double getProportionalTerm() { return lastPTerm; }
    double getIntegralTerm() { return lastITerm; }
    double getDerivativeTerm() { return lastDTerm; }
    double getLastDt() { return lastDt; }
    bool isOutputSaturated() { return wasOutputSaturated; }
    bool isIntegralSaturated() { return wasIntegralSaturated; }
    double getSmoothedSetpoint() { return smoothedSetpoint; }
};

// IMU Data Structure
struct IMUData
{
    float roll, pitch, yaw;             // Euler angles in degrees
    float rollRate, pitchRate, yawRate; // Angular velocities in deg/s
    float accelX, accelY, accelZ;       // Accelerometer data in m/s²
    float gyroX, gyroY, gyroZ;          // Gyroscope data in deg/s
    float temperature;                  // IMU temperature
    bool dataValid;                     // Data validity flag
    unsigned long timestamp;            // Data timestamp
};

// Flight Mode Enumeration
enum FlightMode
{
    FLIGHT_MODE_DISARMED = 0,
    FLIGHT_MODE_MANUAL = 1,        // Direct stick control (no stabilization)
    FLIGHT_MODE_STABILIZE = 2,     // Angle stabilization (default)
    FLIGHT_MODE_ALTITUDE_HOLD = 3, // Stabilize + altitude hold
    FLIGHT_MODE_POSITION_HOLD = 4  // Full GPS position hold
};

// Advanced PID Configuration Structure
struct PIDConfig
{
    // Roll PID parameters (for roll stabilization)
    double roll_kp = ROLL_KP;         // Proportional gain from constants
    double roll_ki = ROLL_KI;         // Integral gain from constants
    double roll_kd = ROLL_KD;         // Derivative gain from constants
    double roll_filter = ROLL_FILTER; // Derivative filter coefficient
    double roll_smooth = ROLL_SMOOTH; // Setpoint smoothing

    // Pitch PID parameters (for pitch stabilization)
    double pitch_kp = PITCH_KP;         // Proportional gain from constants
    double pitch_ki = PITCH_KI;         // Integral gain from constants
    double pitch_kd = PITCH_KD;         // Derivative gain from constants
    double pitch_filter = PITCH_FILTER; // Derivative filter coefficient
    double pitch_smooth = PITCH_SMOOTH; // Setpoint smoothing

    // Yaw PID parameters (for yaw rate control)
    double yaw_kp = YAW_KP;         // Proportional gain from constants
    double yaw_ki = YAW_KI;         // Integral gain from constants
    double yaw_kd = YAW_KD;         // Derivative gain from constants
    double yaw_filter = YAW_FILTER; // More filtering for yaw (noisier)
    double yaw_smooth = YAW_SMOOTH; // More smoothing for yaw commands

    // Altitude PID parameters (for altitude hold)
    double alt_kp = ALT_KP;         // Altitude hold proportional from constants
    double alt_ki = ALT_KI;         // Integral gain from constants
    double alt_kd = ALT_KD;         // Derivative gain from constants
    double alt_filter = ALT_FILTER; // Light filtering for altitude
    double alt_smooth = ALT_SMOOTH; // Gentle altitude setpoint changes

    // Advanced tuning parameters
    double max_angle = MAX_ANGLE;           // Maximum angle setpoint from constants
    double max_yaw_rate = MAX_YAW_RATE;     // Maximum yaw rate from constants
    double max_climb_rate = MAX_CLIMB_RATE; // Maximum climb rate from constants
};

// FreeRTOS Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t radioTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t pidTaskHandle = NULL;

// FreeRTOS Synchronization
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t telemetryMutex;
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t controlMutex;

// Shared data structures (protected by mutexes)
ControlPacket receivedControl;
TelemetryPacket telemetryData;
unsigned long lastControlReceived = 0;
int packetsReceived = 0;
bool sensorsInitialized = false;
bool bh1750Ready = false;
bool ens160Ready = false;

// Motor control variables
volatile bool motorsArmed = false;
volatile bool stabilizedMode = false; // Flight mode: true = Stabilized (PID), false = Manual
volatile int motorSpeeds[4] = {ESC_ARM_PULSE, ESC_ARM_PULSE, ESC_ARM_PULSE, ESC_ARM_PULSE};
unsigned long lastValidControl = 0;
#define CONTROL_TIMEOUT_MS 1000   // 1 second timeout for safety
#define MIN_THROTTLE_FOR_ARM 1100 // Minimum throttle to consider arming
#define MOTOR_TASK_STACK 4096
#define MOTOR_TASK_PRIORITY 4 // High priority for motor control

// Altitude calibration variables
#define PRESSURE_BUFFER_SIZE 20   // Increased buffer for better smoothing
float seaLevelPressure = 1013.25; // Will be calibrated at startup
bool altitudeCalibrated = false;
float pressureReadings[PRESSURE_BUFFER_SIZE]; // For smoothing pressure readings
int pressureIndex = 0;
int pressureCount = 0;

// Enhanced altitude stability
float referenceTemperature = 15.0;   // Reference temperature for compensation (not used for drift correction anymore)
float exponentialPressure = 1013.25; // Exponentially smoothed pressure
float altitudeOffset = 0.0;          // Zero-reference offset
unsigned long lastAltitudeCalibration = 0;
const float PRESSURE_SMOOTHING_ALPHA = 0.1; // Exponential smoothing factor
// When disarmed, bleed any baro drift by slowly re-zeroing altitude baseline
const float ALTITUDE_BASELINE_ZERO_ALPHA = 0.02f; // 0..1 per fast baro update; higher = faster re-zero

// Task timing variables
unsigned long sensorTaskCounter = 0;
unsigned long radioTaskCounter = 0;
unsigned long statusTaskCounter = 0;

// PID Controllers and IMU Data
PIDController rollPID, pitchPID, yawPID, altitudePID;
PIDConfig pidConfig;
IMUData imuData;
FlightMode currentFlightMode = FLIGHT_MODE_DISARMED;

// Optional inner-loop Rate PID controllers (used when USE_CASCADED_ANGLE_RATE=true)
PIDController rollRatePID, pitchRatePID;

// 1D Kalman filter state for roll/pitch (used when USE_KALMAN_ATTITUDE=true)
struct Kalman1D
{
    float angle;       // estimated angle
    float uncertainty; // covariance
};
Kalman1D kalmanRoll{0.0f, 4.0f};
Kalman1D kalmanPitch{0.0f, 4.0f};

// IMU Calibration Data
struct IMUCalibration
{
    float rollOffset = 0.0;
    float pitchOffset = 0.0;
    float yawOffset = 0.0;
    float gyroXOffset = 0.0;
    float gyroYOffset = 0.0;
    float gyroZOffset = 0.0;
    bool calibrated = false;
    int calibrationSamples = 0;
} imuCalibration;

// Yaw drift correction variables
float gyroZBiasSum = 0.0;
int gyroBiasSampleCount = 0;
const int GYRO_BIAS_SAMPLES = 1000; // Number of samples for bias calculation
float adaptiveGyroZBias = 0.0;      // Adaptive bias for yaw drift correction
unsigned long lastYawReset = 0;
const float YAW_LEAKY_ALPHA = 0.9999; // Leaky integrator coefficient (very close to 1)
bool isStationary = false;            // Flag to determine if drone is stationary
unsigned long stationaryStartTime = 0;
const unsigned long STATIONARY_THRESHOLD_TIME = 5000; // 5 seconds to consider stationary

// PID and Flight Control Variables
bool pidEnabled = false;
bool imuInitialized = false;
bool enableVerboseLogging = false;   // Debug verbose logging control
float targetAltitude = 0.0;          // Target altitude for altitude hold mode
float baseThrottleForHover = 1400.0; // Base throttle for hovering (will be auto-adjusted)
unsigned long lastPIDUpdate = 0;
SemaphoreHandle_t pidMutex;
SemaphoreHandle_t imuMutex;

// Enhanced motor control with PID integration
volatile float pidRollOutput = 0.0;
volatile float pidPitchOutput = 0.0;
volatile float pidYawOutput = 0.0;
volatile float pidAltitudeOutput = 0.0;

// ================================
// QUADCOPTER MOTOR MIXING GAINS (tunable)
// These gains represent the maximum microsecond contribution for full-scale (|R|=|P|=|Y|=1.0) inputs
// Adjust to tune authority of each axis.
// k_r: roll, k_p: pitch, k_y: yaw
// Matches prior manual mapping (~±200 roll/pitch, ±150 yaw)
float MIX_KR = 100.0f; // Roll gain (μs per full command)
float MIX_KP = 100.0f; // Pitch gain (μs per full command)
float MIX_KY = 75.0f;  // Yaw gain (μs per full command)

// PWM bounds (fallback to existing defines if present)
#ifndef PWM_MIN_PULSE
#define PWM_MIN_PULSE ESC_ARM_PULSE
#endif
#ifndef PWM_MAX_PULSE
#define PWM_MAX_PULSE ESC_MAX_PULSE
#endif

// ================================
// Motor Calibration Offsets (μs)
// Apply small per-motor corrections to compensate hardware imbalances.
// Positive values increase that motor's command; defaults zero.
// Applied AFTER mixing in all modes (manual and stabilized), same as v2.
int MOTOR1_OFFSET_US = 0; // Front Right (CCW)
int MOTOR2_OFFSET_US = 0; // Back  Right (CW)
int MOTOR3_OFFSET_US = 0; // Front Left  (CW)
int MOTOR4_OFFSET_US = 0; // Back  Left  (CCW)

// ================================
// Scaled Motor Mixing Function (Implements user-specified algorithm)
// Inputs:
//   T = base throttle in microseconds (1000..2000)
//   R, P, Y = normalized control inputs in range [-1.0, 1.0]
// Output:
//   Populates motorSpeeds[4] respecting bounds with dynamic scaling to prevent saturation
// Motor Layout:
//   M1 = Front Right (CCW) -> esc1 (GPIO 13)
//   M2 = Back  Right (CW)  -> esc2 (GPIO 12)
//   M3 = Front Left (CW)   -> esc3 (GPIO 14)
//   M4 = Back  Left (CCW)  -> esc4 (GPIO 27)
// Mixing (per requirement):
//   M1 mix = -k_r*R - k_p*P + k_y*Y
//   M2 mix = -k_r*R + k_p*P - k_y*Y
//   M3 mix = +k_r*R - k_p*P - k_y*Y
//   M4 mix = +k_r*R + k_p*P + k_y*Y
// Dynamic scaling:
//   posMax = max positive mix; negMin = most negative mix
//   s_pos = min(1, (PWM_max - T)/posMax) if posMax>0 else 1
//   s_neg = min(1, (T - PWM_min)/(-negMin)) if negMin<0 else 1
//   final = T + s_pos*positive_mix + s_neg*negative_mix
//   Clamp to [PWM_MIN_PULSE, PWM_MAX_PULSE]
static inline void applyScaledMotorMix(float T, float R, float P, float Y)
{
    // Constrain base throttle
    if (T < PWM_MIN_PULSE)
        T = PWM_MIN_PULSE;
    if (T > PWM_MAX_PULSE)
        T = PWM_MAX_PULSE;

    // Raw mixes (no throttle added yet)
    float m1 = -MIX_KR * R - MIX_KP * P + MIX_KY * Y; // Front Right (CCW)
    float m2 = -MIX_KR * R + MIX_KP * P - MIX_KY * Y; // Back Right (CW)
    float m3 = MIX_KR * R - MIX_KP * P - MIX_KY * Y;  // Front Left (CW)
    float m4 = MIX_KR * R + MIX_KP * P + MIX_KY * Y;  // Back Left (CCW)

    // Determine scaling needed
    float posMax = 0.0f;
    float negMin = 0.0f;
    if (m1 > posMax)
        posMax = m1;
    if (m1 < negMin)
        negMin = m1;
    if (m2 > posMax)
        posMax = m2;
    if (m2 < negMin)
        negMin = m2;
    if (m3 > posMax)
        posMax = m3;
    if (m3 < negMin)
        negMin = m3;
    if (m4 > posMax)
        posMax = m4;
    if (m4 < negMin)
        negMin = m4;

    float s_pos = 1.0f;
    float s_neg = 1.0f;
    if (posMax > 0.0f)
    {
        float headroomHigh = (float)PWM_MAX_PULSE - T;
        if (headroomHigh < 0)
            headroomHigh = 0;
        s_pos = headroomHigh / posMax;
        if (s_pos > 1.0f)
            s_pos = 1.0f;
    }
    if (negMin < 0.0f)
    {
        float headroomLow = T - (float)PWM_MIN_PULSE;
        if (headroomLow < 0)
            headroomLow = 0;
        s_neg = headroomLow / (-negMin);
        if (s_neg > 1.0f)
            s_neg = 1.0f;
    }

    // Apply scaled mixes
    float f1 = T + (m1 >= 0 ? s_pos * m1 : s_neg * m1);
    float f2 = T + (m2 >= 0 ? s_pos * m2 : s_neg * m2);
    float f3 = T + (m3 >= 0 ? s_pos * m3 : s_neg * m3);
    float f4 = T + (m4 >= 0 ? s_pos * m4 : s_neg * m4);

    // Clamp and assign
    if (f1 < PWM_MIN_PULSE)
        f1 = PWM_MIN_PULSE;
    if (f1 > PWM_MAX_PULSE)
        f1 = PWM_MAX_PULSE;
    if (f2 < PWM_MIN_PULSE)
        f2 = PWM_MIN_PULSE;
    if (f2 > PWM_MAX_PULSE)
        f2 = PWM_MAX_PULSE;
    if (f3 < PWM_MIN_PULSE)
        f3 = PWM_MIN_PULSE;
    if (f3 > PWM_MAX_PULSE)
        f3 = PWM_MAX_PULSE;
    if (f4 < PWM_MIN_PULSE)
        f4 = PWM_MIN_PULSE;
    if (f4 > PWM_MAX_PULSE)
        f4 = PWM_MAX_PULSE;

    motorSpeeds[0] = (int)f1; // Front Right (CCW)
    motorSpeeds[1] = (int)f2; // Back Right (CW)
    motorSpeeds[2] = (int)f3; // Front Left (CW)
    motorSpeeds[3] = (int)f4; // Back Left (CCW)
}

// Function declarations
void updateFlightMode();
float getFilteredAltitude();
void updateBaroAltitude();

void setup()
{
    // ESCs already calibrated to 1000-2000µs range (one-time manual calibration done)
    // Simplified startup: attach and drive all ESCs to minimum (safe) immediately.

    // Configure servo library timers for ESCs first
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Attach ESCs to pins immediately
    esc1.setPeriodHertz(ESC_FREQUENCY);
    esc2.setPeriodHertz(ESC_FREQUENCY);
    esc3.setPeriodHertz(ESC_FREQUENCY);
    esc4.setPeriodHertz(ESC_FREQUENCY);

    bool escAttachSuccess = true;
    escAttachSuccess &= esc1.attach(ESC1_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escAttachSuccess &= esc2.attach(ESC2_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escAttachSuccess &= esc3.attach(ESC3_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escAttachSuccess &= esc4.attach(ESC4_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);

    // Initialize serial
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize

    if (!escAttachSuccess)
    {
        Serial.println("⚠️ WARNING: Some ESCs failed to attach properly!");
    }

    Serial.println("=== Drone with Real Sensors - FreeRTOS Version ===");
    Serial.println("ESC calibration skipped (pre-calibrated). Setting all motors to min.");

    // Send minimum (arming) pulse to all ESCs immediately for safety
    esc1.writeMicroseconds(ESC_ARM_PULSE);
    esc2.writeMicroseconds(ESC_ARM_PULSE);
    esc3.writeMicroseconds(ESC_ARM_PULSE);
    esc4.writeMicroseconds(ESC_ARM_PULSE);

    // Initialize motor speeds to safe values
    for (int i = 0; i < 4; i++)
        motorSpeeds[i] = ESC_ARM_PULSE;
    motorsArmed = false;

    Serial.println("Initializing system components...");

    // Initialize I2C for sensors
    Wire.begin(21, 22); // SDA=21, SCL=22
    // Keep bus at 100kHz for broad sensor compatibility (some boards misbehave at 400kHz)
    Wire.setClock(100000);
    delay(100);

    // Initialize pins
    pinMode(BATTERY_PIN, INPUT);
    pinMode(GUVA_PIN, INPUT); // UV sensor analog input
    // Configure ADC attenuation for accurate battery measurements up to ~3.3V
    analogSetAttenuation(ADC_11db);
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

    // Initialize GPS (Serial2 on ESP32)
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // Create FreeRTOS mutexes
    i2cMutex = xSemaphoreCreateMutex();
    telemetryMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
    controlMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();
    imuMutex = xSemaphoreCreateMutex();

    if (i2cMutex == NULL || telemetryMutex == NULL || serialMutex == NULL ||
        controlMutex == NULL || pidMutex == NULL || imuMutex == NULL)
    {
        Serial.println("ERROR: Failed to create mutexes!");
        while (1)
            delay(1000);
    }

    // Initialize enhanced stability variables
    exponentialPressure = seaLevelPressure;
    altitudeOffset = 0.0;
    adaptiveGyroZBias = 0.0;
    gyroZBiasSum = 0.0;
    gyroBiasSampleCount = 0;
    isStationary = false;
    lastYawReset = millis();
    lastAltitudeCalibration = millis();

    Serial.println("✓ Enhanced stability systems initialized");

    // Initialize real sensors
    initializeRealSensors();

    // Initialize IMU and PID controllers (IMU calibration will run here for faster startup)
    initializeIMUAndPID();

    // Perform IMU calibration at initialization (fast, before tasks start)
    if (imuInitialized)
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            Serial.println("Starting IMU calibration (startup)... Keep drone level and still.");
            xSemaphoreGive(serialMutex);
        }

        // Collect a quicker set of samples at ~500 Hz for ~1s
        const int CAL_SAMPLES = 500;
        float gx = 0, gy = 0, gz = 0, rSum = 0, pSum = 0;
        int samples = 0;
        unsigned long start = millis();
        while (samples < CAL_SAMPLES && millis() - start < 2000)
        {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                sensors_event_t accel, gyro, temp;
                if (mpu6050.getEvent(&accel, &gyro, &temp))
                {
                    // 90° CW rotation about Z: bodyX=sensorY, bodyY=-sensorX
                    float bodyAccelX = accel.acceleration.y;
                    float bodyAccelY = -accel.acceleration.x;
                    float bodyAccelZ = accel.acceleration.z;
                    float roll_acc = atan2(bodyAccelY, bodyAccelZ) * 180.0f / PI;
                    float pitch_acc = atan2(-bodyAccelX, sqrt(bodyAccelY * bodyAccelY + bodyAccelZ * bodyAccelZ)) * 180.0f / PI;
                    // Gyro mapping: rollRate=gyroY, pitchRate=-gyroX
                    gx += gyro.gyro.y * 180.0f / PI;
                    gy += (-gyro.gyro.x) * 180.0f / PI;
                    gz += gyro.gyro.z * 180.0f / PI;
                    rSum += roll_acc;
                    pSum += pitch_acc;
                    samples++;
                }
                xSemaphoreGive(i2cMutex);
            }
            delay(2);
        }

        if (samples > 0)
        {
            imuCalibration.gyroXOffset = gx / samples;
            imuCalibration.gyroYOffset = gy / samples;
            imuCalibration.gyroZOffset = gz / samples;
            imuCalibration.rollOffset = rSum / samples;
            imuCalibration.pitchOffset = pSum / samples;
            imuCalibration.calibrated = true;

            // Seed attitude to level
            if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                imuData.roll = 0.0f;
                imuData.pitch = 0.0f;
                imuData.yaw = 0.0f;
                imuData.dataValid = true;
                xSemaphoreGive(imuMutex);
            }

            if (USE_KALMAN_ATTITUDE)
            {
                kalmanRoll.angle = 0.0f;
                kalmanRoll.uncertainty = 4.0f;
                kalmanPitch.angle = 0.0f;
                kalmanPitch.uncertainty = 4.0f;
            }

            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                Serial.printf("✓ IMU calibration done (startup). Gyro offsets: [%.2f, %.2f, %.2f], Level offsets: R=%.2f P=%.2f\n",
                              imuCalibration.gyroXOffset, imuCalibration.gyroYOffset, imuCalibration.gyroZOffset,
                              imuCalibration.rollOffset, imuCalibration.pitchOffset);
                xSemaphoreGive(serialMutex);
            }
        }
    }

    // Initialize radio with PROVEN configuration
    initializeRadio();

    // Initialize telemetry data with safe defaults
    initializeTelemetryData();

    Serial.println("Creating FreeRTOS tasks...");

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensorTaskHandle,
        0 // Core 0
    );

    xTaskCreatePinnedToCore(
        radioTask,
        "RadioTask",
        RADIO_TASK_STACK,
        NULL,
        RADIO_TASK_PRIORITY,
        &radioTaskHandle,
        1 // Core 1
    );

    xTaskCreatePinnedToCore(
        statusTask,
        "StatusTask",
        STATUS_TASK_STACK,
        NULL,
        STATUS_TASK_PRIORITY,
        &statusTaskHandle,
        0 // Core 0
    );

    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        MOTOR_TASK_STACK,
        NULL,
        MOTOR_TASK_PRIORITY,
        &motorTaskHandle,
        1 // Core 1 - dedicated core for motor control
    );

    // Create IMU Task (high frequency sensor reading)
    xTaskCreatePinnedToCore(
        imuTask,
        "IMUTask",
        4096,
        NULL,
        5, // Highest priority for IMU reading
        &imuTaskHandle,
        0 // Core 0
    );

    // Create PID Task (control loop)
    xTaskCreatePinnedToCore(
        pidControlTask,
        "PIDTask",
        4096,
        NULL,
        4, // High priority for PID control
        &pidTaskHandle,
        1 // Core 1 with motor task
    );

    // Check if tasks were created successfully
    if (sensorTaskHandle == NULL || radioTaskHandle == NULL || statusTaskHandle == NULL ||
        motorTaskHandle == NULL || imuTaskHandle == NULL || pidTaskHandle == NULL)
    {
        Serial.println("ERROR: Failed to create FreeRTOS tasks!");
        while (1)
            delay(1000);
    }

    Serial.println("✓ All FreeRTOS tasks created successfully");
    Serial.println("✓ System initialization complete");
    Serial.println("Starting multitasking operation...");
}

void loop()
{
    // FreeRTOS handles all tasks - main loop can be minimal
    // Print memory usage periodically
    static unsigned long lastMemCheck = 0;
    if (millis() - lastMemCheck > 30000) // Every 30 seconds
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("\n=== FreeRTOS Task Status ===");
            Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("Sensor task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(sensorTaskHandle));
            Serial.printf("Radio task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(radioTaskHandle));
            Serial.printf("Status task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(statusTaskHandle));
            Serial.printf("Motor task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(motorTaskHandle));
            Serial.printf("Task counters - Sensor: %lu, Radio: %lu, Status: %lu\n",
                          sensorTaskCounter, radioTaskCounter, statusTaskCounter);
            Serial.println("===========================\n");
            xSemaphoreGive(serialMutex);
        }
        lastMemCheck = millis();
    }

    // Yield to FreeRTOS scheduler
    vTaskDelay(pdMS_TO_TICKS(5000)); // Sleep for 5 seconds
}

void sensorTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Run at 20Hz for smoother altitude updates; schedule slower sensors separately
    const TickType_t loopFrequency = pdMS_TO_TICKS(50);  // 20Hz
    const TickType_t envFrequency = pdMS_TO_TICKS(1000); // 1Hz env sensors
    const TickType_t gpsFrequency = pdMS_TO_TICKS(100);  // 10Hz GPS

    TickType_t lastEnvRead = 0;
    TickType_t lastGPSRead = 0;

    for (;;)
    {
        sensorTaskCounter++;

        // Fast baro-only update every loop to keep altitude fresh for PID
        if (sensorsInitialized)
        {
            updateBaroAltitude();
        }

        // Read environmental sensors (1Hz)
        TickType_t nowTicks = xTaskGetTickCount();
        if ((nowTicks - lastEnvRead) >= envFrequency)
        {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                readEnvironmentalSensors();
                xSemaphoreGive(i2cMutex);
            }
            lastEnvRead = nowTicks;
        }

        // Read GPS more frequently (10Hz)
        if ((nowTicks - lastGPSRead) >= gpsFrequency)
        {
            updateGPS();
            lastGPSRead = nowTicks;
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, loopFrequency);
    }
}

void radioTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // Increased poll rate to 500Hz to further minimize input-to-motor latency
    const TickType_t radioFrequency = pdMS_TO_TICKS(2); // 500Hz radio check

    for (;;)
    {
        radioTaskCounter++;

        // Drain all incoming control data this cycle to minimize latency
        while (radio.available())
        {
            handleControlData();
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, radioFrequency);
    }
}

void statusTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t statusFrequency = pdMS_TO_TICKS(10000); // 0.1Hz (every 10 seconds)

    for (;;)
    {
        statusTaskCounter++;

        // Print system status
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            printStatus();
            xSemaphoreGive(serialMutex);
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, statusFrequency);
    }
}

void initializeRadio()
{
    Serial.println("Initializing RF24 radio...");

    SPI.begin();
    delay(100);

    if (!radio.begin() || !radio.isChipConnected())
    {
        Serial.println("ERROR: Radio initialization failed!");
        while (1)
            delay(1000);
    }

    // Use EXACT same configuration as remote
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_HIGH); // Match remote power level
    // Switched to 1MBPS for lower on-air time and reduced latency (both sides must match)
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(110); // Move away from common Wi-Fi channels to reduce interference
    radio.setAutoAck(true);
    radio.setRetries(3, 5); // Keep short retry delay/count for low worst-case latency
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    // Additional range improvements to match remote
    radio.setCRCLength(RF24_CRC_16); // Use 16-bit CRC for better error detection
    radio.setAddressWidth(5);        // Use full 5-byte addresses

    // Start listening for control data
    radio.startListening();

    Serial.println("✓ Radio configured successfully!");
    Serial.println("✓ Waiting for control data...");
}

void initializeTelemetryData()
{
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        telemetryData.temperature = 2500;  // 25.00°C
        telemetryData.pressureX10 = 10132; // 101.32 kPa -> 1013.2 hPa
        telemetryData.humidity = 60;       // 60%
        telemetryData.battery = 3700;      // 3.7V
        telemetryData.latitudeE7 = 0;
        telemetryData.longitudeE7 = 0;
        telemetryData.satellites = 0;
        telemetryData.status = 0x01; // System OK
        telemetryData.lux = 500;     // 500 lux
        telemetryData.altitude = 0;  // 0.00m
        telemetryData.uvIndex = 300; // UV index 3.00
        telemetryData.eCO2 = 400;    // 400 ppm
        telemetryData.TVOC = 50;     // 50 ppb
        xSemaphoreGive(telemetryMutex);
    }

    // Preload an initial ACK payload so the very first control packet can receive telemetry
    // Pipe 0 is the active reading pipe; preload there.
    TelemetryPacket initialAck;
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        initialAck = telemetryData;
        xSemaphoreGive(telemetryMutex);
        radio.writeAckPayload(0, &initialAck, sizeof(initialAck));
    }
}

void initializeRealSensors()
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.println("Initializing real sensors...");
        xSemaphoreGive(serialMutex);
    }

    // Take I2C mutex for sensor initialization
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
        // Scan I2C bus first
        scanI2CBus();

        // Try BME280 at common addresses
        bool bme280Found = false;
        uint8_t bmeAddresses[] = {0x76, 0x77};
        for (int i = 0; i < 2; i++)
        {
            if (bme280.begin(bmeAddresses[i]))
            {
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.print("✓ BME280 initialized at address 0x");
                    Serial.println(bmeAddresses[i], HEX);
                    xSemaphoreGive(serialMutex);
                }

                // Configure BME280 for stable readings
                bme280.setSampling(Adafruit_BME280::MODE_FORCED,      // Use forced mode for manual readings
                                   Adafruit_BME280::SAMPLING_X16,     // Temperature oversampling
                                   Adafruit_BME280::SAMPLING_X16,     // Pressure oversampling
                                   Adafruit_BME280::SAMPLING_X16,     // Humidity oversampling
                                   Adafruit_BME280::FILTER_X16,       // IIR filter
                                   Adafruit_BME280::STANDBY_MS_1000); // 1 second standby

                sensorsInitialized = true;
                bme280Found = true;
                break;
            }
        }

        if (!bme280Found)
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ BME280 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
        }

        // Initialize AHT21
        if (aht.begin())
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✓ AHT21 initialized");
                xSemaphoreGive(serialMutex);
            }
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ AHT21 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
        }

        // Initialize BH1750 light sensor
        if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✓ BH1750 light sensor initialized");
                xSemaphoreGive(serialMutex);
            }
            bh1750Ready = true;
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ BH1750 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
            bh1750Ready = false;
        }

        // Initialize ENS160 air quality sensor with warm-up, address probe, and retries
        {
            // Brief warm-up before talking to ENS160 after bus activity
            vTaskDelay(pdMS_TO_TICKS(100));

            auto i2cProbe = [](uint8_t addr) -> bool
            {
                Wire.beginTransmission(addr);
                return (Wire.endTransmission() == 0);
            };

            uint8_t ensAddr = 0x53; // ENS160_I2CADDR_1
            if (!i2cProbe(ensAddr))
            {
                // Try alternate address used by some modules
                if (i2cProbe(0x52))
                {
                    ensAddr = 0x52; // ENS160_I2CADDR_2
                }
            }

            bool ok = false;
            for (int attempt = 0; attempt < 3 && !ok; ++attempt)
            {
                // Some library variants support begin(TwoWire&, uint8_t). Fall back to default begin() if not.
                // We guard with a simple probe to avoid spamming the bus if no device is present.
                if (i2cProbe(ensAddr))
                {
                    // Try to init; if your library lacks this overload, default begin() will be used by the linker.
                    ok = ens160.begin();
                }
                if (!ok)
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (ok)
            {
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.print("✓ ENS160 air quality sensor initialized at 0x");
                    Serial.println(ensAddr, HEX);
                    xSemaphoreGive(serialMutex);
                }
                // Give sensor a moment, then set standard operating mode
                ens160.setMode(ENS160_OPMODE_STD);
                ens160Ready = true;
            }
            else
            {
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.println("✗ ENS160 initialization failed - using simulated data");
                    xSemaphoreGive(serialMutex);
                }
                ens160Ready = false;
            }
        }

        xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("✓ GPS Serial2 initialized");
        Serial.println("✓ GUVA-S12SD UV sensor on analog pin");
        xSemaphoreGive(serialMutex);
    }

    // Calibrate altitude (assume current location is reference point)
    calibrateAltitude();

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("✓ Sensor initialization complete");
        xSemaphoreGive(serialMutex);
    }
}

void scanI2CBus()
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.println("Scanning I2C bus...");
        xSemaphoreGive(serialMutex);
    }

    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.print("I2C device found at address 0x");
                if (address < 16)
                    Serial.print("0");
                Serial.println(address, HEX);
                xSemaphoreGive(serialMutex);
            }
            nDevices++;
        }
    }

    if (nDevices == 0)
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("No I2C devices found");
            xSemaphoreGive(serialMutex);
        }
    }
}

void calibrateAltitude()
{
    if (!sensorsInitialized)
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("⚠ BME280 not available - altitude will use default sea level pressure");
            xSemaphoreGive(serialMutex);
        }
        return;
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("Calibrating altitude reference...");
        xSemaphoreGive(serialMutex);
    }

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Take multiple pressure readings for calibration
    float pressureSum = 0;
    float tempSum = 0;
    int validReadings = 0;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
        for (int i = 0; i < 30; i++)
        {
            // Force a reading and wait
            bme280.takeForcedMeasurement();
            vTaskDelay(pdMS_TO_TICKS(100));

            float temperature = bme280.readTemperature();
            float pressure = bme280.readPressure() / 100.0; // Convert Pa to hPa
            if (pressure > 800 && pressure < 1200)          // Reasonable pressure range
            {
                pressureSum += pressure;
                tempSum += temperature;
                validReadings++;
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    Serial.print(".");
                    xSemaphoreGive(serialMutex);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println();
        xSemaphoreGive(serialMutex);
    }

    if (validReadings > 20)
    {
        // Establish reference pressure from averaged samples
        seaLevelPressure = pressureSum / validReadings;
        float avgTemp = tempSum / validReadings;

        // Prime smoothing with calibrated pressure to avoid initial jumps/drift
        exponentialPressure = seaLevelPressure;
        for (int i = 0; i < PRESSURE_BUFFER_SIZE; ++i)
        {
            pressureReadings[i] = seaLevelPressure;
        }
        pressureIndex = 0;
        pressureCount = PRESSURE_BUFFER_SIZE;

        // Reference temperature stored for telemetry only
        referenceTemperature = avgTemp;

        // Zero altitude at startup relative to current pressure
        altitudeOffset = 44330.0f * (1.0f - pow(seaLevelPressure / seaLevelPressure, 0.1903f)); // 0

        altitudeCalibrated = true;
        lastAltitudeCalibration = millis();

        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.print("✓ Altitude calibrated - Ref P: ");
            Serial.print(seaLevelPressure, 2);
            Serial.print(" hPa, Ref T: ");
            Serial.print(referenceTemperature, 2);
            Serial.println("°C (altitude zeroed)");
            xSemaphoreGive(serialMutex);
        }
    }
    else
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("✗ Altitude calibration failed - using standard sea level pressure");
            xSemaphoreGive(serialMutex);
        }
    }
}

void readEnvironmentalSensors()
{
    // Create local copy to minimize mutex lock time
    TelemetryPacket localTelemetry;

    // Read BME280 (temperature and pressure)
    if (sensorsInitialized)
    {
        // Force a measurement for consistent timing
        bme280.takeForcedMeasurement();

        float temp = bme280.readTemperature();
        float currentPressure = bme280.readPressure() / 100.0; // Convert Pa to hPa

        localTelemetry.temperature = (int16_t)(temp * 100);
        localTelemetry.pressureX10 = (uint16_t)(currentPressure * 10);

        // Validate pressure reading
        if (currentPressure > 800 && currentPressure < 1200)
        {
            // Add to pressure buffer for simple smoothing
            pressureReadings[pressureIndex] = currentPressure;
            pressureIndex = (pressureIndex + 1) % PRESSURE_BUFFER_SIZE;
            if (pressureCount < PRESSURE_BUFFER_SIZE)
                pressureCount++;

            // Calculate simple moving average
            float pressureSum = 0;
            for (int i = 0; i < pressureCount; i++)
            {
                pressureSum += pressureReadings[i];
            }
            float smoothedPressure = pressureSum / pressureCount;

            // Apply exponential smoothing for additional stability
            exponentialPressure = (PRESSURE_SMOOTHING_ALPHA * smoothedPressure) +
                                  ((1.0 - PRESSURE_SMOOTHING_ALPHA) * exponentialPressure);

            // Calculate altitude using smoothed pressure only (temperature compensation removed to avoid drift)
            float altitude;
            if (altitudeCalibrated)
            {
                // Use calibrated reference for relative altitude
                altitude = 44330.0 * (1.0 - pow(exponentialPressure / seaLevelPressure, 0.1903));
            }
            else
            {
                // Use standard sea level pressure
                altitude = 44330.0 * (1.0 - pow(exponentialPressure / 1013.25, 0.1903));
            }

            // Apply altitude offset for zero-reference
            altitude -= altitudeOffset;

            // Debug output for pressure and altitude calculation (less frequent)
            static unsigned long lastDebug = 0;
            static int debugCounter = 0;
            if (millis() - lastDebug > 5000) // Debug every 5 seconds
            {
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.print("DEBUG - P: ");
                    Serial.print(currentPressure, 2);
                    Serial.print("/");
                    Serial.print(exponentialPressure, 2);
                    Serial.print(" hPa, Ref: ");
                    Serial.print(seaLevelPressure, 2);
                    Serial.print(" hPa, Alt: ");
                    Serial.print(altitude, 2);
                    Serial.print(" m, Temp: ");
                    Serial.print(temp, 2);
                    Serial.print("°C, YawBias: ");
                    Serial.print(adaptiveGyroZBias, 3);
                    Serial.print(", Stationary: ");
                    Serial.println(isStationary ? "YES" : "NO");
                    xSemaphoreGive(serialMutex);
                }
                lastDebug = millis();
                debugCounter++;
            }

            // Simple range limiting
            if (altitude < -500)
                altitude = -500;
            if (altitude > 5000)
                altitude = 5000;

            // Store altitude in centimeters for 2 decimal precision
            localTelemetry.altitude = (int16_t)(altitude * 100); // Convert meters to centimeters
        }
        else
        {
            // Invalid pressure reading - keep current altitude and previous pressure
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                localTelemetry.altitude = telemetryData.altitude;
                localTelemetry.pressureX10 = telemetryData.pressureX10;
                xSemaphoreGive(telemetryMutex);
            }
        }
    }
    else
    {
        // Fallback to simulated data
        static float temp = 25.0;
        temp += (random(-10, 11) / 100.0);
        localTelemetry.temperature = (int16_t)(temp * 100);

        static float pressure = 1013.20;
        pressure += (random(-5, 6) / 100.0); // simulate ~0.01 hPa steps
        localTelemetry.pressureX10 = (uint16_t)(pressure * 10);

        localTelemetry.altitude = 10000 + random(-2000, 2001); // Simulated altitude in cm
    }

    // Read AHT21 (humidity and temperature)
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp))
    {
        localTelemetry.humidity = (uint8_t)humidity.relative_humidity;
        // Use AHT21 temperature if BME280 failed
        if (!sensorsInitialized)
        {
            localTelemetry.temperature = (int16_t)(temp.temperature * 100);
        }
    }
    else
    {
        // Fallback to simulated humidity
        localTelemetry.humidity = 60 + random(-10, 11);
    }

    // Read real battery voltage with scaling for divider ratio (0.2 => x5)
    // Average a few samples to reduce noise
    const int BATTERY_SAMPLES = 8;
    uint32_t sumRaw = 0;
    for (int i = 0; i < BATTERY_SAMPLES; ++i)
    {
        sumRaw += analogRead(BATTERY_PIN);
        delayMicroseconds(200);
    }
    float batteryRawAvg = sumRaw / (float)BATTERY_SAMPLES;
    // Convert ADC counts to volts at ADC pin
    const float ADC_REF_V = 3.3f;     // ESP32 ADC reference scaling (approx)
    const float ADC_COUNTS = 4095.0f; // 12-bit ADC
    const float DIVIDER_RATIO = 0.2f; // Vout = Vin * 0.2 (R1=30k, R2=7.5k)
    float vOut = (batteryRawAvg / ADC_COUNTS) * ADC_REF_V;
    float batteryVolts = vOut / DIVIDER_RATIO; // Scale back to actual battery voltage
    // Basic sanity clamp (3.0V..16.8V typical for 3S)
    if (batteryVolts < 3.0f || batteryVolts > 20.0f)
    {
        // If reading seems invalid, keep previous or set a safe simulated value
        localTelemetry.battery = 3700 + random(-100, 101);
    }
    else
    {
        localTelemetry.battery = (uint16_t)constrain((int)(batteryVolts * 1000.0f + 0.5f), 0, 65535); // mV
    }

    // Read real light sensor (BH1750)
    if (bh1750Ready)
    {
        float lux = lightMeter.readLightLevel();
        if (lux >= 0)
        {
            localTelemetry.lux = (uint16_t)lux;
        }
        else
        {
            localTelemetry.lux = 500 + random(-100, 101); // Fallback simulated
        }
    }
    else
    {
        localTelemetry.lux = 500 + random(-100, 101); // Simulated if sensor not ready
    }

    // Read real UV sensor (GUVA-S12SD)
    int uvRaw = analogRead(GUVA_PIN);
    // Convert to UV index (calibration may need adjustment based on your setup)
    float uvVoltage = (uvRaw / 4095.0) * 3.3;
    float uvIndex = uvVoltage / 0.1; // Approximate conversion for GUVA-S12SD
    if (uvIndex < 0)
        uvIndex = 0; // Clamp to positive values
    if (uvIndex > 15)
        uvIndex = 15; // Reasonable UV index maximum
    localTelemetry.uvIndex = (uint16_t)(uvIndex * 100);

    // Read real air quality sensor (ENS160)
    if (ens160Ready && ens160.available())
    {
        ens160.measure();
        uint16_t eco2 = ens160.geteCO2();
        uint16_t tvoc = ens160.getTVOC();

        // Validate readings (ENS160 sometimes gives invalid values during warm-up)
        if (eco2 > 400 && eco2 < 5000) // Reasonable CO2 range
        {
            localTelemetry.eCO2 = eco2;
        }
        else
        {
            localTelemetry.eCO2 = 400 + random(-50, 51); // Fallback
        }

        if (tvoc < 1000) // Reasonable TVOC range
        {
            localTelemetry.TVOC = tvoc;
        }
        else
        {
            localTelemetry.TVOC = 50 + random(-20, 21); // Fallback
        }
    }
    else
    {
        // Fallback to simulated values
        localTelemetry.eCO2 = 400 + random(-50, 51);
        localTelemetry.TVOC = 50 + random(-20, 21);
    }

    // Update status
    localTelemetry.status = 0x01; // All systems OK
    if (!sensorsInitialized)
        localTelemetry.status |= 0x02; // Sensor warning

    // Get GPS data from shared telemetry (GPS is updated in its own function)
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        // Copy high precision GPS values directly
        localTelemetry.latitudeE7 = telemetryData.latitudeE7;
        localTelemetry.longitudeE7 = telemetryData.longitudeE7;
        localTelemetry.satellites = telemetryData.satellites;

        // Update all telemetry data atomically
        telemetryData = localTelemetry;
        xSemaphoreGive(telemetryMutex);
    }
}

// Fast BME280-only update for pressure/temperature/altitude (runs ~20Hz)
void updateBaroAltitude()
{
    if (!sensorsInitialized)
        return;

    // Force a measurement for consistent timing
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        bme280.takeForcedMeasurement();
        float temp = bme280.readTemperature();
        float currentPressure = bme280.readPressure() / 100.0; // Pa -> hPa
        xSemaphoreGive(i2cMutex);

        if (currentPressure > 800 && currentPressure < 1200)
        {
            // Update smoothing buffers
            pressureReadings[pressureIndex] = currentPressure;
            pressureIndex = (pressureIndex + 1) % PRESSURE_BUFFER_SIZE;
            if (pressureCount < PRESSURE_BUFFER_SIZE)
                pressureCount++;

            float pressureSum = 0;
            for (int i = 0; i < pressureCount; i++)
            {
                pressureSum += pressureReadings[i];
            }
            float smoothedPressure = pressureSum / max(1, pressureCount);
            exponentialPressure = (PRESSURE_SMOOTHING_ALPHA * smoothedPressure) +
                                  ((1.0 - PRESSURE_SMOOTHING_ALPHA) * exponentialPressure);

            // Compute altitude without aggressive temperature compensation
            float altitudeRaw;
            if (altitudeCalibrated)
                altitudeRaw = 44330.0f * (1.0f - pow(exponentialPressure / seaLevelPressure, 0.1903f));
            else
                altitudeRaw = 44330.0f * (1.0f - pow(exponentialPressure / 1013.25f, 0.1903f));

            // When disarmed, slowly re-zero baseline to eliminate drift
            if (!motorsArmed)
            {
                altitudeOffset += ALTITUDE_BASELINE_ZERO_ALPHA * (altitudeRaw - altitudeOffset);
            }

            float altitude = altitudeRaw - altitudeOffset;

            // Clamp and store (temperature, pressure, altitude)
            if (altitude < -500)
                altitude = -500;
            if (altitude > 5000)
                altitude = 5000;

            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                telemetryData.temperature = (int16_t)(temp * 100);
                telemetryData.pressureX10 = (uint16_t)(currentPressure * 10);
                telemetryData.altitude = (int16_t)(altitude * 100); // meters -> cm
                xSemaphoreGive(telemetryMutex);
            }
        }
    }
}

void updateGPS()
{
    // Read GPS data from Serial2
    while (Serial2.available() > 0)
    {
        if (gps.encode(Serial2.read()))
        {
            if (gps.location.isValid())
            {
                // Update GPS data in shared telemetry
                if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    telemetryData.latitudeE7 = (int32_t)(gps.location.lat() * 10000000.0);
                    telemetryData.longitudeE7 = (int32_t)(gps.location.lng() * 10000000.0);
                    telemetryData.satellites = gps.satellites.value();
                    xSemaphoreGive(telemetryMutex);
                }

                // Capture GPS altitude (meters) when valid
                if (gps.altitude.isValid())
                {
                    gpsAltitudeMeters = gps.altitude.meters();
                }
            }
        }
    }

    // If no GPS fix, use default values
    if (!gps.location.isValid())
    {
        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            telemetryData.latitudeE7 = 0;
            telemetryData.longitudeE7 = 0;
            telemetryData.satellites = 0;
            xSemaphoreGive(telemetryMutex);
        }
        // Invalidate GPS altitude when fix lost
        gpsAltitudeMeters = NAN;
    }
}

void handleControlData()
{
    uint8_t pipeNo = 0;
    if (radio.available(&pipeNo))
    {
        uint8_t len = radio.getDynamicPayloadSize();

        if (len == sizeof(ControlPacket))
        {
            ControlPacket localControl;
            radio.read(&localControl, sizeof(localControl));

            // Update shared control data (for telemetry display)
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                lastControlReceived = millis();
                packetsReceived++;
                xSemaphoreGive(telemetryMutex);
            }

            // Update control data for motor task (separate mutex for higher priority)
            if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                receivedControl = localControl;
                lastValidControl = millis(); // Update motor control timeout
                xSemaphoreGive(controlMutex);
            }

            // Optional verbose debug (throttled) to avoid serial stalls affecting control latency
            if (enableVerboseLogging)
            {
                static uint32_t lastPrintMs = 0;
                if (millis() - lastPrintMs > 200) // 5 Hz logging max when enabled
                {
                    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) == pdTRUE)
                    {
                        Serial.print("Control #");
                        Serial.print(packetsReceived);
                        Serial.print(" T:");
                        Serial.print(localControl.throttle);
                        Serial.print(" R:");
                        Serial.print(localControl.roll);
                        Serial.print(" P:");
                        Serial.print(localControl.pitch);
                        Serial.print(" Y:");
                        Serial.println(localControl.yaw);
                        xSemaphoreGive(serialMutex);
                    }
                    lastPrintMs = millis();
                }
            }

            // Load telemetry data as ACK payload for NEXT packet
            // Build ACK payload quickly; if mutex contention, skip this cycle to keep input latency low
            TelemetryPacket ackTelemetry;
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                ackTelemetry = telemetryData;
                xSemaphoreGive(telemetryMutex);
                radio.writeAckPayload(pipeNo, &ackTelemetry, sizeof(ackTelemetry));
            }
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                Serial.print("Wrong packet size: ");
                Serial.println(len);
                xSemaphoreGive(serialMutex);
            }
        }
    }
}

void printStatus()
{
    unsigned long uptime = millis() / 1000;
    unsigned long lastControl;
    int packets;
    TelemetryPacket currentTelemetry;

    // Get shared data safely
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        lastControl = lastControlReceived;
        packets = packetsReceived;
        currentTelemetry = telemetryData;
        xSemaphoreGive(telemetryMutex);
    }

    bool controlActive = (millis() - lastControl) < 3000;

    Serial.print("Drone status [FreeRTOS+PID] - Running ");
    Serial.print(uptime);
    Serial.print("s, Control packets: ");
    Serial.print(packets);
    Serial.print(", Active: ");
    Serial.print(controlActive ? "YES" : "NO");

    // Flight mode and motor status
    Serial.print(", Mode: ");
    switch (currentFlightMode)
    {
    case FLIGHT_MODE_DISARMED:
        Serial.print("DISARMED");
        break;
    case FLIGHT_MODE_MANUAL:
        Serial.print("MANUAL");
        break;
    case FLIGHT_MODE_STABILIZE:
        Serial.print("STABILIZE");
        break;
    case FLIGHT_MODE_ALTITUDE_HOLD:
        Serial.print("ALT_HOLD");
        break;
    case FLIGHT_MODE_POSITION_HOLD:
        Serial.print("POS_HOLD");
        break;
    default:
        Serial.print("UNKNOWN");
        break;
    }

    Serial.print(", Motors: ");
    Serial.print(motorsArmed ? "ARMED" : "DISARMED");

    // Show flight mode
    Serial.print(stabilizedMode ? " [STABILIZED]" : " [MANUAL]");

    // IMU Status
    Serial.print(", IMU: ");
    if (imuInitialized && imuCalibration.calibrated)
    {
        if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            Serial.print("OK [R:");
            Serial.print(imuData.roll, 1);
            Serial.print("° P:");
            Serial.print(imuData.pitch, 1);
            Serial.print("° Y:");
            Serial.print(imuData.yaw, 1);
            Serial.print("°]");
            xSemaphoreGive(imuMutex);
        }
    }
    else if (imuInitialized && !imuCalibration.calibrated)
    {
        Serial.print("CALIBRATING(");
        Serial.print(imuCalibration.calibrationSamples);
        Serial.print("/500)");
    }
    else
    {
        Serial.print("FAILED");
    }

    // PID Status (simplified - just enabled/disabled)
    Serial.print(", PID: ");
    Serial.print(pidEnabled ? "ENABLED" : "DISABLED");

    // Environmental sensors
    Serial.print(", Sensors: Temp:");
    Serial.print(currentTelemetry.temperature / 100.0);
    Serial.print("°C Hum:");
    Serial.print(currentTelemetry.humidity);
    Serial.print("% Press:");
    Serial.print(currentTelemetry.pressureX10 / 10.0);
    Serial.print("hPa Alt:");
    Serial.print(currentTelemetry.altitude / 100.0); // Display altitude in meters with 2 decimals
    Serial.print("m GPS:");
    Serial.print(currentTelemetry.satellites);
    Serial.print(" sats Batt:");
    Serial.print(currentTelemetry.battery);
    Serial.print("mV Lux:");
    Serial.print(currentTelemetry.lux);
    Serial.print("lx UV:");
    Serial.print(currentTelemetry.uvIndex / 100.0);
    Serial.print(" CO2:");
    Serial.print(currentTelemetry.eCO2);
    Serial.print("ppm TVOC:");
    Serial.print(currentTelemetry.TVOC);
    Serial.println("ppb");
}

// Motor Control Task
void motorTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t motorFrequency = pdMS_TO_TICKS(5); // 200Hz motor update (5ms)

    for (;;)
    {
        // Check for control timeout (safety feature)
        unsigned long currentTime = millis();
        bool controlValid = (currentTime - lastValidControl) < CONTROL_TIMEOUT_MS;

        if (!controlValid)
        {
            // Safety: Stop all motors if no recent control
            for (int i = 0; i < 4; i++)
            {
                motorSpeeds[i] = ESC_ARM_PULSE;
            }
            motorsArmed = false;
        }
        else
        {
            // Snapshot control inputs quickly to avoid long controlMutex holds
            ControlPacket localControl;
            if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(2)) == pdTRUE)
            {
                localControl = receivedControl;
                xSemaphoreGive(controlMutex);
            }
            // Process control inputs using snapshot
            calculateMotorSpeeds(localControl);
        }

        // Write motor speeds to ESCs
        esc1.writeMicroseconds(motorSpeeds[0]); // Front Right (CCW)
        esc2.writeMicroseconds(motorSpeeds[1]); // Back Right (CW)
        esc3.writeMicroseconds(motorSpeeds[2]); // Front Left (CW)
        esc4.writeMicroseconds(motorSpeeds[3]); // Back Left (CCW)

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, motorFrequency);
    }
}

// Calculate Motor Speeds from Control Inputs with PID Integration
void calculateMotorSpeeds()
{
    // Backward-compatible wrapper using current global control (kept for any legacy calls)
    ControlPacket cp;
    if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
        cp = receivedControl;
        xSemaphoreGive(controlMutex);
    }
    calculateMotorSpeeds(cp);
}

// Latency-optimized variant using a snapshot of control inputs
void calculateMotorSpeeds(const ControlPacket &receivedControl)
{
    // Minimal refactor: explicit edge tracking & PID mode change hygiene
    static bool prevArmCmd = false;
    static bool lastModeToggle = false; // cached toggle2 state while armed
    bool armCmd = (receivedControl.toggle1 == 1);
    bool modeToggle = (receivedControl.toggle2 == 1); // ON = Stabilized

    // Rising edge: arming
    if (armCmd && !prevArmCmd)
    {
        motorsArmed = true;
        // Select initial mode based on toggle2
        if (modeToggle)
        {
            currentFlightMode = FLIGHT_MODE_STABILIZE;
            pidEnabled = true;
            stabilizedMode = true;
            // Reset PID integrators to avoid stale windup on entry
            rollPID.reset();
            pitchPID.reset();
            yawPID.reset();
            altitudePID.reset();
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                Serial.println("🔓 MOTORS ARMED - STABILIZED MODE (PID ON)");
                xSemaphoreGive(serialMutex);
            }
        }
        else
        {
            currentFlightMode = FLIGHT_MODE_MANUAL;
            pidEnabled = false;
            stabilizedMode = false;
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                Serial.println("🔓 MOTORS ARMED - MANUAL MODE (PID OFF)");
                xSemaphoreGive(serialMutex);
            }
        }
        lastModeToggle = modeToggle; // initialize cache to suppress duplicate print
    }
    // While armed: detect mode toggle edge
    else if (armCmd && prevArmCmd)
    {
        if (modeToggle != lastModeToggle)
        {
            if (modeToggle)
            {
                currentFlightMode = FLIGHT_MODE_STABILIZE;
                pidEnabled = true;
                stabilizedMode = true;
                rollPID.reset();
                pitchPID.reset();
                yawPID.reset(); // altitude PID only when that mode engaged
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    Serial.println("🛡️ FLIGHT MODE: STABILIZED (PID ON)");
                    xSemaphoreGive(serialMutex);
                }
            }
            else
            {
                currentFlightMode = FLIGHT_MODE_MANUAL;
                pidEnabled = false;
                stabilizedMode = false;
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    Serial.println("🎯 FLIGHT MODE: MANUAL (PID OFF)");
                    xSemaphoreGive(serialMutex);
                }
            }
            lastModeToggle = modeToggle;
        }
    }
    // Falling edge: disarming
    else if (!armCmd && prevArmCmd)
    {
        // Re-calibrate altitude baseline if enough pressure samples collected
        if (pressureCount >= PRESSURE_BUFFER_SIZE / 2)
        {
            seaLevelPressure = exponentialPressure;
            altitudeOffset = 44330.0 * (1.0 - pow(exponentialPressure / seaLevelPressure, 0.1903));
            altitudeCalibrated = true;
            lastAltitudeCalibration = millis();
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.printf("Altitude re-calibrated on disarm - Ref P: %.2f hPa, Offset: %.2f m\n", seaLevelPressure, altitudeOffset);
                xSemaphoreGive(serialMutex);
            }
        }
        imuData.yaw = 0.0f; // reset yaw accumulation
        lastYawReset = millis();
        motorsArmed = false;
        stabilizedMode = false;
        currentFlightMode = FLIGHT_MODE_DISARMED;
        pidEnabled = false;
        // Force motors to min below after armed check block
    }

    prevArmCmd = armCmd;

    if (!motorsArmed)
    {
        for (int i = 0; i < 4; i++)
            motorSpeeds[i] = ESC_ARM_PULSE;
        return;
    }

    // Convert throttle input with full 2000μs range support and intelligent headroom management
    // Virtual Throttle Mode: Remote sends -3000 to +3000 where:
    // -3000 = 0% power (minimum/idle), +3000 = 100% power (maximum = 2000μs when no control applied)
    // When control inputs are active, base throttle is intelligently reduced to maintain differential control

    int throttleInput;
    int baseThrottle;

    if (receivedControl.throttle <= -3000)
    {
        // At or below minimum - motors idle
        throttleInput = 0;
        baseThrottle = ESC_ARM_PULSE;
    }
    else
    {
        if (currentFlightMode == FLIGHT_MODE_MANUAL)
        {
            // New scaled mixer handles saturation itself; give full 1000-2000 range directly
            throttleInput = map(receivedControl.throttle, -3000, 3000, 0, (ESC_MAX_PULSE - ESC_ARM_PULSE));
            baseThrottle = ESC_ARM_PULSE + constrain(throttleInput, 0, (ESC_MAX_PULSE - ESC_ARM_PULSE));
        }
        else
        {
            // Option B: No pre-reserved headroom. Use full 1000-2000 range like manual; mixer scales corrections.
            throttleInput = map(receivedControl.throttle, -3000, 3000, 0, (ESC_MAX_PULSE - ESC_ARM_PULSE));
            baseThrottle = ESC_ARM_PULSE + constrain(throttleInput, 0, (ESC_MAX_PULSE - ESC_ARM_PULSE));
        }
    }

    // Get PID outputs (thread-safe)
    float rollCorrection = 0.0, pitchCorrection = 0.0, yawCorrection = 0.0, altitudeCorrection = 0.0;
    if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        rollCorrection = pidRollOutput;
        pitchCorrection = pidPitchOutput;
        yawCorrection = pidYawOutput;
        altitudeCorrection = pidAltitudeOutput;
        xSemaphoreGive(pidMutex);
    }

    // Motor mixing based on flight mode
    bool usedScaledMixing = false; // Track if new scaled mixer used (skip extra limiting then)
    if (currentFlightMode == FLIGHT_MODE_MANUAL)
    {
        // Manual mode - use scaled mixer with dynamic saturation management
        // Normalize stick inputs (-3000..3000) -> (-1.0 .. 1.0)
        float Rn = constrain((receivedControl.roll / 3000.0f) * MANUAL_AXIS_GAIN, -1.0f, 1.0f);
        float Pn = constrain((receivedControl.pitch / 3000.0f) * MANUAL_AXIS_GAIN, -1.0f, 1.0f);
        float Yn = constrain((receivedControl.yaw / 3000.0f) * MANUAL_AXIS_GAIN, -1.0f, 1.0f);
        applyScaledMotorMix((float)baseThrottle, Rn, Pn, Yn);
        usedScaledMixing = true;
    }
    else if (currentFlightMode >= FLIGHT_MODE_STABILIZE)
    {
        // Stabilized / Altitude hold / future modes - reuse scaled mixer for unified saturation handling
        float adjustedBaseThrottle = baseThrottle;
        if (currentFlightMode >= FLIGHT_MODE_ALTITUDE_HOLD)
        {
            // Option B: altitudeCorrection applied after full-range base mapping (not pre-reserved).
            adjustedBaseThrottle += altitudeCorrection;
        }
        // Clamp base after altitude adjustment (mixer will handle differential scaling)
        if (adjustedBaseThrottle < ESC_ARM_PULSE)
            adjustedBaseThrottle = ESC_ARM_PULSE;
        if (adjustedBaseThrottle > ESC_MAX_PULSE)
            adjustedBaseThrottle = ESC_MAX_PULSE;
        // Normalize PID microsecond outputs to [-1,1] relative to mixer gains.
        // This preserves proportional authority and lets applyScaledMotorMix scale if near saturation.
        float Rn = constrain(rollCorrection / MIX_KR, -1.0f, 1.0f);
        float Pn = constrain(pitchCorrection / MIX_KP, -1.0f, 1.0f);
        float Yn = constrain(yawCorrection / MIX_KY, -1.0f, 1.0f);

        applyScaledMotorMix(adjustedBaseThrottle, Rn, Pn, Yn);
        usedScaledMixing = true;
    }

    // Enhanced motor speed limiting only if legacy/direct mixing path used
    if (!usedScaledMixing)
    {
        // First constrain to basic range
        for (int i = 0; i < 4; i++)
        {
            motorSpeeds[i] = constrain(motorSpeeds[i], ESC_ARM_PULSE, ESC_MAX_PULSE);
        }

        // Advanced safety: If any motor exceeds limits, use differential motor compensation
        int maxMotorSpeed = max(max(motorSpeeds[0], motorSpeeds[1]), max(motorSpeeds[2], motorSpeeds[3]));
        int minMotorSpeed = min(min(motorSpeeds[0], motorSpeeds[1]), min(motorSpeeds[2], motorSpeeds[3]));

        if (maxMotorSpeed > ESC_MAX_PULSE)
        {
            int excess = maxMotorSpeed - ESC_MAX_PULSE;
            for (int i = 0; i < 4; i++)
            {
                motorSpeeds[i] -= excess;
                motorSpeeds[i] = constrain(motorSpeeds[i], ESC_ARM_PULSE, ESC_MAX_PULSE);
            }
        }

        // Additional safety check - if any motor is still below minimum, bring all up proportionally
        minMotorSpeed = min(min(motorSpeeds[0], motorSpeeds[1]), min(motorSpeeds[2], motorSpeeds[3]));
        if (minMotorSpeed < ESC_ARM_PULSE)
        {
            int deficit = ESC_ARM_PULSE - minMotorSpeed;
            for (int i = 0; i < 4; i++)
            {
                motorSpeeds[i] += deficit;
                motorSpeeds[i] = constrain(motorSpeeds[i], ESC_ARM_PULSE, ESC_MAX_PULSE);
            }
        }
    }

    // Apply per-motor calibration offsets AFTER mixing in all modes
    motorSpeeds[0] = constrain(motorSpeeds[0] + MOTOR1_OFFSET_US, ESC_ARM_PULSE, ESC_MAX_PULSE);
    motorSpeeds[1] = constrain(motorSpeeds[1] + MOTOR2_OFFSET_US, ESC_ARM_PULSE, ESC_MAX_PULSE);
    motorSpeeds[2] = constrain(motorSpeeds[2] + MOTOR3_OFFSET_US, ESC_ARM_PULSE, ESC_MAX_PULSE);
    motorSpeeds[3] = constrain(motorSpeeds[3] + MOTOR4_OFFSET_US, ESC_ARM_PULSE, ESC_MAX_PULSE);

    // Enhanced debug output with intelligent throttle management info (less frequent to avoid overwhelming serial)
    static unsigned long lastMotorDebug = 0;
    if (millis() - lastMotorDebug > 2000) // Every 2 seconds
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // Calculate current throttle percentage and control activity
            int maxCurrentMotor = max(max(motorSpeeds[0], motorSpeeds[1]), max(motorSpeeds[2], motorSpeeds[3]));
            int minCurrentMotor = min(min(motorSpeeds[0], motorSpeeds[1]), min(motorSpeeds[2], motorSpeeds[3]));
            int throttlePercent = map(baseThrottle, ESC_ARM_PULSE, ESC_MAX_PULSE, 0, 100);
            int controlActivity = maxCurrentMotor - minCurrentMotor; // Shows how much control authority is being used

            Serial.print("🚁 Motors: Armed:");
            Serial.print(motorsArmed ? "YES" : "NO");
            Serial.print(" | Throttle: Raw:");
            Serial.print(receivedControl.throttle);
            Serial.print(" Base:");
            Serial.print(baseThrottle);
            Serial.print("μs (");
            Serial.print(throttlePercent);
            Serial.print("%) | Motor Range: ");
            Serial.print(minCurrentMotor);
            Serial.print("-");
            Serial.print(maxCurrentMotor);
            Serial.print("μs | Control Activity:");
            Serial.print(controlActivity);
            Serial.print("μs | Speeds:[");
            Serial.print(motorSpeeds[0]);
            Serial.print(",");
            Serial.print(motorSpeeds[1]);
            Serial.print(",");
            Serial.print(motorSpeeds[2]);
            Serial.print(",");
            Serial.print(motorSpeeds[3]);
            Serial.print("] | Mode:");
            switch (currentFlightMode)
            {
            case FLIGHT_MODE_DISARMED:
                Serial.print("DISARMED");
                break;
            case FLIGHT_MODE_MANUAL:
                Serial.print("MANUAL");
                break;
            case FLIGHT_MODE_STABILIZE:
                Serial.print("STABILIZE");
                break;
            case FLIGHT_MODE_ALTITUDE_HOLD:
                Serial.print("ALT_HOLD");
                break;
            default:
                Serial.print("UNKNOWN");
                break;
            }

            Serial.print(" [Armed:");
            Serial.print(motorsArmed ? "YES" : "NO");
            Serial.print("] T:");
            Serial.print(throttleInput);

            if (currentFlightMode >= FLIGHT_MODE_STABILIZE && imuData.dataValid)
            {
                Serial.print(" | IMU R:");
                Serial.print(imuData.roll, 1);
                Serial.print(" P:");
                Serial.print(imuData.pitch, 1);
                Serial.print(" Y:");
                Serial.print(imuData.yaw, 1);
            }

            Serial.print(" | Motors:");
            Serial.print(motorSpeeds[0]);
            Serial.print(",");
            Serial.print(motorSpeeds[1]);
            Serial.print(",");
            Serial.print(motorSpeeds[2]);
            Serial.print(",");
            Serial.println(motorSpeeds[3]);
            xSemaphoreGive(serialMutex);
        }
        lastMotorDebug = millis();
    }
}

// ================================
// PID AND IMU CONTROL FUNCTIONS
// ================================

// Initialize IMU and PID Controllers
void initializeIMUAndPID()
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.println("Initializing MPU6050 IMU and PID controllers...");
        xSemaphoreGive(serialMutex);
    }

    // Initialize MPU6050
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
        if (mpu6050.begin(MPU6050_ADDRESS))
        {
            // Configure MPU6050 for optimal performance
            mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G); // ±8g range
            mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);      // ±500°/s range
            mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);   // 21Hz low-pass filter

            imuInitialized = true;

            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✓ MPU6050 initialized successfully");
                xSemaphoreGive(serialMutex);
            }
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ MPU6050 initialization failed - PID disabled");
                xSemaphoreGive(serialMutex);
            }
            imuInitialized = false;
        }
        xSemaphoreGive(i2cMutex);
    }

    // Initialize PID controllers with advanced features
    rollPID.initialize(pidConfig.roll_kp, pidConfig.roll_ki, pidConfig.roll_kd,
                       -PID_ROLL_PITCH_MAX, PID_ROLL_PITCH_MAX);
    rollPID.setDerivativeFilter(pidConfig.roll_filter);
    rollPID.setSetpointSmoothing(pidConfig.roll_smooth);
    rollPID.setIntegralLimits(0.2, 0.2); // 20% of output range for integral

    pitchPID.initialize(pidConfig.pitch_kp, pidConfig.pitch_ki, pidConfig.pitch_kd,
                        -PID_ROLL_PITCH_MAX, PID_ROLL_PITCH_MAX);
    pitchPID.setDerivativeFilter(pidConfig.pitch_filter);
    pitchPID.setSetpointSmoothing(pidConfig.pitch_smooth);
    pitchPID.setIntegralLimits(0.2, 0.2); // 20% of output range for integral

    yawPID.initialize(pidConfig.yaw_kp, pidConfig.yaw_ki, pidConfig.yaw_kd,
                      -PID_YAW_MAX, PID_YAW_MAX);
    yawPID.setDerivativeFilter(pidConfig.yaw_filter);
    yawPID.setSetpointSmoothing(pidConfig.yaw_smooth);
    yawPID.setIntegralLimits(0.15, 0.15); // 15% of output range for yaw integral

    altitudePID.initialize(pidConfig.alt_kp, pidConfig.alt_ki, pidConfig.alt_kd,
                           -PID_ALTITUDE_MAX, PID_ALTITUDE_MAX);
    altitudePID.setDerivativeFilter(pidConfig.alt_filter);
    altitudePID.setSetpointSmoothing(pidConfig.alt_smooth);
    altitudePID.setIntegralLimits(0.3, 0.3); // 30% of output range for altitude integral    // Initialize IMU data structure
    imuData.dataValid = false;
    imuData.timestamp = 0;

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("✓ PID controllers initialized");
        if (USE_CASCADED_ANGLE_RATE)
        {
            Serial.println("✓ Cascaded angle->rate control ENABLED (roll/pitch)");
        }
        if (USE_KALMAN_ATTITUDE)
        {
            Serial.println("✓ Kalman attitude fusion ENABLED (roll/pitch)");
        }
        if (imuInitialized)
        {
            Serial.println("✓ Starting IMU calibration...");
        }
        xSemaphoreGive(serialMutex);
    }

    // Initialize inner rate PIDs if cascaded mode is enabled
    if (USE_CASCADED_ANGLE_RATE)
    {
        rollRatePID.initialize(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD, -PID_ROLL_PITCH_MAX, PID_ROLL_PITCH_MAX);
        rollRatePID.setDerivativeFilter(pidConfig.roll_filter);
        rollRatePID.setSetpointSmoothing(0.05);
        rollRatePID.setIntegralLimits(0.2, 0.2);

        pitchRatePID.initialize(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD, -PID_ROLL_PITCH_MAX, PID_ROLL_PITCH_MAX);
        pitchRatePID.setDerivativeFilter(pidConfig.pitch_filter);
        pitchRatePID.setSetpointSmoothing(0.05);
        pitchRatePID.setIntegralLimits(0.2, 0.2);
    }
}

// IMU Task - High frequency IMU reading (100Hz)
void imuTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t imuFrequency = pdMS_TO_TICKS(5); // 200Hz IMU reading to match WiFi code timing
    static unsigned long lastIMUTime = 0;             // For dynamic dt calculation

    for (;;)
    {
        if (imuInitialized && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            sensors_event_t accel, gyro, temp;
            bool readSuccess = mpu6050.getEvent(&accel, &gyro, &temp);
            xSemaphoreGive(i2cMutex);

            if (readSuccess && xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // Raw sensor data
                // Apply 90° CW Z rotation mapping to body frame
                imuData.accelX = accel.acceleration.y;     // bodyX
                imuData.accelY = -accel.acceleration.x;    // bodyY
                imuData.accelZ = accel.acceleration.z;     // bodyZ
                imuData.gyroX = gyro.gyro.y * 180.0 / PI;  // roll rate
                imuData.gyroY = -gyro.gyro.x * 180.0 / PI; // pitch rate
                imuData.gyroZ = gyro.gyro.z * 180.0 / PI;  // yaw rate
                imuData.temperature = temp.temperature;

                // Calculate roll and pitch from accelerometer
                // Use full 3D angle calculation to match WiFi code for better accuracy during non-level attitudes
                float roll_acc = atan2(imuData.accelY, sqrt(imuData.accelX * imuData.accelX + imuData.accelZ * imuData.accelZ)) * 180.0 / PI;
                float pitch_acc = atan2(-imuData.accelX, sqrt(imuData.accelY * imuData.accelY + imuData.accelZ * imuData.accelZ)) * 180.0 / PI;
                // Transform and subtract gyro calibration biases (original offsets in sensor frame)
                // (Accelerometer bias terms not stored in this build; only roll/pitch angle offsets handled.)
                // Gyro mapping: body roll rate = sensorY, body pitch rate = -sensorX
                // Calibration stored: gyroXOffset = avg(gyro.gyro.y) => roll bias (body roll rate)
                //                       gyroYOffset = avg(-gyro.gyro.x) => pitch bias (body pitch rate)
                float gyroRollOffset = imuCalibration.gyroXOffset;  // roll bias in transformed frame
                float gyroPitchOffset = imuCalibration.gyroYOffset; // pitch bias in transformed frame
                float gyroXCal = imuData.gyroX - gyroRollOffset;
                float gyroYCal = imuData.gyroY - gyroPitchOffset;
                float gyroZCal = imuData.gyroZ - imuCalibration.gyroZOffset;
                // Apply static level offsets so that calibrated orientation reads 0/0
                float rollAccCal = roll_acc - imuCalibration.rollOffset;
                float pitchAccCal = pitch_acc - imuCalibration.pitchOffset;
                // Overwrite raw for subsequent fusion so downstream uses corrected values
                roll_acc = rollAccCal;
                pitch_acc = pitchAccCal;

                // Calculate actual dt for better accuracy to match WiFi code approach
                unsigned long currentTime = millis();
                float dt = 0.005; // Default 5ms for 200Hz, but use actual timing when available
                if (lastIMUTime > 0)
                {
                    dt = (currentTime - lastIMUTime) / 1000.0f; // Convert to seconds
                    dt = constrain(dt, 0.001f, 0.02f);          // Constrain to reasonable range (1-20ms)
                }
                lastIMUTime = currentTime;

                // Enhanced yaw drift correction

                // Enhanced yaw drift correction
                // Check if drone is stationary (low angular rates on all axes)
                float totalAngularRate = abs(gyroXCal) + abs(gyroYCal) + abs(gyroZCal);
                if (totalAngularRate < 1.0 && !motorsArmed) // Less than 1 deg/s on all axes and disarmed
                {
                    if (!isStationary)
                    {
                        isStationary = true;
                        stationaryStartTime = millis();
                    }
                    else if (millis() - stationaryStartTime > STATIONARY_THRESHOLD_TIME)
                    {
                        // Accumulate bias samples when drone is definitely stationary
                        if (gyroBiasSampleCount < GYRO_BIAS_SAMPLES)
                        {
                            gyroZBiasSum += gyroZCal;
                            gyroBiasSampleCount++;
                        }
                        else
                        {
                            // Update adaptive bias
                            adaptiveGyroZBias = gyroZBiasSum / GYRO_BIAS_SAMPLES;
                            gyroZBiasSum = 0.0;
                            gyroBiasSampleCount = 0;
                        }
                    }
                }
                else
                {
                    isStationary = false;
                    stationaryStartTime = millis();
                }

                // Apply adaptive bias correction to yaw gyro
                float gyroZCorrected = gyroZCal - adaptiveGyroZBias;

                // Use Kalman filter (1D) for roll/pitch if enabled
                if (USE_KALMAN_ATTITUDE)
                {
                    // 1D Kalman filter update for roll
                    kalmanRoll.angle = kalmanRoll.angle + dt * gyroXCal;
                    kalmanRoll.uncertainty = kalmanRoll.uncertainty + dt * dt * (KALMAN_RATE_STD * KALMAN_RATE_STD);
                    float gainR = kalmanRoll.uncertainty / (kalmanRoll.uncertainty + (KALMAN_MEAS_STD * KALMAN_MEAS_STD));
                    kalmanRoll.angle = kalmanRoll.angle + gainR * (rollAccCal - kalmanRoll.angle);
                    kalmanRoll.uncertainty = (1.0f - gainR) * kalmanRoll.uncertainty;

                    // 1D Kalman filter update for pitch
                    kalmanPitch.angle = kalmanPitch.angle + dt * gyroYCal;
                    kalmanPitch.uncertainty = kalmanPitch.uncertainty + dt * dt * (KALMAN_RATE_STD * KALMAN_RATE_STD);
                    float gainP = kalmanPitch.uncertainty / (kalmanPitch.uncertainty + (KALMAN_MEAS_STD * KALMAN_MEAS_STD));
                    kalmanPitch.angle = kalmanPitch.angle + gainP * (pitchAccCal - kalmanPitch.angle);
                    kalmanPitch.uncertainty = (1.0f - gainP) * kalmanPitch.uncertainty;

                    // Clamp to reasonable limits to avoid runaway
                    kalmanRoll.angle = constrain(kalmanRoll.angle, -60.0f, 60.0f);
                    kalmanPitch.angle = constrain(kalmanPitch.angle, -60.0f, 60.0f);

                    imuData.roll = kalmanRoll.angle;
                    imuData.pitch = kalmanPitch.angle;
                }
                else
                {
                    // Complementary filter for stable angle estimation (default)
                    // 98% gyro integration + 2% accelerometer correction
                    float alpha = 0.98;
                    imuData.roll = alpha * (imuData.roll + gyroXCal * dt) + (1.0 - alpha) * rollAccCal;
                    imuData.pitch = alpha * (imuData.pitch + gyroYCal * dt) + (1.0 - alpha) * pitchAccCal;
                }

                // Yaw with leaky integrator and bias correction
                imuData.yaw = (imuData.yaw + gyroZCorrected * dt) * YAW_LEAKY_ALPHA;

                // Store calibrated angular rates
                imuData.rollRate = gyroXCal;
                imuData.pitchRate = gyroYCal;
                imuData.yawRate = gyroZCorrected; // Use bias-corrected yaw rate

                imuData.dataValid = true;
                imuData.timestamp = millis();

                xSemaphoreGive(imuMutex);
            }
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, imuFrequency);
    }
}

// PID Control Task - Runs at 200Hz for smooth control to match WiFi code
void pidControlTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t pidFrequency = pdMS_TO_TICKS(PID_LOOP_PERIOD); // 200Hz PID loop

    for (;;)
    {
        if (imuCalibration.calibrated && imuData.dataValid && pidEnabled)
        {
            // Take mutexes in consistent order to avoid deadlock
            if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                    {
                        // Determine flight mode and setpoints
                        updateFlightMode();

                        if (currentFlightMode >= FLIGHT_MODE_STABILIZE)
                        {
                            // Convert control inputs to angle setpoints with limits and smoothing
                            // Apply maximum angle limits for safety
                            // Roll stick mapping: positive stick -> positive roll angle
                            float rollSetpointRaw = map(receivedControl.roll, -3000, 3000,
                                                        -pidConfig.max_angle, pidConfig.max_angle);
                            float pitchSetpointRaw = map(receivedControl.pitch, -3000, 3000,
                                                         -pidConfig.max_angle, pidConfig.max_angle);

                            // Slew limiting for roll/pitch setpoints (deg/sec)
                            float maxDeltaAngle = PID_SETPOINT_SLEW_ROLL_PITCH * (PID_LOOP_PERIOD / 1000.0f);
                            float rDelta = rollSetpointRaw - rollPID.setpoint;
                            if (fabs(rDelta) > maxDeltaAngle)
                                rollSetpointRaw = rollPID.setpoint + copysign(maxDeltaAngle, rDelta);
                            float pDelta = pitchSetpointRaw - pitchPID.setpoint;
                            if (fabs(pDelta) > maxDeltaAngle)
                                pitchSetpointRaw = pitchPID.setpoint + copysign(maxDeltaAngle, pDelta);

                            // Constrain to safe limits
                            rollPID.setpoint = constrain(rollSetpointRaw, -pidConfig.max_angle, pidConfig.max_angle);
                            pitchPID.setpoint = constrain(pitchSetpointRaw, -pidConfig.max_angle, pidConfig.max_angle);

                            // Yaw is rate control (deg/s), not angle - apply rate limits
                            float yawRateSetpointRaw = map(receivedControl.yaw, -3000, 3000,
                                                           -pidConfig.max_yaw_rate, pidConfig.max_yaw_rate);
                            // Slew limit yaw rate changes
                            float maxDeltaYaw = PID_SETPOINT_SLEW_YAW_RATE * (PID_LOOP_PERIOD / 1000.0f);
                            float yDelta = yawRateSetpointRaw - yawPID.setpoint;
                            if (fabs(yDelta) > maxDeltaYaw)
                                yawRateSetpointRaw = yawPID.setpoint + copysign(maxDeltaYaw, yDelta);
                            yawPID.setpoint = constrain(yawRateSetpointRaw, -pidConfig.max_yaw_rate, pidConfig.max_yaw_rate);

                            // Choose control topology
                            if (!USE_CASCADED_ANGLE_RATE)
                            {
                                // Direct angle stabilization (current behavior)
                                rollPID.input = imuData.roll;
                                pitchPID.input = imuData.pitch;
                                yawPID.input = imuData.yawRate; // Rate control for yaw

                                rollPID.enabled = true;
                                pitchPID.enabled = true;
                                yawPID.enabled = true;
                            }
                            else
                            {
                                // Cascaded control: Angle outer loop generates desired rate setpoints
                                // Outer loop uses rollPID/pitchPID as angle controllers, output interpreted as desired rate
                                rollPID.input = imuData.roll;
                                pitchPID.input = imuData.pitch;
                                rollPID.enabled = true;
                                pitchPID.enabled = true;
                                double desiredRollRate = constrain(rollPID.compute(), -MAX_ROLL_PITCH_RATE, MAX_ROLL_PITCH_RATE);
                                double desiredPitchRate = constrain(pitchPID.compute(), -MAX_ROLL_PITCH_RATE, MAX_ROLL_PITCH_RATE);

                                // Inner rate loop tracks gyro rates
                                rollRatePID.setpoint = desiredRollRate;
                                pitchRatePID.setpoint = desiredPitchRate;
                                rollRatePID.input = imuData.rollRate;
                                pitchRatePID.input = imuData.pitchRate;
                                rollRatePID.enabled = true;
                                pitchRatePID.enabled = true;

                                // Yaw remains rate control
                                yawPID.input = imuData.yawRate;
                                yawPID.enabled = true;
                            }

                            // Integrator gating: disable integral accumulation when throttle very low (< ~10%)
                            // Map receivedControl.throttle (-3000..3000) to [0..1]
                            float normThrottle = (receivedControl.throttle + 3000.0f) / 6000.0f; // 0..1
                            bool allowI = normThrottle > PID_THROTTLE_GATE;                      // configurable threshold (35%)
                            rollPID.setIntegralEnabled(allowI);
                            pitchPID.setIntegralEnabled(allowI);
                            yawPID.setIntegralEnabled(allowI); // yaw may also benefit from gating on ground

                            // Compute PID outputs
                            if (!USE_CASCADED_ANGLE_RATE)
                            {
                                pidRollOutput = rollPID.compute();
                                pidPitchOutput = pitchPID.compute();
                            }
                            else
                            {
                                // Use inner rate loop outputs for roll/pitch
                                pidRollOutput = rollRatePID.compute();
                                pidPitchOutput = pitchRatePID.compute();
                            }
                            pidYawOutput = yawPID.compute();
                            // Yaw feed-forward term (setpoint in deg/s)
                            if (YAW_FEED_FORWARD_GAIN > 0.0f)
                            {
                                float ff = YAW_FEED_FORWARD_GAIN * yawPID.setpoint;
                                pidYawOutput += ff;
                                if (pidYawOutput > PID_YAW_MAX)
                                    pidYawOutput = PID_YAW_MAX;
                                if (pidYawOutput < -PID_YAW_MAX)
                                    pidYawOutput = -PID_YAW_MAX;
                            }

                            // Altitude hold mode with climb rate limiting
                            if (currentFlightMode >= FLIGHT_MODE_ALTITUDE_HOLD)
                            {
                                // Use pressure altitude for altitude hold
                                altitudePID.input = getFilteredAltitude();
                                altitudePID.enabled = true;

                                // Limit altitude changes to reasonable climb rate
                                float altitudeError = altitudePID.setpoint - altitudePID.input;
                                float maxAltitudeChange = pidConfig.max_climb_rate * (PID_LOOP_PERIOD / 1000.0);

                                if (abs(altitudeError) > maxAltitudeChange)
                                {
                                    // Gradually approach target altitude
                                    if (altitudeError > 0)
                                        altitudePID.setpoint = altitudePID.input + maxAltitudeChange;
                                    else
                                        altitudePID.setpoint = altitudePID.input - maxAltitudeChange;
                                }

                                pidAltitudeOutput = altitudePID.compute();
                            }
                            else
                            {
                                altitudePID.enabled = false;
                                pidAltitudeOutput = 0.0;
                            }
                        }
                        else
                        {
                            // Manual mode - disable all PID controllers
                            rollPID.enabled = false;
                            pitchPID.enabled = false;
                            yawPID.enabled = false;
                            altitudePID.enabled = false;

                            pidRollOutput = 0.0;
                            pidPitchOutput = 0.0;
                            pidYawOutput = 0.0;
                            pidAltitudeOutput = 0.0;
                        }

                        // Release mutexes in reverse order
                        xSemaphoreGive(pidMutex);
                    }
                    xSemaphoreGive(imuMutex);
                }
                xSemaphoreGive(controlMutex);
            }

            // Enhanced debugging output every 100 cycles (2 seconds at 50Hz)
            static int debugCounter = 0;
            if (++debugCounter >= 100)
            {
                debugCounter = 0;

                if (enableVerboseLogging && currentFlightMode >= FLIGHT_MODE_STABILIZE)
                {
                    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        // PID diagnostic information
                        Serial.print("PID Diagnostics - Mode: ");
                        Serial.print(currentFlightMode);
                        Serial.println();

                        // Roll axis diagnostics
                        Serial.print("  Roll: SP=");
                        Serial.print(rollPID.setpoint, 1);
                        Serial.print(" IN=");
                        Serial.print(rollPID.input, 1);
                        Serial.print(" OUT=");
                        Serial.print(pidRollOutput, 1);
                        Serial.print(" [P=");
                        Serial.print(rollPID.getProportionalTerm(), 1);
                        Serial.print(" I=");
                        Serial.print(rollPID.getIntegralTerm(), 1);
                        Serial.print(" D=");
                        Serial.print(rollPID.getDerivativeTerm(), 1);
                        Serial.print("]");
                        if (rollPID.isOutputSaturated())
                            Serial.print(" SAT");
                        if (rollPID.isIntegralSaturated())
                            Serial.print(" I-SAT");
                        Serial.println();

                        // Pitch axis diagnostics
                        Serial.print("  Pitch: SP=");
                        Serial.print(pitchPID.setpoint, 1);
                        Serial.print(" IN=");
                        Serial.print(pitchPID.input, 1);
                        Serial.print(" OUT=");
                        Serial.print(pidPitchOutput, 1);
                        Serial.print(" [P=");
                        Serial.print(pitchPID.getProportionalTerm(), 1);
                        Serial.print(" I=");
                        Serial.print(pitchPID.getIntegralTerm(), 1);
                        Serial.print(" D=");
                        Serial.print(pitchPID.getDerivativeTerm(), 1);
                        Serial.print("]");
                        if (pitchPID.isOutputSaturated())
                            Serial.print(" SAT");
                        if (pitchPID.isIntegralSaturated())
                            Serial.print(" I-SAT");
                        Serial.println();

                        // Yaw axis diagnostics (rate control)
                        Serial.print("  Yaw: SP=");
                        Serial.print(yawPID.setpoint, 1);
                        Serial.print(" IN=");
                        Serial.print(yawPID.input, 1);
                        Serial.print(" OUT=");
                        Serial.print(pidYawOutput, 1);
                        Serial.print(" [P=");
                        Serial.print(yawPID.getProportionalTerm(), 1);
                        Serial.print(" I=");
                        Serial.print(yawPID.getIntegralTerm(), 1);
                        Serial.print(" D=");
                        Serial.print(yawPID.getDerivativeTerm(), 1);
                        Serial.print("]");
                        if (yawPID.isOutputSaturated())
                            Serial.print(" SAT");
                        if (yawPID.isIntegralSaturated())
                            Serial.print(" I-SAT");
                        Serial.println();

                        // Altitude diagnostics if in altitude hold
                        if (currentFlightMode >= FLIGHT_MODE_ALTITUDE_HOLD)
                        {
                            Serial.print("  Alt: SP=");
                            Serial.print(altitudePID.setpoint, 2);
                            Serial.print(" IN=");
                            Serial.print(altitudePID.input, 2);
                            Serial.print(" OUT=");
                            Serial.print(pidAltitudeOutput, 1);
                            Serial.print(" [P=");
                            Serial.print(altitudePID.getProportionalTerm(), 1);
                            Serial.print(" I=");
                            Serial.print(altitudePID.getIntegralTerm(), 1);
                            Serial.print(" D=");
                            Serial.print(altitudePID.getDerivativeTerm(), 1);
                            Serial.print("]");
                            if (altitudePID.isOutputSaturated())
                                Serial.print(" SAT");
                            if (altitudePID.isIntegralSaturated())
                                Serial.print(" I-SAT");
                            Serial.println();
                        }

                        Serial.println("---");
                        xSemaphoreGive(serialMutex);
                    }
                }
            }
        }
        else
        {
            // PID disabled or IMU not ready
            if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                pidRollOutput = 0.0;
                pidPitchOutput = 0.0;
                pidYawOutput = 0.0;
                pidAltitudeOutput = 0.0;
                xSemaphoreGive(pidMutex);
            }
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, pidFrequency);
    }
}

// Update flight mode based on control inputs and system state
void updateFlightMode()
{
    static FlightMode previousMode = FLIGHT_MODE_DISARMED;

    if (!motorsArmed)
    {
        currentFlightMode = FLIGHT_MODE_DISARMED;
        pidEnabled = false;
    }
    else
    {
        // If motors are armed but still in disarmed mode, switch to manual
        if (currentFlightMode == FLIGHT_MODE_DISARMED)
        {
            currentFlightMode = FLIGHT_MODE_MANUAL;
            pidEnabled = false;
        }

        // Check joystick button states for mode switching
        // Joy1 button: Toggle between Manual and Stabilize
        // Joy2 button: Enable altitude hold (if in stabilize mode)

        static bool joy1ButtonPressed = false;
        static bool joy2ButtonPressed = false;

        // Joy1 button (toggle stabilization)
        if (receivedControl.joy1_btn == 0 && !joy1ButtonPressed) // Button pressed (active low)
        {
            joy1ButtonPressed = true;
            if (currentFlightMode == FLIGHT_MODE_MANUAL)
            {
                currentFlightMode = FLIGHT_MODE_STABILIZE;
                pidEnabled = true;

                // Reset PID integrators when enabling
                rollPID.reset();
                pitchPID.reset();
                yawPID.reset();
            }
            else if (currentFlightMode >= FLIGHT_MODE_STABILIZE)
            {
                currentFlightMode = FLIGHT_MODE_MANUAL;
                pidEnabled = false;
            }
        }
        else if (receivedControl.joy1_btn == 1) // Button released
        {
            joy1ButtonPressed = false;
        }

        // Joy2 button (altitude hold toggle)
        if (receivedControl.joy2_btn == 0 && !joy2ButtonPressed) // Button pressed
        {
            joy2ButtonPressed = true;
            if (currentFlightMode == FLIGHT_MODE_STABILIZE)
            {
                currentFlightMode = FLIGHT_MODE_ALTITUDE_HOLD;
                targetAltitude = getFilteredAltitude(); // Set current altitude as target
                altitudePID.setpoint = targetAltitude;
                altitudePID.reset();
            }
            else if (currentFlightMode == FLIGHT_MODE_ALTITUDE_HOLD)
            {
                currentFlightMode = FLIGHT_MODE_STABILIZE;
            }
        }
        else if (receivedControl.joy2_btn == 1) // Button released
        {
            joy2ButtonPressed = false;
        }

        // Default to manual mode on startup
        if (currentFlightMode == FLIGHT_MODE_DISARMED && motorsArmed)
        {
            currentFlightMode = FLIGHT_MODE_MANUAL;
            pidEnabled = false;
        }
    }

    // Print mode changes
    if (currentFlightMode != previousMode)
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.print("Flight mode changed to: ");
            switch (currentFlightMode)
            {
            case FLIGHT_MODE_DISARMED:
                Serial.println("DISARMED");
                break;
            case FLIGHT_MODE_MANUAL:
                Serial.println("MANUAL");
                break;
            case FLIGHT_MODE_STABILIZE:
                Serial.println("STABILIZE");
                break;
            case FLIGHT_MODE_ALTITUDE_HOLD:
                Serial.println("ALTITUDE HOLD");
                break;
            case FLIGHT_MODE_POSITION_HOLD:
                Serial.println("POSITION HOLD");
                break;
            }
            xSemaphoreGive(serialMutex);
        }
        previousMode = currentFlightMode;
    }
}

// Get filtered altitude from barometric sensor
float getFilteredAltitude()
{
    // Prefer GPS altitude when available and reliable, else fall back to barometric
    float baroAltM = 0.0f;
    uint8_t sats = 0;
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        baroAltM = telemetryData.altitude / 100.0f; // cm -> m
        sats = telemetryData.satellites;
        xSemaphoreGive(telemetryMutex);
    }

    float gpsAlt = gpsAltitudeMeters;
    bool useGPS = (sats >= 4) && !isnan(gpsAlt) && fabs(gpsAlt) < 10000.0f;
    return useGPS ? gpsAlt : baroAltM;
}
