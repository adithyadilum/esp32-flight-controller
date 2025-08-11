#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
// #include <SPI.h> // SPI not needed in WiFi-only build (NRF24 removed)
// #include <RF24.h> // Radio removed
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h> // GPS kept (future altitude fusion & UI fields reference altitude_gps)
// Unused environmental / light / air-quality / ToF sensors removed for WiFi-only controller:
// #include <BH1750.h>
// #include "ScioSense_ENS160.h"
// #include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
// #include <VL53L0X.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// WiFi Credentials
const char *ssid = "Dialog 4G";       // Change this to your WiFi name
const char *password = "0N7NT00ANTQ"; // Change this to your WiFi password

// Web Server
WebServer server(80);

// Component Pins
// NRF24L01 pins removed (radio not used in WiFi build)
// #define NRF_CE_PIN 4
// #define NRF_CSN_PIN 5
#define ESC1_PIN 13 // Front Right (CCW)
#define ESC2_PIN 12 // Back Right (CW)
#define ESC3_PIN 14 // Front Left (CW)
#define ESC4_PIN 27 // Back Left (CCW)
#define LED_R1_PIN 25
#define LED_G1_PIN 33
#define LED_B1_PIN 32
#define BUZZER_PIN 26
#define SDA_PIN 21
#define SCL_PIN 22
#define GUVA_PIN 36
#define GPS_RX_PIN 16
#define BATTERY_PIN 35

// Navigation LED Pins
#define NAV_FRONT_RIGHT_GREEN 32
#define NAV_BACK_RIGHT_GREEN 33
#define NAV_BACK_LEFT_RED 25
#define NAV_FRONT_CENTER_WHITE 17
#define NAV_FRONT_LEFT_RED 2
#define NAV_BACK_CENTER_WHITE 15

// PCA9548A I2C Multiplexer
// #define PCA9548A_ADDR 0x70 // I2C multiplexer not used (ToF sensors removed)

// Control Loop Timing
#define CONTROL_LOOP_FREQUENCY 100                          // Hz
#define CONTROL_LOOP_PERIOD (1000 / CONTROL_LOOP_FREQUENCY) // ms

// Motor Limits
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_ARM_VALUE 1000
#define MOTOR_IDLE 1100
#define MOTOR_HOVER_ESTIMATE 1550 // Will be calibrated

// Component Objects
// RF24 radio removed – WiFi dashboard provides control inputs
Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
// Removed unused environmental sensor objects (ENS160, AHT21, BH1750)
TinyGPSPlus gps;

// Removed ToF sensor objects (VL53L0X) – not used in stabilization logic yet

// ================================
// CUSTOM PID CONTROLLER IMPLEMENTATION
// ================================
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

    // Integrator gating & leak
    bool allowIntegral; // External gating flag (e.g. throttle above gate)
    double iTermLeak;   // Leak factor per compute call (fraction of iTerm to bleed off when active)

    // Derivative filtering
    double derivativeAlpha; // Low-pass filter coefficient for derivative

    // Setpoint smoothing
    double setpointAlpha; // Smoothing factor for setpoint changes

    // Anti-windup flags
    bool wasOutputSaturated;

    // Diagnostic variables for web dashboard
    double proportional_term;
    double integral_term;
    double derivative_term;
    bool output_saturated;
    bool integral_saturated;

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

        allowIntegral = true;
        iTermLeak = 0.001; // Default very small leak (0.1% per compute)

        // Filter coefficients (higher = more filtering)
        derivativeAlpha = 0.1; // 10% new, 90% old for derivative
        setpointAlpha = 0.05;  // 5% new, 95% old for setpoint smoothing

        // Initialize diagnostic variables
        proportional_term = 0;
        integral_term = 0;
        derivative_term = 0;
        output_saturated = false;
        integral_saturated = false;
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
        proportional_term = kp * error;

        // Integral term handling with gating and leak
        // Only accumulate if integral allowed AND (not saturated OR error driving out of saturation)
        bool shouldAccumulateIntegral = allowIntegral && (!wasOutputSaturated ||
                                                          (wasOutputSaturated && ((error > 0 && output <= outputMin) ||
                                                                                  (error < 0 && output >= outputMax))));

        if (shouldAccumulateIntegral && ki != 0.0)
        {
            iTerm += ki * error * dt;
        }
        else
        {
            // Apply leak when not accumulating to bleed off stored integral (prevents lingering bias)
            if (iTermLeak > 0.0 && iTerm != 0.0)
            {
                double leakFactor = (1.0 - iTermLeak);
                if (leakFactor < 0.0)
                    leakFactor = 0.0;
                if (leakFactor > 1.0)
                    leakFactor = 1.0;
                iTerm *= leakFactor;
            }
        }
        // Clamp integral term to prevent windup
        integral_saturated = false;
        if (iTerm > iTermMax)
        {
            iTerm = iTermMax;
            integral_saturated = true;
        }
        else if (iTerm < iTermMin)
        {
            iTerm = iTermMin;
            integral_saturated = true;
        }
        integral_term = iTerm;

        // Derivative term with filtered input to reduce noise
        double dInput = 0;
        if (dt > 0)
        {
            dInput = (filteredInput - lastFilteredInput) / dt;
        }
        derivative_term = -kd * dInput; // Negative because we want to reduce rate of change

        // Compute final output
        double rawOutput = proportional_term + integral_term + derivative_term;

        // Output saturation/clamping
        output = rawOutput;
        output_saturated = false;
        if (output > outputMax)
        {
            output = outputMax;
            wasOutputSaturated = true;
            output_saturated = true;
        }
        else if (output < outputMin)
        {
            output = outputMin;
            wasOutputSaturated = true;
            output_saturated = true;
        }
        else
        {
            wasOutputSaturated = false;
        }

        // Remember values for next iteration
        lastInput = input;
        lastFilteredInput = filteredInput;
        lastTime = now;

        return output;
    }

    // Inline diagnostic getters (used by web dashboard)
    double getProportionalTerm() const { return proportional_term; }
    double getIntegralTerm() const { return integral_term; }
    double getDerivativeTerm() const { return derivative_term; }
    bool isOutputSaturated() const { return output_saturated; }
    bool isIntegralSaturated() const { return integral_saturated; }

    void reset()
    {
        iTerm = 0;
        lastInput = input;
        lastFilteredInput = input;
        filteredInput = input;
        smoothedSetpoint = setpoint;
        lastTime = millis();
        wasOutputSaturated = false;
        output_saturated = false;
        integral_saturated = false;
        proportional_term = 0;
        integral_term = 0;
        derivative_term = 0;
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

    void setMode(bool enable)
    {
        if (enable && !enabled)
        {
            reset();
        }
        enabled = enable;
    }

    // (Getters inserted later in struct)
}; // End of PIDController struct

// ================================
// Global RTOS handles & synchronization primitives
// (Were accidentally injected inside PIDController; restored here)
// ================================
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
SemaphoreHandle_t sensorMutex = NULL;
QueueHandle_t commandQueue = NULL;

// Flight Controller State
enum FlightMode
{
    MODE_DISARMED = 0,
    MODE_ARMED = 1,
    MODE_MANUAL = 2,
    MODE_STABILIZE = 3,
    MODE_ALTITUDE_HOLD = 4,
    MODE_POSITION_HOLD = 5,
    MODE_EMERGENCY = 6
};

struct FlightState
{
    FlightMode mode;
    bool motors_armed;
    bool emergency_stop;
    bool gps_available;
    bool altitude_hold_active;
    float hover_throttle;
    unsigned long last_command_time;
    unsigned long arm_time;
};

// Custom PID Controllers - Using advanced controller from droneFreeRTOS.ino
PIDController rollPID, pitchPID, yawPID, altitudePID;

// PID Parameters (will be tuned) - Enhanced values for better stabilization
struct PIDParams
{
    double roll_kp, roll_ki, roll_kd;
    double pitch_kp, pitch_ki, pitch_kd;
    double yaw_kp, yaw_ki, yaw_kd;
    double altitude_kp, altitude_ki, altitude_kd;
};

// Initialize with enhanced values for better stabilization
PIDParams pidParams = {
    // Roll PID - Enhanced for stronger correction with anti-windup
    .roll_kp = 2.5,  // Balanced response
    .roll_ki = 0.1,  // Small integral for steady-state error
    .roll_kd = 0.15, // Damping for smooth control
    // Pitch PID - Enhanced for stronger correction with anti-windup
    .pitch_kp = 2.5,  // Balanced response
    .pitch_ki = 0.1,  // Small integral for steady-state error
    .pitch_kd = 0.15, // Damping for smooth control
    // Yaw PID - Enhanced for better authority with rate control
    .yaw_kp = 1.0,  // Lower gain for yaw
    .yaw_ki = 0.05, // Very small integral for yaw
    .yaw_kd = 0.05, // Minimal damping for yaw
    // Altitude PID
    .altitude_kp = 1.5,
    .altitude_ki = 0.2,
    .altitude_kd = 0.8};

// Control Inputs (from WiFi dashboard or NRF24L01)
struct ControlInputs
{
    float throttle; // 0-1 (0 = motor_idle, 1 = motor_max)
    float roll;     // -1 to 1 (left/right)
    float pitch;    // -1 to 1 (forward/backward)
    float yaw;      // -1 to 1 (rotate left/right)
    bool arm_switch;
    bool mode_switch;
    unsigned long timestamp;
};

// Sensor Data Structure
struct SensorData
{
    // IMU Data
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float accel_x, accel_y, accel_z;

    // Altitude Data
    float altitude_baro;
    float altitude_gps;
    float altitude_filtered;
    float vertical_velocity;

    // Position Data (GPS)
    double latitude;
    double longitude;
    float ground_speed;
    int satellites;

    // Environmental
    float temperature;
    float humidity;
    float pressure;
    float air_quality_index;

    // Battery
    float battery_voltage;
    float battery_percentage;

    // System
    unsigned long timestamp;
    bool valid;
};

// IMU Calibration Structure
struct IMUCalibration
{
    float roll_offset;
    float pitch_offset;
    float yaw_offset;
    float accel_x_offset;
    float accel_y_offset;
    float accel_z_offset;
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
    bool calibrated;
};

// IMU calibration data - will be set during startup calibration
IMUCalibration imuCal = {
    .roll_offset = 0.0,
    .pitch_offset = 0.0,
    .yaw_offset = 0.0,
    .accel_x_offset = 0.0,
    .accel_y_offset = 0.0,
    .accel_z_offset = 0.0,
    .gyro_x_offset = 0.0,
    .gyro_y_offset = 0.0,
    .gyro_z_offset = 0.0,
    .calibrated = false};

// Global State Variables
FlightState flightState;
ControlInputs controlInputs;
SensorData sensorData;

// Motor outputs (PWM values)
struct MotorOutputs
{
    int motor1; // Front Right (CCW)
    int motor2; // Back Right (CW)
    int motor3; // Front Left (CW)
    int motor4; // Back Left (CCW)
};

// Debug Information Structure for Web Dashboard
struct DebugData
{
    // Status Section
    String flight_mode;
    bool armed;
    float throttle_input;
    int throttle_pwm;
    float roll_angle;
    float pitch_angle;
    float yaw_rate;
    float altitude;
    float battery_voltage;

    // PID Debug Section - Enhanced with custom PID diagnostics
    bool pid_active;
    bool pid_enabled_by_throttle;
    float roll_output;
    float pitch_output;
    float yaw_output;
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;

    // Advanced PID Diagnostics
    float roll_proportional;
    float roll_integral;
    float roll_derivative;
    bool roll_output_saturated;
    bool roll_integral_saturated;

    float pitch_proportional;
    float pitch_integral;
    float pitch_derivative;
    bool pitch_output_saturated;
    bool pitch_integral_saturated;

    float yaw_proportional;
    float yaw_integral;
    float yaw_derivative;
    bool yaw_output_saturated;
    bool yaw_integral_saturated;

    // Calibration Debug Section
    bool imu_cal_active;
    float roll_cal_offset;
    float pitch_cal_offset;

    // Motor Mix Section
    int base_pwm;
    int motor1_final;
    int motor2_final;
    int motor3_final;
    int motor4_final;
    float motor1_offset;
    float motor2_offset;
    float motor3_offset;
    float motor4_offset;

    // Inputs Section
    float roll_input;
    float pitch_input;
    float yaw_input;

    // System Section
    bool sensors_ok;
    bool emergency_active;
    bool imu_calibrated;
    unsigned long uptime;
    unsigned int free_heap;

    // Update timestamp
    unsigned long timestamp;
};

MotorOutputs motorOutputs;

// Debug data for web dashboard
DebugData debugData;

// Motor calibration offsets to compensate for mechanical imbalances
// Negative values reduce motor power, positive values increase it
struct MotorCalibration
{
    float motor1_offset; // Front Right (CCW)
    float motor2_offset; // Back Right (CW)
    float motor3_offset; // Front Left (CW) (reduce if left side lifts early)
    float motor4_offset; // Back Left (CCW) (reduce if left side lifts early)
};

// Motor calibration offsets (applied only in MANUAL mode). Defaults to zero.
MotorCalibration motorCal = {
    .motor1_offset = 0.0, // Front Right (CCW)
    .motor2_offset = 0.0, // Back Right (CW)
    .motor3_offset = 0.0, // Front Left (CW)
    .motor4_offset = 0.0  // Back Left (CCW)
};

// =====================================
// Unified Motor Mixing (Matches droneFreeRTOS)
// =====================================
// Tunable mixer gains (μs contribution at full normalized input |R|=|P|=|Y|=1)
// Aligned with RF (droneFreeRTOS.ino) build for cross-build tuning consistency.
// If you need higher authority, adjust here AND in the RF version together.
float MIX_KR = 100.0f; // Roll gain (was 200.0f pre-unification)
float MIX_KP = 100.0f; // Pitch gain (was 200.0f)
float MIX_KY = 75.0f;  // Yaw gain (was 150.0f)

// =====================================
// Live-tunable control enhancement variables (mirroring droneFreeRTOS)
// =====================================
float manualAxisGain = 1.30f; // Extra authority in manual mode (scales stick R/P/Y)
float throttleGate = 0.35f;   // Below this throttle, disable PID integrators (and optionally entire PID)
float slewRateRP = 180.0f;    // deg/sec max change for roll/pitch angle setpoints (slew limiter)
float slewRateYaw = 360.0f;   // deg/sec^2 equivalent for yaw rate command smoothing (simplified)
float yawFeedForward = 0.15f; // Fraction of yaw setpoint added directly to yaw PID output

// Advanced fusion/control feature toggles (default off to preserve current behavior)
bool USE_KALMAN_ATTITUDE = false;     // If true, use 1D Kalman filter for roll/pitch angles
bool USE_CASCADED_ANGLE_RATE = false; // If true, use angle->rate cascaded loops for roll/pitch

// Cascaded inner rate PID controllers (roll/pitch); yaw already rate-controlled by yawPID
PIDController rollRatePID, pitchRatePID;

// Rate PID tunables and limits
struct RatePIDParams
{
    double roll_kp, roll_ki, roll_kd;
    double pitch_kp, pitch_ki, pitch_kd;
};

RatePIDParams ratePidParams = {
    // conservative defaults; tune live via dashboard
    .roll_kp = 0.10,
    .roll_ki = 0.02,
    .roll_kd = 0.002,
    .pitch_kp = 0.10,
    .pitch_ki = 0.02,
    .pitch_kd = 0.002};

float MAX_ROLL_PITCH_RATE = 180.0f; // deg/s limit for outer angle loop output (desired rate)

// 1D Kalman filter state for attitude (roll/pitch)
struct Kalman1D
{
    float x; // angle estimate (deg)
    float p; // estimate variance
    bool initialized;
} kalmanRoll = {0, 1, false}, kalmanPitch = {0, 1, false};

// Kalman noise parameters (tunable via dashboard)
float KALMAN_RATE_STD = 4.0f; // deg/s (gyro rate noise std)
float KALMAN_MEAS_STD = 3.0f; // deg (accel angle noise std)

// Slew limiter state
static float prevRollSetpointCmd = 0.0f;
static float prevPitchSetpointCmd = 0.0f;
static float prevYawRateCmd = 0.0f;
static unsigned long prevSlewTimeMs = 0;

// ESC pulse bounds (use existing defines)
#ifndef PWM_MIN_PULSE
#define PWM_MIN_PULSE MOTOR_MIN
#endif
#ifndef PWM_MAX_PULSE
#define PWM_MAX_PULSE MOTOR_MAX
#endif

// Scaled mixer applying dynamic saturation management (same algorithm as droneFreeRTOS)
// Writes directly into motorOutputs.*
static inline void applyScaledMotorMix(float baseMicros, float Rn, float Pn, float Yn)
{
    // Constrain base
    if (baseMicros < PWM_MIN_PULSE)
        baseMicros = PWM_MIN_PULSE;
    if (baseMicros > PWM_MAX_PULSE)
        baseMicros = PWM_MAX_PULSE;

    // Raw mixes (direction pattern identical to droneFreeRTOS M1..M4 order)
    float m1 = -MIX_KR * Rn - MIX_KP * Pn + MIX_KY * Yn; // Front Right (CCW)
    float m2 = -MIX_KR * Rn + MIX_KP * Pn - MIX_KY * Yn; // Back Right (CW)
    float m3 = MIX_KR * Rn - MIX_KP * Pn - MIX_KY * Yn;  // Front Left (CW)
    float m4 = MIX_KR * Rn + MIX_KP * Pn + MIX_KY * Yn;  // Back Left (CCW)

    // Find mix extremes
    float posMax = 0.0f, negMin = 0.0f;
    float mixes[4] = {m1, m2, m3, m4};
    for (int i = 0; i < 4; ++i)
    {
        if (mixes[i] > posMax)
            posMax = mixes[i];
        if (mixes[i] < negMin)
            negMin = mixes[i];
    }

    float s_pos = 1.0f;
    float s_neg = 1.0f;
    if (posMax > 0.0f)
    {
        float headroomHigh = PWM_MAX_PULSE - baseMicros;
        if (headroomHigh < 0)
            headroomHigh = 0;
        s_pos = headroomHigh / posMax;
        if (s_pos > 1.0f)
            s_pos = 1.0f;
    }
    if (negMin < 0.0f)
    {
        float headroomLow = baseMicros - PWM_MIN_PULSE;
        if (headroomLow < 0)
            headroomLow = 0;
        s_neg = headroomLow / (-negMin);
        if (s_neg > 1.0f)
            s_neg = 1.0f;
    }

    // Apply scaling
    float f1 = baseMicros + (m1 >= 0 ? s_pos * m1 : s_neg * m1);
    float f2 = baseMicros + (m2 >= 0 ? s_pos * m2 : s_neg * m2);
    float f3 = baseMicros + (m3 >= 0 ? s_pos * m3 : s_neg * m3);
    float f4 = baseMicros + (m4 >= 0 ? s_pos * m4 : s_neg * m4);

    // Clamp
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

    motorOutputs.motor1 = (int)f1;
    motorOutputs.motor2 = (int)f2;
    motorOutputs.motor3 = (int)f3;
    motorOutputs.motor4 = (int)f4;
}

// Function Declarations
void initializeHardware();
void initializePIDControllers();
void setupWebServer();
void connectWiFi();

// FreeRTOS Task Functions
void controlTask(void *parameter);
void sensorTask(void *parameter);
void telemetryTask(void *parameter);

// Core Flight Functions
void updateSensors();
void calculateOrientation();
void runPIDControllers();
void mixMotorOutputs();
void outputMotorCommands();
void checkSafetyConditions();
void handleEmergency();

// Utility Functions
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float constrainFloat(float value, float min_val, float max_val);
void updateFlightMode();
void calibrateIMU();

// LED Control Functions
void initializeLEDs();
void updateNavigationLights();
void updateStatusLights();
void setThrottlePulse(float throttle_percentage);

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 Weather Drone Flight Controller Starting...");

    // Initialize hardware
    initializeHardware();

    // Calibrate IMU to set proper setpoints for level flight
    Serial.println("Starting IMU calibration...");
    calibrateIMU();
    Serial.println("IMU calibration completed!");

    // Initialize flight state
    flightState.mode = MODE_DISARMED;
    flightState.motors_armed = false;
    flightState.emergency_stop = false;
    flightState.gps_available = false;
    flightState.altitude_hold_active = false; // Ensure altitude hold starts disabled
    flightState.hover_throttle = MOTOR_HOVER_ESTIMATE;
    flightState.last_command_time = 0;
    flightState.arm_time = 0;

    // Initialize control inputs
    memset(&controlInputs, 0, sizeof(controlInputs));
    memset(&sensorData, 0, sizeof(sensorData));

    // Set initial sensor timestamp to prevent immediate emergency
    sensorData.timestamp = millis();

    // Initialize PID controllers
    initializePIDControllers();

    // Connect to WiFi
    connectWiFi();

    // Setup web server
    setupWebServer();

    // Create FreeRTOS synchronization primitives
    sensorMutex = xSemaphoreCreateMutex();
    commandQueue = xQueueCreate(10, sizeof(ControlInputs));

    // Create FreeRTOS tasks with larger stack sizes
    xTaskCreatePinnedToCore(
        controlTask,        // Function
        "Control Task",     // Name
        8192,               // Stack size (increased)
        NULL,               // Parameters
        3,                  // Priority (higher = more important)
        &controlTaskHandle, // Handle
        1                   // Core (0 or 1)
    );

    xTaskCreatePinnedToCore(
        sensorTask,        // Function
        "Sensor Task",     // Name
        8192,              // Stack size (increased)
        NULL,              // Parameters
        2,                 // Priority
        &sensorTaskHandle, // Handle
        0                  // Core
    );

    xTaskCreatePinnedToCore(
        telemetryTask,        // Function
        "Telemetry Task",     // Name
        4096,                 // Stack size (doubled)
        NULL,                 // Parameters
        1,                    // Priority
        &telemetryTaskHandle, // Handle
        0                     // Core
    );

    Serial.println("Flight Controller initialized successfully!");
    Serial.println("Starting control tasks...");
}

void loop()
{
    // Handle web server requests
    server.handleClient();

    // Main loop runs at lower priority - FreeRTOS handles the control loops
    delay(10);
}

void initializeHardware()
{
    // Initialize pins
    pinMode(LED_R1_PIN, OUTPUT);
    pinMode(LED_G1_PIN, OUTPUT);
    pinMode(LED_B1_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(GUVA_PIN, INPUT);
    pinMode(BATTERY_PIN, INPUT);

    // Initialize navigation LED pins
    initializeLEDs();

    // Set ADC attenuation for battery monitoring
    analogSetAttenuation(ADC_11db);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz for faster sensor reads

    // SPI init removed (only used for NRF24L01 previously). Re-enable if future peripherals require SPI.

    // Initialize MPU6050 IMU
    if (mpu.begin(0x68))
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("MPU6050 initialized successfully");
    }
    else
    {
        Serial.println("ERROR: MPU6050 initialization failed!");
    }

    // Initialize BME280
    if (bme.begin(0x76) || bme.begin(0x77))
    {
        Serial.println("BME280 initialized successfully");
    }
    else
    {
        Serial.println("ERROR: BME280 initialization failed!");
    }

    // Initialize ESCs (pre-calibrated to 1000–2000µs range; no runtime calibration needed)
    esc1.attach(ESC1_PIN, MOTOR_MIN, MOTOR_MAX);
    esc2.attach(ESC2_PIN, MOTOR_MIN, MOTOR_MAX);
    esc3.attach(ESC3_PIN, MOTOR_MIN, MOTOR_MAX);
    esc4.attach(ESC4_PIN, MOTOR_MIN, MOTOR_MAX);

    // Immediately drive all motors to minimum for safety
    esc1.writeMicroseconds(MOTOR_ARM_VALUE);
    esc2.writeMicroseconds(MOTOR_ARM_VALUE);
    esc3.writeMicroseconds(MOTOR_ARM_VALUE);
    esc4.writeMicroseconds(MOTOR_ARM_VALUE);

    Serial.println("Hardware initialization complete");

    // Brief startup sequence using navigation LEDs
    // Red LEDs first
    digitalWrite(NAV_BACK_LEFT_RED, HIGH);
    digitalWrite(NAV_FRONT_LEFT_RED, HIGH);
    delay(200);
    digitalWrite(NAV_BACK_LEFT_RED, LOW);
    digitalWrite(NAV_FRONT_LEFT_RED, LOW);

    // Green LEDs next
    digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
    digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
    delay(200);

    // White center LEDs last
    digitalWrite(NAV_FRONT_CENTER_WHITE, HIGH);
    digitalWrite(NAV_BACK_CENTER_WHITE, HIGH);
    delay(200);
    digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
    digitalWrite(NAV_BACK_CENTER_WHITE, LOW);

    // Startup beep
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
}

void initializePIDControllers()
{
    // Initialize custom PID controllers with advanced features
    rollPID.initialize(pidParams.roll_kp, pidParams.roll_ki, pidParams.roll_kd, -400, 400);
    rollPID.setDerivativeFilter(0.1);   // Reduce noise
    rollPID.setSetpointSmoothing(0.05); // Smooth setpoint changes

    pitchPID.initialize(pidParams.pitch_kp, pidParams.pitch_ki, pidParams.pitch_kd, -400, 400);
    pitchPID.setDerivativeFilter(0.1);   // Reduce noise
    pitchPID.setSetpointSmoothing(0.05); // Smooth setpoint changes

    yawPID.initialize(pidParams.yaw_kp, pidParams.yaw_ki, pidParams.yaw_kd, -200, 200);
    yawPID.setDerivativeFilter(0.15); // More filtering for yaw (noisier)
    yawPID.setSetpointSmoothing(0.1); // More smoothing for yaw commands

    altitudePID.initialize(pidParams.altitude_kp, pidParams.altitude_ki, pidParams.altitude_kd, 0, 1000);
    altitudePID.setDerivativeFilter(0.05);  // Light filtering for altitude
    altitudePID.setSetpointSmoothing(0.02); // Gentle altitude setpoint changes

    // Initialize inner rate PIDs (disabled by default unless cascaded enabled)
    rollRatePID.initialize(ratePidParams.roll_kp, ratePidParams.roll_ki, ratePidParams.roll_kd, -400, 400);
    rollRatePID.setDerivativeFilter(0.2);
    rollRatePID.setSetpointSmoothing(0.05);
    pitchRatePID.initialize(ratePidParams.pitch_kp, ratePidParams.pitch_ki, ratePidParams.pitch_kd, -400, 400);
    pitchRatePID.setDerivativeFilter(0.2);
    pitchRatePID.setSetpointSmoothing(0.05);

    Serial.println("Custom PID controllers initialized with advanced features:");
    Serial.println("- Anti-windup protection");
    Serial.println("- Derivative filtering");
    Serial.println("- Setpoint smoothing");
    Serial.println("- Output saturation protection");
    Serial.printf("PID Gains - Roll: P=%.2f I=%.2f D=%.3f, Pitch: P=%.2f I=%.2f D=%.3f, Yaw: P=%.2f I=%.2f D=%.3f\n",
                  pidParams.roll_kp, pidParams.roll_ki, pidParams.roll_kd,
                  pidParams.pitch_kp, pidParams.pitch_ki, pidParams.pitch_kd,
                  pidParams.yaw_kp, pidParams.yaw_ki, pidParams.yaw_kd);
    Serial.printf("Rate PID Gains - RollRate: P=%.2f I=%.2f D=%.3f, PitchRate: P=%.2f I=%.2f D=%.3f\n",
                  ratePidParams.roll_kp, ratePidParams.roll_ki, ratePidParams.roll_kd,
                  ratePidParams.pitch_kp, ratePidParams.pitch_ki, ratePidParams.pitch_kd);
}

void connectWiFi()
{
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println();
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println();
        Serial.println("WiFi connection failed!");
    }
}

// We'll continue with the task implementations in the next part...

// ================================
// FREERTOS TASK IMPLEMENTATIONS
// ================================

void controlTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD);

    Serial.println("Control Task started - Running at 100Hz");

    while (true)
    {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Check safety conditions first
        checkSafetyConditions();

        if (flightState.emergency_stop)
        {
            handleEmergency();
            continue;
        }

        // Update flight mode based on inputs
        updateFlightMode();

        // Update LED status indicators
        updateStatusLights();
        updateNavigationLights();

        // Comprehensive Debug Data Collection for Web Dashboard
        static int debug_counter = 0;
        if (++debug_counter >= 100)
        { // Every 1 second at 100Hz for smooth web updates
            debug_counter = 0;

            // Update debug data structure for web dashboard
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(2)) == pdTRUE)
            {
                // Status Section
                switch (flightState.mode)
                {
                case MODE_MANUAL:
                    debugData.flight_mode = "MANUAL";
                    break;
                case MODE_STABILIZE:
                    debugData.flight_mode = "STABILIZE";
                    break;
                case MODE_ALTITUDE_HOLD:
                    debugData.flight_mode = "ALT_HOLD";
                    break;
                case MODE_POSITION_HOLD:
                    debugData.flight_mode = "POS_HOLD";
                    break;
                case MODE_EMERGENCY:
                    debugData.flight_mode = "EMERGENCY";
                    break;
                default:
                    debugData.flight_mode = "DISARMED";
                    break;
                }

                debugData.armed = flightState.motors_armed;
                debugData.throttle_input = controlInputs.throttle;
                debugData.throttle_pwm = mapFloat(controlInputs.throttle, 0.0, 1.0, MOTOR_IDLE, MOTOR_MAX);
                debugData.roll_angle = sensorData.roll_angle;
                debugData.pitch_angle = sensorData.pitch_angle;
                debugData.yaw_rate = sensorData.yaw_rate;
                debugData.altitude = sensorData.altitude_filtered;
                debugData.battery_voltage = sensorData.battery_voltage;

                // PID Debug Section - Enhanced with custom PID diagnostics
                debugData.pid_active = (flightState.mode >= MODE_STABILIZE);
                debugData.pid_enabled_by_throttle = (controlInputs.throttle > throttleGate); // Reflect live throttle gate
                debugData.roll_output = rollPID.output;
                debugData.pitch_output = pitchPID.output;
                debugData.yaw_output = yawPID.output;
                debugData.roll_setpoint = rollPID.setpoint;
                debugData.pitch_setpoint = pitchPID.setpoint;
                debugData.yaw_setpoint = yawPID.setpoint;

                // Advanced PID Diagnostics - Individual P, I, D components and saturation status
                debugData.roll_proportional = rollPID.getProportionalTerm();
                debugData.roll_integral = rollPID.getIntegralTerm();
                debugData.roll_derivative = rollPID.getDerivativeTerm();
                debugData.roll_output_saturated = rollPID.isOutputSaturated();
                debugData.roll_integral_saturated = rollPID.isIntegralSaturated();

                debugData.pitch_proportional = pitchPID.getProportionalTerm();
                debugData.pitch_integral = pitchPID.getIntegralTerm();
                debugData.pitch_derivative = pitchPID.getDerivativeTerm();
                debugData.pitch_output_saturated = pitchPID.isOutputSaturated();
                debugData.pitch_integral_saturated = pitchPID.isIntegralSaturated();

                debugData.yaw_proportional = yawPID.getProportionalTerm();
                debugData.yaw_integral = yawPID.getIntegralTerm();
                debugData.yaw_derivative = yawPID.getDerivativeTerm();
                debugData.yaw_output_saturated = yawPID.isOutputSaturated();
                debugData.yaw_integral_saturated = yawPID.isIntegralSaturated();
                debugData.pitch_output_saturated = pitchPID.isOutputSaturated();
                debugData.pitch_integral_saturated = pitchPID.isIntegralSaturated();

                debugData.yaw_proportional = yawPID.getProportionalTerm();
                debugData.yaw_integral = yawPID.getIntegralTerm();
                debugData.yaw_derivative = yawPID.getDerivativeTerm();
                debugData.yaw_output_saturated = yawPID.isOutputSaturated();
                debugData.yaw_integral_saturated = yawPID.isIntegralSaturated();

                // Calibration Debug Section
                debugData.imu_cal_active = imuCal.calibrated;
                debugData.roll_cal_offset = imuCal.roll_offset;
                debugData.pitch_cal_offset = imuCal.pitch_offset;

                // Motor Mix Section
                debugData.base_pwm = debugData.throttle_pwm;
                debugData.motor1_final = motorOutputs.motor1;
                debugData.motor2_final = motorOutputs.motor2;
                debugData.motor3_final = motorOutputs.motor3;
                debugData.motor4_final = motorOutputs.motor4;
                debugData.motor1_offset = motorCal.motor1_offset;
                debugData.motor2_offset = motorCal.motor2_offset;
                debugData.motor3_offset = motorCal.motor3_offset;
                debugData.motor4_offset = motorCal.motor4_offset;

                // Inputs Section
                debugData.roll_input = controlInputs.roll;
                debugData.pitch_input = controlInputs.pitch;
                debugData.yaw_input = controlInputs.yaw;

                // System Section
                debugData.sensors_ok = sensorData.valid;
                debugData.emergency_active = flightState.emergency_stop;
                debugData.imu_calibrated = imuCal.calibrated;
                debugData.uptime = millis();
                debugData.free_heap = ESP.getFreeHeap();
                debugData.timestamp = millis();

                xSemaphoreGive(sensorMutex);
            }
        }

        // Only run control loops if armed and not in emergency
        if (flightState.motors_armed)
        {
            if (flightState.mode == MODE_MANUAL)
            {
                // Manual mode - use unified mixer (handles base throttle, inputs, scaling, and offsets)
                mixMotorOutputs();
                outputMotorCommands();
            }
            else if (flightState.mode >= MODE_STABILIZE)
            {
                // Get latest sensor data (thread-safe)
                if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    // Copy sensor data to PID controller inputs
                    rollPID.input = sensorData.roll_angle;
                    pitchPID.input = sensorData.pitch_angle;
                    yawPID.input = sensorData.yaw_rate;
                    altitudePID.input = sensorData.altitude_filtered;

                    xSemaphoreGive(sensorMutex);

                    // Run PID controllers
                    runPIDControllers();

                    // Mix motor outputs
                    mixMotorOutputs();

                    // Output to motors
                    outputMotorCommands();
                }
            }
        }
        else
        {
            // Motors disarmed - ensure all motors at idle
            motorOutputs.motor1 = MOTOR_ARM_VALUE;
            motorOutputs.motor2 = MOTOR_ARM_VALUE;
            motorOutputs.motor3 = MOTOR_ARM_VALUE;
            motorOutputs.motor4 = MOTOR_ARM_VALUE;
            outputMotorCommands();
        }
    }
}

void sensorTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz sensor reading

    Serial.println("Sensor Task started - Running at 200Hz");

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Update all sensor readings
        updateSensors();

        // Calculate orientation from IMU
        calculateOrientation();

        // Update timestamp
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            sensorData.timestamp = millis();
            sensorData.valid = true;
            xSemaphoreGive(sensorMutex);
        }
    }
}

void telemetryTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz telemetry

    Serial.println("Telemetry Task started - Running at 10Hz");

    // Wait a few seconds before first status print
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("=== FLIGHT CONTROLLER READY ===");
    Serial.printf("Web Dashboard: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("Waiting for commands via web interface...");

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Handle web server and telemetry
        // This will be expanded for NRF24L01 later

        // Print debug info every 5 seconds (reduce frequency further)
        static int counter = 0;
        if (++counter >= 50) // 5 seconds at 10Hz
        {
            counter = 0;

            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // Only print if we have valid sensor data
                if (sensorData.timestamp > 0)
                {
                    Serial.printf("Status - Mode: %d, Armed: %s, Roll: %.1f°, Pitch: %.1f°, Alt: %.1fm, Batt: %.1fV\n",
                                  flightState.mode,
                                  flightState.motors_armed ? "YES" : "NO",
                                  sensorData.roll_angle,
                                  sensorData.pitch_angle,
                                  sensorData.altitude_filtered,
                                  sensorData.battery_voltage);
                }
                xSemaphoreGive(sensorMutex);
            }
        }
    }
}

// ================================
// CORE FLIGHT CONTROL FUNCTIONS
// ================================

void updateSensors()
{
    static unsigned long lastBME280Read = 0;
    static unsigned long lastGPSRead = 0;
    static unsigned long lastBatteryRead = 0;

    // Read IMU data (high frequency)
    sensors_event_t accel, gyro, temp;
    bool imu_success = mpu.getEvent(&accel, &gyro, &temp);

    if (imu_success && xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        sensorData.accel_x = accel.acceleration.x;
        sensorData.accel_y = accel.acceleration.y;
        sensorData.accel_z = accel.acceleration.z;
        // Roll rate: removed previous inversion (was negated) to correct inverted roll response
        sensorData.roll_rate = gyro.gyro.x * 180.0 / PI;
        sensorData.pitch_rate = gyro.gyro.y * 180.0 / PI;
        sensorData.yaw_rate = gyro.gyro.z * 180.0 / PI;
        xSemaphoreGive(sensorMutex);
    }

    // Read BME280 (lower frequency)
    if (millis() - lastBME280Read > 50)
    { // 20Hz
        lastBME280Read = millis();

        float pressure = bme.readPressure();
        if (!isnan(pressure) && pressure > 0)
        {
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                sensorData.pressure = pressure / 100.0;               // Convert to hPa
                sensorData.altitude_baro = bme.readAltitude(1013.25); // Sea level pressure
                sensorData.temperature = bme.readTemperature();
                sensorData.humidity = bme.readHumidity();
                xSemaphoreGive(sensorMutex);
            }
        }
    }

    // GPS reading disabled for now to prevent errors
    /*
    // Read GPS (lower frequency)
    if (millis() - lastGPSRead > 100)
    { // 10Hz
        lastGPSRead = millis();
        // GPS code commented out until Serial2 is properly initialized
    }
    */

    // Read battery (very low frequency)
    if (millis() - lastBatteryRead > 1000)
    { // 1Hz
        lastBatteryRead = millis();

        int rawValue = analogRead(BATTERY_PIN);
        float vOut = (rawValue / 4095.0) * 3.3;

        // For 3S LiPo battery reading - adjust voltage divider ratio
        // If no voltage divider is used, multiply by appropriate factor
        // Common voltage divider: R1=10k, R2=10k (divide by 2) = vOut * 2
        // For 3S LiPo (12.6V max): need divide by ~4, so vOut * 4
        // Empirical correction: 3.4V reading suggests vOut * 3.7 for 12.6V
        float batteryVoltage = vOut * 3.7; // Adjusted for actual hardware setup

        // Debug battery reading every 10 seconds
        static int battery_debug_counter = 0;
        if (++battery_debug_counter >= 10)
        {
            battery_debug_counter = 0;
            Serial.printf("Battery Debug - Raw ADC: %d, vOut: %.2f, Final: %.2f\n", rawValue, vOut, batteryVoltage);
        }

        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            sensorData.battery_voltage = batteryVoltage;
            // Calculate percentage (3S LiPo: 9.0V-12.6V) - display only
            sensorData.battery_percentage = ((batteryVoltage - 9.0) / (12.6 - 9.0)) * 100.0;
            sensorData.battery_percentage = constrainFloat(sensorData.battery_percentage, 0.0, 100.0);
            xSemaphoreGive(sensorMutex);
        }
    }
}

void calculateOrientation()
{
    static float roll_angle = 0.0;
    static float pitch_angle = 0.0;
    static bool prevCalibrated = false; // Track calibration edge
    static unsigned long lastUpdate = 0;

    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0; // Convert to seconds
    lastUpdate = now;

    if (dt > 0.1)
        dt = 0.1; // Limit dt to prevent large jumps

    // Get current sensor readings (thread-safe)
    float accel_x, accel_y, accel_z;
    float gyro_roll, gyro_pitch;

    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        accel_x = sensorData.accel_x;
        accel_y = sensorData.accel_y;
        accel_z = sensorData.accel_z;
        gyro_roll = sensorData.roll_rate;
        gyro_pitch = sensorData.pitch_rate;
        xSemaphoreGive(sensorMutex);
    }
    else
    {
        return; // Skip this update if we can't get sensor data
    }

    // Apply IMU calibration offsets if calibrated
    if (imuCal.calibrated)
    {
        accel_x -= imuCal.accel_x_offset;
        accel_y -= imuCal.accel_y_offset;
        accel_z -= imuCal.accel_z_offset;
        // Subtract stored gyro biases (convert stored rad/s to deg/s)
        gyro_roll -= (imuCal.gyro_x_offset * 180.0 / PI);
        gyro_pitch -= (imuCal.gyro_y_offset * 180.0 / PI);
    }

    // Calculate angles from accelerometer
    // Fix roll axis to match physical tilt direction
    // Remove leading negative to align roll angle sign with control input (previous sign caused inverted response)
    float accel_roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / PI;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI;

    // Apply calibration offsets to get relative angles from level position
    // Do not subtract additional angle offsets here; accelerometer biases are already
    // compensated above. Additional angle offset subtraction would double-correct and
    // introduce a residual bias when level. The complementary filter is seeded after
    // calibration to ensure the fused attitude starts at level.

    // If calibration was just completed, re-seed the complementary filter state
    // to the level-corrected accelerometer estimate to avoid residual bias.
    if (imuCal.calibrated && !prevCalibrated)
    {
        roll_angle = accel_roll;
        pitch_angle = accel_pitch;
    }
    prevCalibrated = imuCal.calibrated;

    // Fusion: complementary filter or 1D Kalman (optional)
    if (USE_KALMAN_ATTITUDE)
    {
        // Initialize on first use or on calibration edge
        if (!kalmanRoll.initialized || (imuCal.calibrated && !prevCalibrated))
        {
            kalmanRoll.x = accel_roll;
            kalmanRoll.p = 4.0f;
            kalmanRoll.initialized = true;
            kalmanPitch.x = accel_pitch;
            kalmanPitch.p = 4.0f;
            kalmanPitch.initialized = true;
        }
        // Prediction
        float q = (KALMAN_RATE_STD * KALMAN_RATE_STD) * dt; // process noise variance contribution
        kalmanRoll.x += gyro_roll * dt;
        kalmanRoll.p += q;
        kalmanPitch.x += gyro_pitch * dt;
        kalmanPitch.p += q;
        // Measurement update
        float r = (KALMAN_MEAS_STD * KALMAN_MEAS_STD);
        float k_r = kalmanRoll.p / (kalmanRoll.p + r);
        float k_p = kalmanPitch.p / (kalmanPitch.p + r);
        kalmanRoll.x += k_r * (accel_roll - kalmanRoll.x);
        kalmanRoll.p *= (1.0f - k_r);
        kalmanPitch.x += k_p * (accel_pitch - kalmanPitch.x);
        kalmanPitch.p *= (1.0f - k_p);
        roll_angle = kalmanRoll.x;
        pitch_angle = kalmanPitch.x;
    }
    else
    {
        // Complementary filter (combine gyro and accel)
        const float alpha = 0.98; // Gyro weight (higher = trust gyro more)
        roll_angle = alpha * (roll_angle + gyro_roll * dt) + (1.0f - alpha) * accel_roll;
        pitch_angle = alpha * (pitch_angle + gyro_pitch * dt) + (1.0f - alpha) * accel_pitch;
    }

    // Update global sensor data
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        sensorData.roll_angle = roll_angle;
        sensorData.pitch_angle = pitch_angle;

        // Simple altitude filtering (combine GPS and barometric)
        if (flightState.gps_available && sensorData.satellites >= 4)
        {
            // Use GPS altitude when available and reliable
            sensorData.altitude_filtered = sensorData.altitude_gps;
        }
        else
        {
            // Fall back to barometric altitude
            sensorData.altitude_filtered = sensorData.altitude_baro;
        }

        xSemaphoreGive(sensorMutex);
    }
}

void runPIDControllers()
{
    // Unified with RF controller logic: MANUAL mode passes raw sticks to mixer; STABILIZE enables angle/yaw rate PIDs.
    // Mixer gains have been reduced to 100/100/75 to match droneFreeRTOS.ino for consistent tuning behavior.
    // Time delta for slew limiting
    unsigned long nowMs = millis();
    float dt = (prevSlewTimeMs == 0) ? 0.01f : (float)(nowMs - prevSlewTimeMs) / 1000.0f;
    if (dt <= 0)
        dt = 0.01f; // fallback
    prevSlewTimeMs = nowMs;

    // Raw desired commands from sticks (pre-slew)
    // Invert roll stick here to compensate for previous sensor sign convention change
    float desiredRollDeg = (controlInputs.roll) * 30.0f; // +/-30°
    // Invert pitch: positive input should command nose-up (increase rear motor thrust)
    float desiredPitchDeg = -controlInputs.pitch * 30.0f; // +/-30° (inverted)
    float desiredYawRate = controlInputs.yaw * 180.0f;    // +/-180°/s (rate)

    // Apply calibration reference if IMU calibrated (angles already relative in sensorData)
    // Slew limit roll/pitch setpoints
    float maxDeltaRP = slewRateRP * dt; // deg change allowed this cycle
    float deltaR = desiredRollDeg - prevRollSetpointCmd;
    if (deltaR > maxDeltaRP)
        deltaR = maxDeltaRP;
    if (deltaR < -maxDeltaRP)
        deltaR = -maxDeltaRP;
    prevRollSetpointCmd += deltaR;

    float deltaP = desiredPitchDeg - prevPitchSetpointCmd;
    if (deltaP > maxDeltaRP)
        deltaP = maxDeltaRP;
    if (deltaP < -maxDeltaRP)
        deltaP = -maxDeltaRP;
    prevPitchSetpointCmd += deltaP;

    // Yaw slew limiting (rate command smoothing)
    float maxDeltaYaw = slewRateYaw * dt; // deg/s change allowed
    float deltaY = desiredYawRate - prevYawRateCmd;
    if (deltaY > maxDeltaYaw)
        deltaY = maxDeltaYaw;
    if (deltaY < -maxDeltaYaw)
        deltaY = -maxDeltaYaw;
    prevYawRateCmd += deltaY;

    if (flightState.mode >= MODE_STABILIZE)
    {
        if (!USE_CASCADED_ANGLE_RATE)
        {
            // Direct angle stabilization (existing behavior)
            rollPID.setpoint = prevRollSetpointCmd;
            pitchPID.setpoint = prevPitchSetpointCmd;
        }
        else
        {
            // Outer angle loop will compute desired rates; set setpoints for rate PIDs later
            rollPID.setpoint = prevRollSetpointCmd;
            pitchPID.setpoint = prevPitchSetpointCmd;
        }
        yawPID.setpoint = prevYawRateCmd; // yaw stays a rate setpoint
    }

    // Altitude hold setpoint maintenance handled elsewhere

    // Throttle gate now only gates integral accumulation; P/D remain active so sliders respond at any throttle
    bool aboveGate = controlInputs.throttle > throttleGate;
    rollPID.allowIntegral = aboveGate;
    pitchPID.allowIntegral = aboveGate;
    yawPID.allowIntegral = aboveGate;

    if (flightState.mode >= MODE_STABILIZE)
    {
        if (!USE_CASCADED_ANGLE_RATE)
        {
            // Original direct angle control path
            rollPID.setMode(true);
            pitchPID.setMode(true);
            yawPID.setMode(true);

            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                rollPID.input = sensorData.roll_angle;
                pitchPID.input = sensorData.pitch_angle;
                yawPID.input = sensorData.yaw_rate;
                xSemaphoreGive(sensorMutex);
            }

            rollPID.compute();
            pitchPID.compute();
            yawPID.compute();
        }
        else
        {
            // Cascaded angle->rate: outer angle PIDs produce desired rates; inner PIDs track actual rates
            rollPID.setMode(true);
            pitchPID.setMode(true);
            yawPID.setMode(true); // yaw still direct rate
            rollRatePID.setMode(true);
            pitchRatePID.setMode(true);

            float rollAngle, pitchAngle, rollRate, pitchRate, yawRate;
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                rollAngle = sensorData.roll_angle;
                pitchAngle = sensorData.pitch_angle;
                rollRate = sensorData.roll_rate;
                pitchRate = sensorData.pitch_rate;
                yawRate = sensorData.yaw_rate;
                xSemaphoreGive(sensorMutex);
            }
            else
            {
                // Fallback to last known inputs in controllers
                rollAngle = rollPID.input;
                pitchAngle = pitchPID.input;
                rollRate = 0;
                pitchRate = 0;
                yawRate = 0;
            }

            rollPID.input = rollAngle;
            pitchPID.input = pitchAngle;
            yawPID.input = yawRate;

            // Outer angle compute to get desired rate commands
            double rollDesiredRate = constrainFloat((float)rollPID.compute(), -MAX_ROLL_PITCH_RATE, MAX_ROLL_PITCH_RATE);
            double pitchDesiredRate = constrainFloat((float)pitchPID.compute(), -MAX_ROLL_PITCH_RATE, MAX_ROLL_PITCH_RATE);

            // Inner rate loop setpoints and inputs
            rollRatePID.setpoint = rollDesiredRate;
            pitchRatePID.setpoint = pitchDesiredRate;
            rollRatePID.input = rollRate;
            pitchRatePID.input = pitchRate;

            rollRatePID.allowIntegral = aboveGate;
            pitchRatePID.allowIntegral = aboveGate;
            rollRatePID.compute();
            pitchRatePID.compute();

            // Map inner loop outputs into original roll/pitch PID outputs for mixer
            rollPID.output = rollRatePID.output;
            pitchPID.output = pitchRatePID.output;
            // Yaw direct
            yawPID.compute();
        }

        // Yaw feed-forward always applies
        if (yawFeedForward != 0.0f)
        {
            double ff = yawFeedForward * yawPID.setpoint; // setpoint in deg/s
            yawPID.output += ff;
            if (yawPID.output > yawPID.outputMax)
                yawPID.output = yawPID.outputMax;
            if (yawPID.output < yawPID.outputMin)
                yawPID.output = yawPID.outputMin;
        }
    }
    else
    {
        rollPID.setMode(false);
        pitchPID.setMode(false);
        yawPID.setMode(false);
        rollPID.output = 0.0;
        pitchPID.output = 0.0;
        yawPID.output = 0.0;
    }

    if (flightState.altitude_hold_active)
    {
        altitudePID.setMode(true);
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            altitudePID.input = sensorData.altitude_filtered;
            xSemaphoreGive(sensorMutex);
        }
        altitudePID.compute();
    }
}

void mixMotorOutputs()
{
    // 1. Determine base throttle with dynamic headroom (align with droneFreeRTOS behavior)
    // controlInputs.throttle expected 0..1
    float throttleInputNorm = constrainFloat(controlInputs.throttle, 0.0f, 1.0f);

    float baseMicros;
    if (flightState.mode == MODE_MANUAL)
    {
        // Manual: give full usable range (idle..max) directly
        baseMicros = mapFloat(throttleInputNorm, 0.0f, 1.0f, MOTOR_IDLE, MOTOR_MAX);
    }
    else
    {
        // Option B: NO pre-reserved headroom. Use full user throttle range (idle..max) for base.
        // Differential authority is then dynamically scaled inside applyScaledMotorMix.
        baseMicros = mapFloat(throttleInputNorm, 0.0f, 1.0f, MOTOR_IDLE, MOTOR_MAX);

        // Altitude hold additive correction (only if active) applied on top of full-range base.
        if (flightState.altitude_hold_active)
        {
            const float ALT_CORR_MAX = 300.0f;                          // ±300 µs window
            float centeredAlt = (altitudePID.output - 500.0f) / 500.0f; // altitudePID 0..1000 -> -1..+1
            centeredAlt = constrainFloat(centeredAlt, -1.0f, 1.0f);
            baseMicros += centeredAlt * ALT_CORR_MAX;
        }
        // Clamp base after altitude adjustment
        if (baseMicros < MOTOR_IDLE)
            baseMicros = MOTOR_IDLE;
        if (baseMicros > MOTOR_MAX)
            baseMicros = MOTOR_MAX;
    }

    // 2. Prepare control axes (manual inputs or PID outputs depending on mode)
    float Rn = 0.0f, Pn = 0.0f, Yn = 0.0f;
    if (flightState.mode == MODE_MANUAL)
    {
        // Manual: use stick inputs directly normalized (already -1..1 expected)
        // Roll sign: positive roll input tilts right; left roll increases right-side motors
        Rn = constrainFloat((controlInputs.roll) * manualAxisGain, -1.0f, 1.0f);
        // Invert pitch normalization so positive stick increases back motor thrust (nose-up)
        Pn = -constrainFloat(controlInputs.pitch * manualAxisGain, -1.0f, 1.0f);
        Yn = constrainFloat(controlInputs.yaw * manualAxisGain, -1.0f, 1.0f);
    }
    else
    {
        // Stabilized modes: use PID outputs normalized by gains
        Rn = constrainFloat((float)(rollPID.output / MIX_KR), -1.0f, 1.0f);
        Pn = constrainFloat((float)(pitchPID.output / MIX_KP), -1.0f, 1.0f);
        Yn = constrainFloat((float)(yawPID.output / MIX_KY), -1.0f, 1.0f);
    }

    // 3. Apply unified scaled mixing (handles saturation headroom automatically)
    applyScaledMotorMix(baseMicros, Rn, Pn, Yn);

    // 4. Apply motor calibration offsets AFTER mixing (applies to all modes).
    // Manual mode already applies offsets in the manual path; stabilized uses this path via mixMotorOutputs.
    motorOutputs.motor1 = constrain(motorOutputs.motor1 + (int)motorCal.motor1_offset, MOTOR_MIN, MOTOR_MAX);
    motorOutputs.motor2 = constrain(motorOutputs.motor2 + (int)motorCal.motor2_offset, MOTOR_MIN, MOTOR_MAX);
    motorOutputs.motor3 = constrain(motorOutputs.motor3 + (int)motorCal.motor3_offset, MOTOR_MIN, MOTOR_MAX);
    motorOutputs.motor4 = constrain(motorOutputs.motor4 + (int)motorCal.motor4_offset, MOTOR_MIN, MOTOR_MAX);
}

void outputMotorCommands()
{
    // Output PWM commands to ESCs
    esc1.writeMicroseconds(motorOutputs.motor1);
    esc2.writeMicroseconds(motorOutputs.motor2);
    esc3.writeMicroseconds(motorOutputs.motor3);
    esc4.writeMicroseconds(motorOutputs.motor4);
}

void checkSafetyConditions()
{
    // Check for emergency conditions
    unsigned long now = millis();

    // Initialize battery voltage properly to avoid false alarms
    static bool battery_initialized = false;
    if (!battery_initialized && sensorData.battery_voltage > 9.0)
    {
        battery_initialized = true;
    }

    // Only check command timeout if motors are armed and we've received commands
    // Command timeout temporarily disabled for testing
    // TODO: Re-enable command timeout once web interface sends continuous heartbeat
    // if (flightState.motors_armed && controlInputs.timestamp > 0 && (now - controlInputs.timestamp > 5000))
    // { // 5 second timeout only when armed
    //     flightState.emergency_stop = true;
    //     Serial.println("EMERGENCY: Command timeout!");
    // }

    // Battery safety check disabled - display only
    // TODO: Fix battery voltage reading hardware/software issue
    // if (flightState.motors_armed && battery_initialized && sensorData.battery_voltage > 9.0 && sensorData.battery_percentage < 10.0)
    // {
    //     flightState.emergency_stop = true;
    //     Serial.println("EMERGENCY: Critical battery voltage!");
    // }

    // Check for sensor failures - only if sensors have been initialized and motors are armed
    if (flightState.motors_armed && sensorData.timestamp > 0 && (now - sensorData.timestamp > 500))
    { // 500ms sensor timeout only when armed
        flightState.emergency_stop = true;
        Serial.println("EMERGENCY: Sensor data timeout!");
    }

    // Check for excessive tilt angles - only when armed
    if (flightState.motors_armed && (abs(sensorData.roll_angle) > 60.0 || abs(sensorData.pitch_angle) > 60.0))
    {
        flightState.emergency_stop = true;
        Serial.println("EMERGENCY: Excessive tilt angle!");
    }
}

void handleEmergency()
{
    // Emergency procedure: gradually reduce throttle
    static int emergency_throttle = MOTOR_IDLE;
    static unsigned long last_emergency_update = 0;

    unsigned long now = millis();
    if (now - last_emergency_update > 50)
    { // Update every 50ms
        last_emergency_update = now;

        if (emergency_throttle > MOTOR_ARM_VALUE)
        {
            emergency_throttle -= 5; // Gradually reduce
        }

        // Set all motors to emergency throttle
        motorOutputs.motor1 = emergency_throttle;
        motorOutputs.motor2 = emergency_throttle;
        motorOutputs.motor3 = emergency_throttle;
        motorOutputs.motor4 = emergency_throttle;

        outputMotorCommands();

        // Flash red navigation LEDs during emergency
        static bool led_state = false;
        led_state = !led_state;
        digitalWrite(NAV_BACK_LEFT_RED, led_state);
        digitalWrite(NAV_FRONT_LEFT_RED, led_state);
    }
}

void updateFlightMode()
{
    // Edge-detected arming & mode toggling unified with RF build semantics.
    // arm_switch: momentary or latched; we treat rising edge (false->true) as ARMED, falling as DISARM.
    // mode_switch: while armed, changes flight mode: false = MANUAL, true = STABILIZE.
    static bool lastArmSwitch = false;
    static bool lastModeSwitch = false;

    bool armEdgeRising = (!lastArmSwitch && controlInputs.arm_switch);
    bool armEdgeFalling = (lastArmSwitch && !controlInputs.arm_switch);
    lastArmSwitch = controlInputs.arm_switch;

    // Handle arming on rising edge
    if (armEdgeRising && !flightState.motors_armed)
    {
        bool throttle_ok = (controlInputs.throttle < 0.05f); // require near-zero throttle
        bool level_ok = true;
        if (sensorData.timestamp > 0 && sensorData.valid)
        {
            level_ok = (fabs(sensorData.roll_angle) < 30.0f && fabs(sensorData.pitch_angle) < 30.0f);
        }
        if (throttle_ok && level_ok)
        {
            flightState.motors_armed = true;
            flightState.arm_time = millis();
            flightState.emergency_stop = false;
            // Determine initial mode from current mode_switch state
            flightState.mode = controlInputs.mode_switch ? MODE_STABILIZE : MODE_MANUAL;
            lastModeSwitch = controlInputs.mode_switch; // seed edge detector
            Serial.printf("🔓 ARMED - %s MODE\n", flightState.mode == MODE_STABILIZE ? "STABILIZE" : "MANUAL");
            // Simple feedback (non-blocking style minimal delays preserved)
            digitalWrite(BUZZER_PIN, HIGH);
            delay(120);
            digitalWrite(BUZZER_PIN, LOW);
        }
        else
        {
            Serial.printf("ARM BLOCKED (thr %.2f, level %s)\n", controlInputs.throttle, level_ok ? "OK" : "BAD");
        }
    }
    // Disarm on falling edge
    if (armEdgeFalling && flightState.motors_armed)
    {
        flightState.motors_armed = false;
        flightState.mode = MODE_DISARMED;
        flightState.altitude_hold_active = false;
        Serial.println("🔒 DISARMED");
        digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
        digitalWrite(NAV_BACK_CENTER_WHITE, LOW);
    }

    // If not armed, nothing further
    if (!flightState.motors_armed)
        return;

    // Mode switch edge detection (supports in-flight switching)
    bool modeState = controlInputs.mode_switch;
    if (modeState != lastModeSwitch)
    {
        flightState.mode = modeState ? MODE_STABILIZE : MODE_MANUAL;
        Serial.printf("🛩 Flight Mode -> %s\n", flightState.mode == MODE_STABILIZE ? "STABILIZE" : "MANUAL");
        // Reset PIDs when enabling stabilize for clean integrators
        if (flightState.mode == MODE_STABILIZE)
        {
            rollPID.reset();
            pitchPID.reset();
            yawPID.reset();
        }
        lastModeSwitch = modeState;
    }

    // (Altitude hold activation remains disabled / placeholder to avoid unintentional engagement.)
}

// ================================
// UTILITY FUNCTIONS
// ================================

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrainFloat(float value, float min_val, float max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

void calibrateIMU()
{
    Serial.println("===== IMU CALIBRATION STARTING =====");
    Serial.println("Place drone on FLAT GROUND and keep STATIONARY during calibration!");
    Serial.println("This calibration compensates for IMU mounting misalignment.");
    Serial.println("Calibration will take 15 seconds...");

    // Visual indication of calibration - blink all LEDs
    for (int i = 0; i < 5; i++)
    {
        digitalWrite(LED_R1_PIN, HIGH);
        digitalWrite(LED_G1_PIN, HIGH);
        digitalWrite(LED_B1_PIN, HIGH);
        digitalWrite(NAV_FRONT_RIGHT_GREEN, LOW);
        digitalWrite(NAV_BACK_RIGHT_GREEN, LOW);
        digitalWrite(NAV_BACK_LEFT_RED, LOW);
        digitalWrite(NAV_FRONT_LEFT_RED, LOW);
        delay(200);

        digitalWrite(LED_R1_PIN, LOW);
        digitalWrite(LED_G1_PIN, LOW);
        digitalWrite(LED_B1_PIN, LOW);
        digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
        digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
        digitalWrite(NAV_BACK_LEFT_RED, HIGH);
        digitalWrite(NAV_FRONT_LEFT_RED, HIGH);
        delay(200);
    }

    // Calibration beep sequence
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }

    delay(2000); // Wait 2 seconds before starting calibration

    // Reset calibration values
    imuCal.roll_offset = 0.0;
    imuCal.pitch_offset = 0.0;
    imuCal.yaw_offset = 0.0;
    imuCal.accel_x_offset = 0.0;
    imuCal.accel_y_offset = 0.0;
    imuCal.accel_z_offset = 0.0;
    imuCal.gyro_x_offset = 0.0;
    imuCal.gyro_y_offset = 0.0;
    imuCal.gyro_z_offset = 0.0;

    // Extended calibration for better accuracy with longer samples
    const int numSamples = 1500; // 15 seconds at ~100Hz for higher precision
    float accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    int validSamples = 0;

    Serial.println("Collecting extended calibration samples for precise mounting offset compensation...");

    for (int i = 0; i < numSamples; i++)
    {
        sensors_event_t accel, gyro, temp;
        if (mpu.getEvent(&accel, &gyro, &temp))
        {
            accel_x_sum += accel.acceleration.x;
            accel_y_sum += accel.acceleration.y;
            accel_z_sum += accel.acceleration.z;
            gyro_x_sum += gyro.gyro.x;
            gyro_y_sum += gyro.gyro.y;
            gyro_z_sum += gyro.gyro.z;
            validSamples++;
        }

        // Progress indication
        if (i % 150 == 0)
        {
            Serial.printf("Calibration progress: %d%%\n", (i * 100) / numSamples);
            // Brief beep for progress
            digitalWrite(BUZZER_PIN, HIGH);
            delay(30);
            digitalWrite(BUZZER_PIN, LOW);
        }

        delay(10); // ~100Hz sampling rate
    }

    if (validSamples > 750) // Need at least 750 valid samples for precision
    {
        // Calculate sensor offsets
        imuCal.accel_x_offset = accel_x_sum / validSamples;
        imuCal.accel_y_offset = accel_y_sum / validSamples;
        imuCal.accel_z_offset = (accel_z_sum / validSamples) - 9.81; // Remove gravity from Z-axis
        imuCal.gyro_x_offset = gyro_x_sum / validSamples;
        imuCal.gyro_y_offset = gyro_y_sum / validSamples;
        imuCal.gyro_z_offset = gyro_z_sum / validSamples;

        // Calculate the current "level" angles from flat ground position
        // These represent the IMU mounting misalignment relative to the drone frame
        float raw_roll = atan2(imuCal.accel_y_offset, sqrt(imuCal.accel_x_offset * imuCal.accel_x_offset + (imuCal.accel_z_offset + 9.81) * (imuCal.accel_z_offset + 9.81))) * 180.0 / PI;
        float raw_pitch = atan2(-imuCal.accel_x_offset, sqrt(imuCal.accel_y_offset * imuCal.accel_y_offset + (imuCal.accel_z_offset + 9.81) * (imuCal.accel_z_offset + 9.81))) * 180.0 / PI;

        // Store these as the offsets to subtract from future angle calculations
        // This makes the current flat ground position read as 0°, 0°
        imuCal.roll_offset = raw_roll;
        imuCal.pitch_offset = raw_pitch;
        imuCal.yaw_offset = 0.0; // Yaw reference is relative

        imuCal.calibrated = true;

        Serial.println("===== IMU CALIBRATION COMPLETED =====");
        Serial.printf("Accelerometer offsets: X=%.4f, Y=%.4f, Z=%.4f m/s²\n",
                      imuCal.accel_x_offset, imuCal.accel_y_offset, imuCal.accel_z_offset);
        Serial.printf("Gyroscope offsets: X=%.4f, Y=%.4f, Z=%.4f rad/s\n",
                      imuCal.gyro_x_offset, imuCal.gyro_y_offset, imuCal.gyro_z_offset);
        Serial.printf("IMU mounting angle offsets: Roll=%.3f°, Pitch=%.3f°\n",
                      imuCal.roll_offset, imuCal.pitch_offset);
        Serial.println("Current flat ground position will now read as 0°, 0° (perfect level).");
        Serial.println("PID setpoints for stabilize mode will be relative to this calibrated level position.");

        // Success beep sequence
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(200);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }

        // Green LED on to indicate successful calibration
        digitalWrite(LED_G1_PIN, HIGH);
        delay(2000);
        digitalWrite(LED_G1_PIN, LOW);
    }
    else
    {
        Serial.println("===== IMU CALIBRATION FAILED =====");
        Serial.printf("Not enough valid samples: %d/%d\n", validSamples, numSamples);
        Serial.println("Check IMU connection and try again.");
        Serial.println("Ensure drone is completely stationary during calibration.");

        // Error beep sequence
        for (int i = 0; i < 5; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(100);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }

        // Red LED on to indicate failed calibration
        digitalWrite(LED_R1_PIN, HIGH);
        delay(3000);
        digitalWrite(LED_R1_PIN, LOW);
    }

    Serial.println("=====================================");
}

// ================================
// WEB SERVER SETUP
// ================================

void setupWebServer()
{
    // Serve main dashboard
    server.on("/", HTTP_GET, []()
              {
        String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Drone Flight Controller</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; padding: 20px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .status { display: flex; flex-wrap: wrap; gap: 10px; }
        .status-item { padding: 10px; border-radius: 4px; color: white; min-width: 100px; text-align: center; }
        .status-armed { background: #4CAF50; }
        .status-disarmed { background: #f44336; }
        .status-manual { background: #ff9800; }
        .status-stabilize { background: #2196F3; }
        .controls { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .joystick { text-align: center; padding: 20px; }
        button { padding: 10px 20px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; }
        .btn-primary { background: #2196F3; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn-success { background: #4CAF50; color: white; }
        input[type="range"] { width: 100%; }
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
        .sensor-value { padding: 10px; background: #e3f2fd; border-radius: 4px; text-align: center; }
        .emergency { background: #f44336; color: white; font-weight: bold; padding: 15px; border-radius: 8px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Drone Flight Controller</h1>
        
        <!-- Flight Status -->
        <div class="card">
            <h2>Flight Status</h2>
            <div class="status" id="flightStatus">
                <div class="status-item status-disarmed">DISARMED</div>
                <div class="status-item">Manual Mode</div>
            </div>
            <div id="emergencyAlert" style="display: none;" class="emergency">
                WARNING: EMERGENCY MODE ACTIVE - LANDING AUTOMATICALLY
            </div>
        </div>
        
        <!-- Control Interface -->
        <div class="card">
            <h2>Flight Controls</h2>
            <div class="controls">
                <div class="joystick">
                    <h3>Throttle</h3>
                    <input type="range" id="throttle" min="0" max="100" value="0" orient="vertical" oninput="updateControl()">
                    <span id="throttle_val">0%</span>
                </div>
                <div class="joystick">
                    <h3>Roll (Left/Right)</h3>
                    <input type="range" id="roll" min="-100" max="100" value="0" oninput="updateControl()">
                    <span id="roll_val">0</span>
                </div>
                <div class="joystick">
                    <h3>Pitch (Forward/Back)</h3>
                    <input type="range" id="pitch" min="-100" max="100" value="0" oninput="updateControl()">
                    <span id="pitch_val">0</span>
                </div>
                <div class="joystick">
                    <h3>Yaw (Rotate)</h3>
                    <input type="range" id="yaw" min="-100" max="100" value="0" oninput="updateControl()">
                    <span id="yaw_val">0</span>
                </div>
            </div>
            <div style="text-align: center; margin-top: 20px;">
                <button class="btn-success" onclick="armMotors()">ARM MOTORS</button>
                <button class="btn-danger" onclick="disarmMotors()">DISARM MOTORS</button>
                <button class="btn-primary" onclick="toggleFlightMode()">TOGGLE MODE</button>
                <button class="btn-danger" onclick="emergencyStop()">EMERGENCY STOP</button>
            </div>
        </div>
        
        <!-- Sensor Data -->
        <div class="card">
            <h2>Live Sensor Data</h2>
            <div class="sensor-grid" id="sensorData">
                <div class="sensor-value">Roll: 0.0 deg</div>
                <div class="sensor-value">Pitch: 0.0 deg</div>
                <div class="sensor-value">Altitude: 0.0 m</div>
                <div class="sensor-value">Battery: 0.0V</div>
            </div>
        </div>
        
        <!-- Motor Calibration -->
        <div class="card">
            <h2>Motor Calibration (Fix Imbalance)</h2>
            <p style="color: #666; font-size: 14px;">Adjust motor offsets to compensate for mechanical imbalances. Negative values reduce motor power.</p>
            <div class="controls">
                <div>
                    <h3>Motor 1 (Front Right CCW)</h3>
                    <label>Offset: <input type="number" id="motor1_offset" value="0" step="1" min="-200" max="200" onchange="updateMotorCal()"></label>
                </div>
                <div>
                    <h3>Motor 2 (Back Right CW)</h3>
                    <label>Offset: <input type="number" id="motor2_offset" value="0" step="1" min="-200" max="200" onchange="updateMotorCal()"></label>
                </div>
                <div>
                    <h3>Motor 3 (Front Left CW)</h3>
                    <label>Offset: <input type="number" id="motor3_offset" value="0" step="1" min="-200" max="200" onchange="updateMotorCal()"></label>
                </div>
                <div>
                    <h3>Motor 4 (Back Left CCW)</h3>
                    <label>Offset: <input type="number" id="motor4_offset" value="0" step="1" min="-200" max="200" onchange="updateMotorCal()"></label>
                </div>
            </div>
            <div style="text-align: center; margin-top: 15px;">
                <button class="btn-primary" onclick="resetMotorCal()">Reset All to 0</button>
                <button class="btn-success" onclick="saveMotorCal()">Save Calibration</button>
            </div>
        </div>
        
        <!-- PID Tuning -->
        <div class="card">
            <h2>PID Tuning</h2>
            <div class="controls">
                <div>
                    <h3>Roll PID</h3>
                    <label>P: <input type="number" id="roll_p" value="2.0" step="0.1" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="roll_i" value="0.0" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="roll_d" value="0.1" step="0.01" onchange="updatePID()"></label><br>
                </div>
                <div>
                    <h3>Pitch PID</h3>
                    <label>P: <input type="number" id="pitch_p" value="2.0" step="0.1" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="pitch_i" value="0.0" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="pitch_d" value="0.1" step="0.01" onchange="updatePID()"></label><br>
                </div>
                <div>
                    <h3>Yaw PID</h3>
                    <label>P: <input type="number" id="yaw_p" value="1.0" step="0.1" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="yaw_i" value="0.0" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="yaw_d" value="0.05" step="0.01" onchange="updatePID()"></label><br>
                </div>
                <div>
                    <h3>Altitude PID</h3>
                    <label>P: <input type="number" id="alt_p" value="1.50" step="0.05" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="alt_i" value="0.20" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="alt_d" value="0.80" step="0.05" onchange="updatePID()"></label><br>
                </div>
            </div>
                        <hr>
                        <h3>Mixer / Control Gains</h3>
                        <div class="controls">
                            <div>
                                <label>Mix KR (Roll): <input type="number" id="mix_kr" value="200" step="5" onchange="updatePID()"></label><br>
                                <label>Mix KP (Pitch): <input type="number" id="mix_kp" value="200" step="5" onchange="updatePID()"></label><br>
                                <label>Mix KY (Yaw): <input type="number" id="mix_ky" value="150" step="5" onchange="updatePID()"></label><br>
                            </div>
                            <div>
                                <label>Manual Axis Gain: <input type="number" id="manual_axis_gain" value="1.30" step="0.05" onchange="updatePID()"></label><br>
                                <label>Throttle Gate: <input type="number" id="throttle_gate" value="0.35" step="0.01" min="0" max="1" onchange="updatePID()"></label><br>
                                <label>Slew Roll/Pitch (deg/s): <input type="number" id="slew_rp" value="120" step="5" onchange="updatePID()"></label><br>
                                <label>Slew Yaw Rate (deg/s/s): <input type="number" id="slew_yaw" value="720" step="10" onchange="updatePID()"></label><br>
                                <label>Yaw Feed-Forward: <input type="number" id="yaw_ff" value="0.10" step="0.01" onchange="updatePID()"></label><br>
                                <label>Hover Throttle (µs): <input type="number" id="hover_thr" value="1550" step="5" onchange="updatePID()"></label><br>
                                <label>Integral Leak (0-0.05): <input type="number" id="i_leak" value="0.001" step="0.001" min="0" max="0.05" onchange="updatePID()"></label><br>
                            </div>
                        </div>
                        <hr>
                        <h3>Advanced: Fusion & Cascaded Control</h3>
                        <div class="controls">
                            <div>
                                <label><input type="checkbox" id="use_kalman"> Use Kalman for Roll/Pitch</label><br>
                                <label>Kalman Rate Std (deg/s): <input type="number" id="kalman_rate_std" value="4.0" step="0.5" onchange="updatePID()"></label><br>
                                <label>Kalman Meas Std (deg): <input type="number" id="kalman_meas_std" value="3.0" step="0.5" onchange="updatePID()"></label><br>
                            </div>
                            <div>
                                <label><input type="checkbox" id="use_cascaded"> Use Cascaded Angle→Rate</label><br>
                                <label>Max Roll/Pitch Rate (deg/s): <input type="number" id="max_rp_rate" value="180" step="10" onchange="updatePID()"></label><br>
                                <label>Rate Roll P: <input type="number" id="rate_roll_p" value="0.10" step="0.01" onchange="updatePID()"></label>
                                <label>I: <input type="number" id="rate_roll_i" value="0.02" step="0.01" onchange="updatePID()"></label>
                                <label>D: <input type="number" id="rate_roll_d" value="0.002" step="0.001" onchange="updatePID()"></label><br>
                                <label>Rate Pitch P: <input type="number" id="rate_pitch_p" value="0.10" step="0.01" onchange="updatePID()"></label>
                                <label>I: <input type="number" id="rate_pitch_i" value="0.02" step="0.01" onchange="updatePID()"></label>
                                <label>D: <input type="number" id="rate_pitch_d" value="0.002" step="0.001" onchange="updatePID()"></label>
                            </div>
                        </div>
        </div>
        
        <!-- Flight Controller Debug Console -->
        <div class="card">
            <h2>Flight Controller Debug Console</h2>
            <div style="background: #1e1e1e; color: #00ff00; padding: 15px; border-radius: 4px; font-family: 'Courier New', monospace; font-size: 14px; max-height: 600px; overflow-y: auto;" id="debugConsole">
                <div style="color: #ffffff;">======== FLIGHT CONTROLLER DEBUG ========</div>
                <div id="debugContent">
                    <div style="color: #ffff00;">[Status]</div>
                    <div>Mode: DISARMED | Armed: NO | Throttle: 0.00 (PWM: 1100)</div>
                    <div>Roll: 0.0° | Pitch: 0.0° | Yaw: 0.0°/s | Alt: 0.0m | Batt: 0.0V</div>
                    <br>
                    <div style="color: #ffff00;">[System]</div>
                    <div>Loop Freq: 100Hz | Sensors: OK | Emergency: NO</div>
                    <div>Uptime: 0 ms | Free Heap: 0 bytes</div>
                </div>
                <div style="color: #ffffff; margin-top: 10px;">==========================================</div>
            </div>
            <div style="text-align: center; margin-top: 10px;">
                <button class="btn-primary" onclick="toggleDebugConsole()">Toggle Auto-Update</button>
                <button class="btn-primary" onclick="clearDebugConsole()">Clear Console</button>
            </div>
        </div>
    </div>

    <script>
        let controlUpdateTimeout = null;
        let armed = false;
        let flightMode = 'MANUAL';
        
        // Update flight data every 100ms
        setInterval(updateFlightData, 100);
        
        // Send heartbeat control data every 500ms to prevent timeout
        setInterval(sendHeartbeat, 500);
        
        function sendHeartbeat() {
            // Only send heartbeat if armed
            if (armed) {
                const data = {
                    throttle: parseFloat(document.getElementById('throttle').value) / 100.0,
                    roll: parseFloat(document.getElementById('roll').value) / 100.0,
                    pitch: parseFloat(document.getElementById('pitch').value) / 100.0,
                    yaw: parseFloat(document.getElementById('yaw').value) / 100.0
                };
                
                fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
            }
        }
        
        function updateFlightData() {
            fetch('/api/flight_data')
                .then(response => response.json())
                .then(data => {
                    updateFlightStatus(data.status);
                    updateSensorDisplay(data.sensors);
                })
                .catch(error => console.log('Data update error:', error));
        }
        
        function updateFlightStatus(status) {
            const container = document.getElementById('flightStatus');
            const emergency = document.getElementById('emergencyAlert');
            
            armed = status.armed;
            flightMode = status.mode;
            
            let statusClass = status.armed ? 'status-armed' : 'status-disarmed';
            let modeClass = status.mode === 'STABILIZE' ? 'status-stabilize' : 'status-manual';
            
            container.innerHTML = `
                <div class="status-item ${statusClass}">${status.armed ? '[ARMED]' : '[DISARMED]'}</div>
                <div class="status-item ${modeClass}">${status.mode} Mode</div>
                <div class="status-item">Alt Hold: ${status.altitude_hold ? 'ON' : 'OFF'}</div>
            `;
            
            if (status.emergency) {
                emergency.style.display = 'block';
            } else {
                emergency.style.display = 'none';
            }
        }
        
        function updateSensorDisplay(sensors) {
            const container = document.getElementById('sensorData');
            container.innerHTML = `
                <div class="sensor-value">Roll: ${sensors.roll.toFixed(1)} deg</div>
                <div class="sensor-value">Pitch: ${sensors.pitch.toFixed(1)} deg</div>
                <div class="sensor-value">Yaw Rate: ${sensors.yaw_rate.toFixed(1)} deg/s</div>
                <div class="sensor-value">Altitude: ${sensors.altitude.toFixed(2)} m</div>
                <div class="sensor-value">Battery: ${sensors.battery_voltage.toFixed(2)}V (${sensors.battery_percentage.toFixed(1)}%)</div>
                <div class="sensor-value">GPS: ${sensors.satellites} sats</div>
                <div class="sensor-value">Motor 1: ${sensors.motor1}</div>
                <div class="sensor-value">Motor 2: ${sensors.motor2}</div>
                <div class="sensor-value">Motor 3: ${sensors.motor3}</div>
                <div class="sensor-value">Motor 4: ${sensors.motor4}</div>
            `;
        }
        
        function updateControl() {
            document.getElementById('throttle_val').textContent = document.getElementById('throttle').value + '%';
            document.getElementById('roll_val').textContent = document.getElementById('roll').value;
            document.getElementById('pitch_val').textContent = document.getElementById('pitch').value;
            document.getElementById('yaw_val').textContent = document.getElementById('yaw').value;
            
            // Throttle control input updates
            if (controlUpdateTimeout) clearTimeout(controlUpdateTimeout);
            controlUpdateTimeout = setTimeout(() => {
                const data = {
                    throttle: parseFloat(document.getElementById('throttle').value) / 100.0,
                    roll: parseFloat(document.getElementById('roll').value) / 100.0,
                    pitch: parseFloat(document.getElementById('pitch').value) / 100.0,
                    yaw: parseFloat(document.getElementById('yaw').value) / 100.0
                };
                
                fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
            }, 50); // 20Hz update rate
        }
        
        function armMotors() {
            // Safety: Set throttle to 0 before arming
            document.getElementById('throttle').value = 0;
            updateControl();
            
            fetch('/api/arm', {method: 'POST'})
                .then(() => console.log('Arm command sent - throttle set to 0 for safety'));
        }
        
        function disarmMotors() {
            fetch('/api/disarm', {method: 'POST'})
                .then(() => console.log('Disarm command sent'));
        }
        
        function toggleFlightMode() {
            fetch('/api/toggle_mode', {method: 'POST'})
                .then(() => console.log('Mode toggle sent'));
        }
        
        function emergencyStop() {
            if (confirm('EMERGENCY STOP - Are you sure?')) {
                fetch('/api/emergency', {method: 'POST'})
                    .then(() => console.log('Emergency stop activated'));
            }
        }
        
        function updatePID() {
            const pidData = {
                roll_p: parseFloat(document.getElementById('roll_p').value),
                roll_i: parseFloat(document.getElementById('roll_i').value),
                roll_d: parseFloat(document.getElementById('roll_d').value),
                pitch_p: parseFloat(document.getElementById('pitch_p').value),
                pitch_i: parseFloat(document.getElementById('pitch_i').value),
                pitch_d: parseFloat(document.getElementById('pitch_d').value),
                yaw_p: parseFloat(document.getElementById('yaw_p').value),
                yaw_i: parseFloat(document.getElementById('yaw_i').value),
                yaw_d: parseFloat(document.getElementById('yaw_d').value),
                alt_p: parseFloat(document.getElementById('alt_p').value),
                alt_i: parseFloat(document.getElementById('alt_i').value),
                alt_d: parseFloat(document.getElementById('alt_d').value),
                mix_kr: parseFloat(document.getElementById('mix_kr').value),
                mix_kp: parseFloat(document.getElementById('mix_kp').value),
                mix_ky: parseFloat(document.getElementById('mix_ky').value),
                manual_axis_gain: parseFloat(document.getElementById('manual_axis_gain').value),
                throttle_gate: parseFloat(document.getElementById('throttle_gate').value),
                slew_rp: parseFloat(document.getElementById('slew_rp').value),
                slew_yaw: parseFloat(document.getElementById('slew_yaw').value),
                yaw_ff: parseFloat(document.getElementById('yaw_ff').value),
                hover_thr: parseFloat(document.getElementById('hover_thr').value),
                i_leak: parseFloat(document.getElementById('i_leak').value),
                use_kalman: document.getElementById('use_kalman').checked,
                kalman_rate_std: parseFloat(document.getElementById('kalman_rate_std').value),
                kalman_meas_std: parseFloat(document.getElementById('kalman_meas_std').value),
                use_cascaded: document.getElementById('use_cascaded').checked,
                max_rp_rate: parseFloat(document.getElementById('max_rp_rate').value),
                rate_roll_p: parseFloat(document.getElementById('rate_roll_p').value),
                rate_roll_i: parseFloat(document.getElementById('rate_roll_i').value),
                rate_roll_d: parseFloat(document.getElementById('rate_roll_d').value),
                rate_pitch_p: parseFloat(document.getElementById('rate_pitch_p').value),
                rate_pitch_i: parseFloat(document.getElementById('rate_pitch_i').value),
                rate_pitch_d: parseFloat(document.getElementById('rate_pitch_d').value)
            };
            
            fetch('/api/pid_update', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(pidData)
            });
        }
        
        function updateMotorCal() {
            const motorCalData = {
                motor1_offset: parseFloat(document.getElementById('motor1_offset').value),
                motor2_offset: parseFloat(document.getElementById('motor2_offset').value),
                motor3_offset: parseFloat(document.getElementById('motor3_offset').value),
                motor4_offset: parseFloat(document.getElementById('motor4_offset').value)
            };
            
            fetch('/api/motor_calibration', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(motorCalData)
            }).then(() => console.log('Motor calibration updated'));
        }
        
        function resetMotorCal() {
            document.getElementById('motor1_offset').value = 0;
            document.getElementById('motor2_offset').value = 0;
            document.getElementById('motor3_offset').value = 0;
            document.getElementById('motor4_offset').value = 0;
            updateMotorCal();
        }
        
        function saveMotorCal() {
            // Save current calibration values (could be expanded to EEPROM storage)
            updateMotorCal();
            alert('Motor calibration saved!');
        }
        
        // Debug Console Functionality
        let debugUpdateEnabled = true;
        let debugUpdateInterval = null;
        
        // Start debug updates automatically
        function startDebugUpdates() {
            if (debugUpdateInterval) clearInterval(debugUpdateInterval);
            debugUpdateInterval = setInterval(updateDebugConsole, 1000); // Update every 1 second
        }
        
        function updateDebugConsole() {
            if (!debugUpdateEnabled) return;
            
            fetch('/api/debug')
                .then(response => response.json())
                .then(data => {
                    const content = document.getElementById('debugContent');
                    let html = '';
                    
                    // [Status] Section
                    html += '<div style="color: #ffff00;">[Status]</div>';
                    html += `<div>Mode: ${data.status.mode} | Armed: ${data.status.armed ? 'YES' : 'NO'} | Throttle: ${data.status.throttle.toFixed(2)} (PWM: ${data.status.throttle_pwm})</div>`;
                    html += `<div>Roll: ${data.status.roll.toFixed(1)}° | Pitch: ${data.status.pitch.toFixed(1)}° | Yaw: ${data.status.yaw_rate.toFixed(1)}°/s | Alt: ${data.status.altitude.toFixed(1)}m | Batt: ${data.status.battery.toFixed(1)}V</div>`;
                    html += '<br>';
                    
                    // [PID Debug] Section - Enhanced with custom PID diagnostics
                    if (data.pid.active) {
                        html += '<div style="color: #ffff00;">[PID Debug - Custom Controller]</div>';
                        html += `<div>Output -> Roll: ${data.pid.roll_output >= 0 ? '+' : ''}${data.pid.roll_output.toFixed(1)} | Pitch: ${data.pid.pitch_output >= 0 ? '+' : ''}${data.pid.pitch_output.toFixed(1)} | Yaw: ${data.pid.yaw_output >= 0 ? '+' : ''}${data.pid.yaw_output.toFixed(1)}</div>`;
                        html += `<div>Setpoints -> Roll: ${data.pid.roll_setpoint >= 0 ? '+' : ''}${data.pid.roll_setpoint.toFixed(1)}° | Pitch: ${data.pid.pitch_setpoint >= 0 ? '+' : ''}${data.pid.pitch_setpoint.toFixed(1)}° | Yaw: ${data.pid.yaw_setpoint >= 0 ? '+' : ''}${data.pid.yaw_setpoint.toFixed(1)}°/s</div>`;
                        
                        // Roll PID components
                        html += `<div style="color: #00ffff;">Roll P: ${data.pid.roll_p >= 0 ? '+' : ''}${data.pid.roll_p.toFixed(1)} | I: ${data.pid.roll_i >= 0 ? '+' : ''}${data.pid.roll_i.toFixed(1)} | D: ${data.pid.roll_d >= 0 ? '+' : ''}${data.pid.roll_d.toFixed(1)}`;
                        if (data.pid.roll_sat) html += ' | <span style="color: #ff6600;">OUT-SAT</span>';
                        if (data.pid.roll_i_sat) html += ' | <span style="color: #ff6600;">I-SAT</span>';
                        html += '</div>';
                        
                        // Pitch PID components
                        html += `<div style="color: #00ffff;">Pitch P: ${data.pid.pitch_p >= 0 ? '+' : ''}${data.pid.pitch_p.toFixed(1)} | I: ${data.pid.pitch_i >= 0 ? '+' : ''}${data.pid.pitch_i.toFixed(1)} | D: ${data.pid.pitch_d >= 0 ? '+' : ''}${data.pid.pitch_d.toFixed(1)}`;
                        if (data.pid.pitch_sat) html += ' | <span style="color: #ff6600;">OUT-SAT</span>';
                        if (data.pid.pitch_i_sat) html += ' | <span style="color: #ff6600;">I-SAT</span>';
                        html += '</div>';
                        
                        // Yaw PID components
                        html += `<div style="color: #00ffff;">Yaw P: ${data.pid.yaw_p >= 0 ? '+' : ''}${data.pid.yaw_p.toFixed(1)} | I: ${data.pid.yaw_i >= 0 ? '+' : ''}${data.pid.yaw_i.toFixed(1)} | D: ${data.pid.yaw_d >= 0 ? '+' : ''}${data.pid.yaw_d.toFixed(1)}`;
                        if (data.pid.yaw_sat) html += ' | <span style="color: #ff6600;">OUT-SAT</span>';
                        if (data.pid.yaw_i_sat) html += ' | <span style="color: #ff6600;">I-SAT</span>';
                        html += '</div>';
                        
                        html += `<div>PID Status: ${data.pid.enabled_by_throttle ? '<span style="color: #00ff00;">ACTIVE</span>' : '<span style="color: #ff6600;">THROTTLE DEADBAND</span>'} (Min throttle: 30% for control)</div>`;
                        html += '<div style="color: #00ff00;">Features: Anti-windup, Derivative Filtering, Setpoint Smoothing</div>';
                        html += '<br>';
                    }
                    
                    // [Calibration] Section - show calibration offsets
                    if (data.calibration && data.calibration.active) {
                        html += '<div style="color: #ffff00;">[Calibration]</div>';
                        html += `<div>IMU Level Offsets -> Roll: ${data.calibration.roll_offset >= 0 ? '+' : ''}${data.calibration.roll_offset.toFixed(2)}° | Pitch: ${data.calibration.pitch_offset >= 0 ? '+' : ''}${data.calibration.pitch_offset.toFixed(2)}°</div>`;
                        html += '<div>Current angles are relative to these calibrated level positions</div>';
                        html += '<br>';
                    }
                    
                    // [Motor Mix] Section
                    html += '<div style="color: #ffff00;">[Motor Mix]</div>';
                    html += `<div>BasePWM: ${data.motors.base_pwm} -> Motors: [${data.motors.motor1}, ${data.motors.motor2}, ${data.motors.motor3}, ${data.motors.motor4}]</div>`;
                    html += `<div>Calibration: [${data.motors.cal1 >= 0 ? '+' : ''}${data.motors.cal1.toFixed(0)}, ${data.motors.cal2 >= 0 ? '+' : ''}${data.motors.cal2.toFixed(0)}, ${data.motors.cal3 >= 0 ? '+' : ''}${data.motors.cal3.toFixed(0)}, ${data.motors.cal4 >= 0 ? '+' : ''}${data.motors.cal4.toFixed(0)}]</div>`;
                    html += '<br>';
                    
                    // [Inputs] Section
                    html += '<div style="color: #ffff00;">[Inputs]</div>';
                    html += `<div>Throttle: ${data.inputs.throttle.toFixed(3)} | Roll: ${data.inputs.roll >= 0 ? '+' : ''}${data.inputs.roll.toFixed(2)} | Pitch: ${data.inputs.pitch >= 0 ? '+' : ''}${data.inputs.pitch.toFixed(2)} | Yaw: ${data.inputs.yaw >= 0 ? '+' : ''}${data.inputs.yaw.toFixed(2)}</div>`;
                    html += '<br>';
                    
                    // [System] Section
                    html += '<div style="color: #ffff00;">[System]</div>';
                    html += `<div>Loop Freq: 100Hz | Sensors: ${data.system.sensors_ok ? 'OK' : 'FAIL'} | Emergency: ${data.system.emergency ? 'YES' : 'NO'}</div>`;
                    html += `<div>IMU Calibrated: ${data.system.imu_calibrated ? 'YES' : 'NO'} | Uptime: ${data.system.uptime} ms | Free Heap: ${data.system.free_heap} bytes</div>`;
                    
                    content.innerHTML = html;
                    
                    // Auto-scroll to bottom if needed
                    const console = document.getElementById('debugConsole');
                    console.scrollTop = console.scrollHeight;
                })
                .catch(error => {
                    console.log('Debug update error:', error);
                    const content = document.getElementById('debugContent');
                    content.innerHTML = '<div style="color: #ff0000;">Debug data unavailable - Check connection</div>';
                });
        }
        
        function toggleDebugConsole() {
            debugUpdateEnabled = !debugUpdateEnabled;
            const button = event.target;
            if (debugUpdateEnabled) {
                button.textContent = 'Pause Updates';
                button.className = 'btn-danger';
                startDebugUpdates();
            } else {
                button.textContent = 'Resume Updates';
                button.className = 'btn-success';
                if (debugUpdateInterval) {
                    clearInterval(debugUpdateInterval);
                    debugUpdateInterval = null;
                }
            }
        }
        
        function clearDebugConsole() {
            const content = document.getElementById('debugContent');
            content.innerHTML = '<div style="color: #00ff00;">Debug console cleared - waiting for new data...</div>';
        }
        
        // Start debug updates when page loads
        startDebugUpdates();
    </script>
</body>
</html>
)rawliteral";
        server.send(200, "text/html", html); });

    // API: Flight data endpoint
    server.on("/api/flight_data", HTTP_GET, []()
              {
        StaticJsonDocument<1024> doc;
        
        // Flight status
        JsonObject status = doc.createNestedObject("status");
        status["armed"] = flightState.motors_armed;
        status["mode"] = (flightState.mode == MODE_MANUAL) ? "MANUAL" : 
                       (flightState.mode == MODE_STABILIZE) ? "STABILIZE" : "DISARMED";
        status["altitude_hold"] = flightState.altitude_hold_active;
        status["emergency"] = flightState.emergency_stop;
        
        // Sensor data
        JsonObject sensors = doc.createNestedObject("sensors");
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            sensors["roll"] = sensorData.roll_angle;
            sensors["pitch"] = sensorData.pitch_angle;
            sensors["yaw_rate"] = sensorData.yaw_rate;
            sensors["altitude"] = sensorData.altitude_filtered;
            sensors["battery_voltage"] = sensorData.battery_voltage;
            sensors["battery_percentage"] = sensorData.battery_percentage;
            sensors["satellites"] = sensorData.satellites;
            xSemaphoreGive(sensorMutex);
        }
        
        // Motor outputs
        sensors["motor1"] = motorOutputs.motor1;
        sensors["motor2"] = motorOutputs.motor2;
        sensors["motor3"] = motorOutputs.motor3;
        sensors["motor4"] = motorOutputs.motor4;
        
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response); });

    // API: Control input endpoint
    server.on("/api/control", HTTP_POST, []()
              {
        if (!server.hasArg("plain")) {
            server.send(400, "text/plain", "No data");
            return;
        }
        StaticJsonDocument<256> doc;
        DeserializationError err = deserializeJson(doc, server.arg("plain"));
        if (err) {
            server.send(400, "text/plain", "Bad JSON");
            return;
        }
        auto getNorm = [](JsonVariant v, float defVal, float minV, float maxV){
            if (v.isNull()) return defVal;
            float f = v.as<float>();
            if (isnan(f)) return defVal;
            if (f < minV) f = minV; else if (f > maxV) f = maxV;
            return f;
        };
        controlInputs.throttle = getNorm(doc["throttle"], 0.0f, 0.0f, 1.0f);
        controlInputs.roll     = getNorm(doc["roll"],     0.0f, -1.0f, 1.0f);
        controlInputs.pitch    = getNorm(doc["pitch"],    0.0f, -1.0f, 1.0f);
        controlInputs.yaw      = getNorm(doc["yaw"],      0.0f, -1.0f, 1.0f);
        controlInputs.timestamp = millis();

        static int control_debug_counter = 0;
        if (++control_debug_counter >= 20) { // ~1s
            control_debug_counter = 0;
            Serial.printf("Web Control Input - Thr: %.2f R: %.2f P: %.2f Y: %.2f (gate %.2f)\n",
                          controlInputs.throttle, controlInputs.roll, controlInputs.pitch, controlInputs.yaw, throttleGate);
        }
        server.send(200, "text/plain", "OK"); });

    // API: Arm motors
    server.on("/api/arm", HTTP_POST, []()
              {
        Serial.println("ARM button pressed via web interface");
        controlInputs.arm_switch = true;
        controlInputs.timestamp = millis();
        Serial.printf("Control inputs updated - arm_switch: %s, timestamp: %lu\n", 
                     controlInputs.arm_switch ? "true" : "false", 
                     controlInputs.timestamp);
        server.send(200, "text/plain", "Arm command sent"); });

    // API: Disarm motors
    server.on("/api/disarm", HTTP_POST, []()
              {
        Serial.println("DISARM button pressed via web interface");
        controlInputs.arm_switch = false;
        controlInputs.timestamp = millis();
        Serial.printf("Control inputs updated - arm_switch: %s, timestamp: %lu\n", 
                     controlInputs.arm_switch ? "true" : "false", 
                     controlInputs.timestamp);
        server.send(200, "text/plain", "Disarm command sent"); });

    // API: Toggle flight mode
    server.on("/api/toggle_mode", HTTP_POST, []()
              {
        Serial.println("TOGGLE MODE button pressed via web interface");
        controlInputs.mode_switch = !controlInputs.mode_switch;
        controlInputs.timestamp = millis();
        Serial.printf("Mode switch toggled to: %s\n", 
                     controlInputs.mode_switch ? "true" : "false");
        server.send(200, "text/plain", "Mode toggled"); });

    // API: Emergency stop
    server.on("/api/emergency", HTTP_POST, []()
              {
        flightState.emergency_stop = true;
        Serial.println("EMERGENCY STOP ACTIVATED via web interface!");
        server.send(200, "text/plain", "Emergency stop activated"); });

    // API: Update PID parameters
    server.on("/api/pid_update", HTTP_POST, []()
              {
        if (server.hasArg("plain")) {
            StaticJsonDocument<768> doc;
            deserializeJson(doc, server.arg("plain"));
            
            // Update PID parameters
            pidParams.roll_kp = doc["roll_p"];
            pidParams.roll_ki = doc["roll_i"];
            pidParams.roll_kd = doc["roll_d"];
            pidParams.pitch_kp = doc["pitch_p"];
            pidParams.pitch_ki = doc["pitch_i"];
            pidParams.pitch_kd = doc["pitch_d"];
            pidParams.yaw_kp = doc["yaw_p"];
            pidParams.yaw_ki = doc["yaw_i"];
            pidParams.yaw_kd = doc["yaw_d"];
            if (doc.containsKey("alt_p")) pidParams.altitude_kp = doc["alt_p"].as<double>();
            if (doc.containsKey("alt_i")) pidParams.altitude_ki = doc["alt_i"].as<double>();
            if (doc.containsKey("alt_d")) pidParams.altitude_kd = doc["alt_d"].as<double>();
            // Extended mixer / control constants (live tuning)
            if (doc.containsKey("mix_kr")) MIX_KR = doc["mix_kr"]; 
            if (doc.containsKey("mix_kp")) MIX_KP = doc["mix_kp"]; 
            if (doc.containsKey("mix_ky")) MIX_KY = doc["mix_ky"]; 
            if (doc.containsKey("manual_axis_gain")) manualAxisGain = doc["manual_axis_gain"].as<float>();
            if (doc.containsKey("throttle_gate"))    throttleGate   = doc["throttle_gate"].as<float>();
            if (doc.containsKey("slew_rp"))          slewRateRP     = doc["slew_rp"].as<float>();
            if (doc.containsKey("slew_yaw"))         slewRateYaw    = doc["slew_yaw"].as<float>();
            if (doc.containsKey("yaw_ff"))           yawFeedForward = doc["yaw_ff"].as<float>();
            if (doc.containsKey("hover_thr"))        flightState.hover_throttle = doc["hover_thr"].as<float>();
            if (doc.containsKey("i_leak")) {
                float leak = doc["i_leak"].as<float>();
                leak = constrain(leak, 0.0f, 0.05f);
                rollPID.iTermLeak = leak; pitchPID.iTermLeak = leak; yawPID.iTermLeak = leak; altitudePID.iTermLeak = leak; rollRatePID.iTermLeak = leak; pitchRatePID.iTermLeak = leak;
            }
            
            // Apply to custom PID controllers
            rollPID.setTunings(pidParams.roll_kp, pidParams.roll_ki, pidParams.roll_kd);
            pitchPID.setTunings(pidParams.pitch_kp, pidParams.pitch_ki, pidParams.pitch_kd);
            yawPID.setTunings(pidParams.yaw_kp, pidParams.yaw_ki, pidParams.yaw_kd);
            altitudePID.setTunings(pidParams.altitude_kp, pidParams.altitude_ki, pidParams.altitude_kd);
            
            // Advanced fusion/control toggles and parameters
            if (doc.containsKey("use_kalman")) {
                USE_KALMAN_ATTITUDE = doc["use_kalman"].as<bool>();
            }
            if (doc.containsKey("kalman_rate_std")) KALMAN_RATE_STD = doc["kalman_rate_std"].as<float>();
            if (doc.containsKey("kalman_meas_std")) KALMAN_MEAS_STD = doc["kalman_meas_std"].as<float>();

            if (doc.containsKey("use_cascaded")) {
                USE_CASCADED_ANGLE_RATE = doc["use_cascaded"].as<bool>();
            }
            if (doc.containsKey("max_rp_rate")) MAX_ROLL_PITCH_RATE = doc["max_rp_rate"].as<float>();
            if (doc.containsKey("rate_roll_p")) ratePidParams.roll_kp = doc["rate_roll_p"].as<double>();
            if (doc.containsKey("rate_roll_i")) ratePidParams.roll_ki = doc["rate_roll_i"].as<double>();
            if (doc.containsKey("rate_roll_d")) ratePidParams.roll_kd = doc["rate_roll_d"].as<double>();
            if (doc.containsKey("rate_pitch_p")) ratePidParams.pitch_kp = doc["rate_pitch_p"].as<double>();
            if (doc.containsKey("rate_pitch_i")) ratePidParams.pitch_ki = doc["rate_pitch_i"].as<double>();
            if (doc.containsKey("rate_pitch_d")) ratePidParams.pitch_kd = doc["rate_pitch_d"].as<double>();

            // Apply to inner rate PIDs
            rollRatePID.setTunings(ratePidParams.roll_kp, ratePidParams.roll_ki, ratePidParams.roll_kd);
            pitchRatePID.setTunings(ratePidParams.pitch_kp, ratePidParams.pitch_ki, ratePidParams.pitch_kd);
            
            Serial.println("PID parameters updated");
            server.send(200, "text/plain", "PID updated");
        } else {
            server.send(400, "text/plain", "No data");
        } });

    // API: Update Motor Calibration
    server.on("/api/motor_calibration", HTTP_POST, []()
              {
        if (server.hasArg("plain")) {
            StaticJsonDocument<256> doc;
            deserializeJson(doc, server.arg("plain"));
            
            // Update motor calibration offsets
            motorCal.motor1_offset = doc["motor1_offset"];
            motorCal.motor2_offset = doc["motor2_offset"];
            motorCal.motor3_offset = doc["motor3_offset"];
            motorCal.motor4_offset = doc["motor4_offset"];
            
            Serial.printf("Motor calibration updated - M1: %.1f, M2: %.1f, M3: %.1f, M4: %.1f\n",
                         motorCal.motor1_offset, motorCal.motor2_offset,
                         motorCal.motor3_offset, motorCal.motor4_offset);
            server.send(200, "text/plain", "Motor calibration updated");
        } else {
            server.send(400, "text/plain", "No data");
        } });

    // API: Debug Data for Web Dashboard
    server.on("/api/debug", HTTP_GET, []()
              {
        StaticJsonDocument<1024> doc;
        
        // Status Section
        doc["status"]["mode"] = debugData.flight_mode;
        doc["status"]["armed"] = debugData.armed;
        doc["status"]["throttle"] = debugData.throttle_input;
        doc["status"]["throttle_pwm"] = debugData.throttle_pwm;
        doc["status"]["roll"] = debugData.roll_angle;
        doc["status"]["pitch"] = debugData.pitch_angle;
        doc["status"]["yaw_rate"] = debugData.yaw_rate;
        doc["status"]["altitude"] = debugData.altitude;
        doc["status"]["battery"] = debugData.battery_voltage;
        
        // PID Debug Section - Enhanced with custom PID diagnostics
        doc["pid"]["active"] = debugData.pid_active;
        doc["pid"]["enabled_by_throttle"] = debugData.pid_enabled_by_throttle;
        doc["pid"]["roll_output"] = debugData.roll_output;
        doc["pid"]["pitch_output"] = debugData.pitch_output;
        doc["pid"]["yaw_output"] = debugData.yaw_output;
        doc["pid"]["roll_setpoint"] = debugData.roll_setpoint;
        doc["pid"]["pitch_setpoint"] = debugData.pitch_setpoint;
        doc["pid"]["yaw_setpoint"] = debugData.yaw_setpoint;
    doc["pid"]["alt_p"] = pidParams.altitude_kp;
    doc["pid"]["alt_i"] = pidParams.altitude_ki;
    doc["pid"]["alt_d"] = pidParams.altitude_kd;
    doc["pid"]["hover_thr"] = flightState.hover_throttle;
    doc["pid"]["i_leak"] = rollPID.iTermLeak;
        
        // Advanced PID Diagnostics - Individual P, I, D components
        doc["pid"]["roll_p"] = debugData.roll_proportional;
        doc["pid"]["roll_i"] = debugData.roll_integral;
        doc["pid"]["roll_d"] = debugData.roll_derivative;
        doc["pid"]["roll_sat"] = debugData.roll_output_saturated;
        doc["pid"]["roll_i_sat"] = debugData.roll_integral_saturated;
        
        doc["pid"]["pitch_p"] = debugData.pitch_proportional;
        doc["pid"]["pitch_i"] = debugData.pitch_integral;
        doc["pid"]["pitch_d"] = debugData.pitch_derivative;
        doc["pid"]["pitch_sat"] = debugData.pitch_output_saturated;
        doc["pid"]["pitch_i_sat"] = debugData.pitch_integral_saturated;
        
        doc["pid"]["yaw_p"] = debugData.yaw_proportional;
        doc["pid"]["yaw_i"] = debugData.yaw_integral;
        doc["pid"]["yaw_d"] = debugData.yaw_derivative;
        doc["pid"]["yaw_sat"] = debugData.yaw_output_saturated;
        doc["pid"]["yaw_i_sat"] = debugData.yaw_integral_saturated;
        
        // Calibration Debug Section
        doc["calibration"]["active"] = debugData.imu_cal_active;
        doc["calibration"]["roll_offset"] = debugData.roll_cal_offset;
        doc["calibration"]["pitch_offset"] = debugData.pitch_cal_offset;
        
        // Motor Mix Section
        doc["motors"]["base_pwm"] = debugData.base_pwm;
        doc["motors"]["motor1"] = debugData.motor1_final;
        doc["motors"]["motor2"] = debugData.motor2_final;
        doc["motors"]["motor3"] = debugData.motor3_final;
        doc["motors"]["motor4"] = debugData.motor4_final;
        doc["motors"]["cal1"] = debugData.motor1_offset;
        doc["motors"]["cal2"] = debugData.motor2_offset;
        doc["motors"]["cal3"] = debugData.motor3_offset;
        doc["motors"]["cal4"] = debugData.motor4_offset;
        
        // Inputs Section
        doc["inputs"]["throttle"] = debugData.throttle_input;
        doc["inputs"]["roll"] = debugData.roll_input;
        doc["inputs"]["pitch"] = debugData.pitch_input;
        doc["inputs"]["yaw"] = debugData.yaw_input;
        
        // System Section
        doc["system"]["sensors_ok"] = debugData.sensors_ok;
        doc["system"]["emergency"] = debugData.emergency_active;
        doc["system"]["imu_calibrated"] = debugData.imu_calibrated;
        doc["system"]["uptime"] = debugData.uptime;
        doc["system"]["free_heap"] = debugData.free_heap;
        doc["system"]["timestamp"] = debugData.timestamp;
        
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response); });

    server.begin();
    Serial.println("Web server started - Flight Controller Dashboard ready");
}

// ================================
// LED CONTROL FUNCTIONS
// ================================

void initializeLEDs()
{
    // Initialize navigation LED pins
    pinMode(NAV_FRONT_RIGHT_GREEN, OUTPUT);
    pinMode(NAV_BACK_RIGHT_GREEN, OUTPUT);
    pinMode(NAV_BACK_LEFT_RED, OUTPUT);
    pinMode(NAV_FRONT_CENTER_WHITE, OUTPUT);
    pinMode(NAV_FRONT_LEFT_RED, OUTPUT);
    pinMode(NAV_BACK_CENTER_WHITE, OUTPUT);

    // Turn on navigation lights immediately (always on when powered)
    digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
    digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
    digitalWrite(NAV_BACK_LEFT_RED, HIGH);
    digitalWrite(NAV_FRONT_LEFT_RED, HIGH);

    // Center white LEDs start off (controlled by status)
    digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
    digitalWrite(NAV_BACK_CENTER_WHITE, LOW);

    Serial.println("Navigation LEDs initialized");
}

void updateNavigationLights()
{
    // Nav lights (Red/Green) are always ON during powered-on state
    // This helps identify orientation at night (Red = left, Green = right)
    static bool nav_lights_on = true;

    if (nav_lights_on)
    {
        digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
        digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
        digitalWrite(NAV_BACK_LEFT_RED, HIGH);
        digitalWrite(NAV_FRONT_LEFT_RED, HIGH);
    }
}

void updateStatusLights()
{
    static unsigned long last_blink_time = 0;
    static bool blink_state = false;
    unsigned long now = millis();

    // Center white LEDs behavior based on flight state
    if (!flightState.motors_armed)
    {
        // Arming Blink: Slow blink when waiting to arm (1Hz)
        if (now - last_blink_time > 500)
        {
            last_blink_time = now;
            blink_state = !blink_state;
            digitalWrite(NAV_FRONT_CENTER_WHITE, blink_state);
            digitalWrite(NAV_BACK_CENTER_WHITE, blink_state);
        }
    }
    else
    {
        // Flight Mode On: Solid light when armed
        digitalWrite(NAV_FRONT_CENTER_WHITE, HIGH);
        digitalWrite(NAV_BACK_CENTER_WHITE, HIGH);

        // Optional: Throttle Sync Pulse - Flash intensity reflects throttle level
        setThrottlePulse(controlInputs.throttle);
    }
}

void setThrottlePulse(float throttle_percentage)
{
    // Optional throttle sync pulse feature
    // Flash the center white LEDs with intensity reflecting throttle level
    static unsigned long last_pulse_time = 0;
    static bool pulse_state = false;
    unsigned long now = millis();

    // Only pulse if throttle is above minimum and motors are armed
    if (flightState.motors_armed && throttle_percentage > 0.05)
    {
        // Pulse frequency increases with throttle (10Hz to 50Hz)
        int pulse_interval = map(throttle_percentage * 100, 5, 100, 100, 20);

        if (now - last_pulse_time > pulse_interval)
        {
            last_pulse_time = now;
            pulse_state = !pulse_state;

            // Brief flash to indicate throttle activity
            if (pulse_state)
            {
                digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
                digitalWrite(NAV_BACK_CENTER_WHITE, LOW);
            }
            else
            {
                digitalWrite(NAV_FRONT_CENTER_WHITE, HIGH);
                digitalWrite(NAV_BACK_CENTER_WHITE, HIGH);
            }
        }
    }
}
