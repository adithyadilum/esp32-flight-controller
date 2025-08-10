/*
 * DEVELOPMENT VERSION - Complete Remote Controller - Stable Non-FreeRTOS Version
 * 🔬 DEVELOPMENT FIRMWARE FOR ENHANCED CONTROL FEATURES
 *
 * Reliable control at 200ms intervals (5Hz) for stable drone communication
 * Compatible with droneFreeRTOS.ino
 *
 * DEVELOPMENT FOCUS:
 * - Advanced control modes and flight assistance
 * - Real-time PID parameter tuning interface
 * - Enhanced telemetry display and data logging
 * - Flight mode switching (Stabilized/Manual) and safety enhancements
 *
 * CONTROL FEATURES:
 * - Virtual throttle with rate limiting for smooth control
 * - ARM/DISARM safety switch (Toggle 1)
 * - Flight mode switching: Stabilized/Manual (Toggle 2)
 * - Comprehensive telemetry display and Firebase logging
 * - Manual throttle reset via joystick button
 *
 * ⚠️ EXPERIMENTAL - FOR DEVELOPMENT USE ONLY
 * Use stable_remote/remoteControllerStable.ino for production flights
 */

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Firebase configuration
#define FIREBASE_HOST "skyforge-4606b-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "HBgETEJwFCsLswgXH2SuzYupJPWhJqWnJdfkFvGO"
#define WIFI_SSID "Dialog 4G"
#define WIFI_PASSWORD "0N7NT00ANTQ"

// Upload frequency configuration (in milliseconds)
#define FIREBASE_UPLOAD_INTERVAL 60000 // 60 seconds (conservative for stability)
#define FIREBASE_ENABLED true          // Enabled with simple approach

// OLED Display Configuration - Set to false to completely disable OLED functionality
#define OLED_ENABLED false // Set to false to disable OLED display functionality for now

// Common testing intervals:
// 10000  = 10 seconds (normal)
// 30000  = 30 seconds (slow/production)
// 60000  = 1 minute (very slow)

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5

// OLED Display Configuration
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Joystick pins - Updated configuration
#define JOY1_X_PIN 39   // Joystick 1 X-axis
#define JOY1_Y_PIN 36   // Joystick 1 Y-axis
#define JOY1_BTN_PIN 33 // Joystick 1 Button (Note: Not functioning well)
#define JOY2_X_PIN 34   // Joystick 2 X-axis
#define JOY2_Y_PIN 35   // Joystick 2 Y-axis
#define JOY2_BTN_PIN 32 // Joystick 2 Button
#define TOGGLE_SW1_PIN 27
#define TOGGLE_SW2_PIN 14

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// OLED Display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Control packet (12 bytes) - Added button states and toggle switches
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

// Enhanced telemetry packet (22 bytes) - matches drone with lux, altitude, UV index, eCO2, and TVOC
struct TelemetryPacket
{
    int16_t temperature; // x100 - Real BME280 data
    uint16_t pressure;   // x10 - Real BME280 data
    uint8_t humidity;    // % - Real BME280 data
    uint16_t battery;    // mV - Real battery voltage
    int16_t latitude;    // GPS latitude (simplified)
    int16_t longitude;   // GPS longitude (simplified)
    uint8_t satellites;  // GPS satellite count
    uint8_t status;      // System status
    uint16_t lux;        // Light level in lux
    int16_t altitude;    // Altitude in centimeters from BME280 (for 2 decimal precision)
    uint16_t uvIndex;    // UV index x100 from GUVA sensor
    uint16_t eCO2;       // Equivalent CO2 in ppm from ENS160
    uint16_t TVOC;       // Total VOC in ppb from ENS160
};

ControlPacket controlData;
TelemetryPacket telemetryData;
unsigned long lastControlSend = 0;
unsigned long lastFirebaseUpload = 0;
unsigned long lastTelemetryPrint = 0;
unsigned long lastStatusPrint = 0;
bool telemetryReceived = false;
bool firebaseReady = false;
bool wifiConnected = false;

// Virtual Throttle Mode variables
int16_t virtualThrottle = 0;           // Current virtual throttle value (0 to 3000)
const int16_t THROTTLE_DEADZONE = 200; // Deadzone around center (±200)
const int16_t THROTTLE_RATE = 120;     // Throttle change rate per update (reach max in ~5 seconds)
const int16_t THROTTLE_MIN = 0;        // Minimum throttle value (0% power)
const int16_t THROTTLE_MAX = 3000;     // Maximum throttle value (100% power)
const int16_t CONTROL_DEADZONE = 150;  // Deadzone for roll/pitch/yaw controls

// Safety and control mode variables
bool isArmed = false;          // Arm/disarm state (Toggle 1)
bool isStabilizedMode = true;  // Flight mode state (Toggle 2) - true = Stabilized, false = Manual
bool lastToggle1State = false; // For edge detection
bool lastToggle2State = false; // For edge detection

// Statistics tracking
int successCount = 0;
int failCount = 0;
int telemetryCount = 0;
int packetNumber = 1;
bool displayAvailable = false; // Track if OLED display is working

void scanI2CDevices()
{
    Serial.println("🔍 Scanning I2C bus for devices...");
    delay(100); // Give I2C devices time to initialize

    int deviceCount = 0;
    bool oledFound = false;

    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("✅ I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println();
            deviceCount++;

            // Check if it's likely an OLED display
            if (address == 0x3C || address == 0x3D)
            {
                Serial.println("   ^ This looks like an OLED display!");
                oledFound = true;
            }
        }
        else if (error == 4)
        {
            Serial.print("⚠️  Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" (error code 4)");
        }

        delay(2); // Small delay between addresses for reliability
    }

    Serial.println("────────────────────────────────");
    if (deviceCount == 0)
    {
        Serial.println("❌ No I2C devices found!");
        Serial.println("💡 Check your wiring:");
        Serial.println("   - SDA should be connected to GPIO 21");
        Serial.println("   - SCL should be connected to GPIO 22");
        Serial.println("   - VCC should be connected to 3.3V");
        Serial.println("   - GND should be connected to GND");
    }
    else
    {
        Serial.print("✅ Total I2C devices found: ");
        Serial.println(deviceCount);
        if (oledFound)
        {
            Serial.println("🎉 OLED display detected!");
        }
        else
        {
            Serial.println("⚠️  No OLED display found at 0x3C or 0x3D");
        }
    }
    Serial.println("I2C scan complete\n");
}

void updateOLEDDisplay()
{
    if (!displayAvailable)
    {
        return; // Skip display update if display is not available
    }

    // Try to update display with improved error handling
    static int displayErrorCount = 0;
    static unsigned long lastErrorReset = 0;
    const int maxDisplayErrors = 5;
    const unsigned long errorResetInterval = 30000; // Reset error count every 30 seconds

    // Reset error count periodically to allow recovery
    if (millis() - lastErrorReset > errorResetInterval)
    {
        if (displayErrorCount > 0)
        {
            Serial.print("🔄 Resetting OLED error count (was ");
            Serial.print(displayErrorCount);
            Serial.println("), attempting recovery...");
        }
        displayErrorCount = 0;
        lastErrorReset = millis();
    }

    // If we've had too many errors, disable display updates temporarily
    if (displayErrorCount >= maxDisplayErrors)
    {
        static unsigned long lastErrorMessage = 0;
        if (millis() - lastErrorMessage > 10000) // Print message every 10 seconds
        {
            Serial.println("⚠️  OLED Display temporarily disabled due to I2C errors");
            Serial.println("   Will auto-retry in a few seconds...");
            lastErrorMessage = millis();
        }
        return;
    }

    // Attempt display update with error recovery
    bool updateSuccess = false;
    int attempts = 0;
    const int maxAttempts = 2; // Reduced attempts to minimize blocking

    while (!updateSuccess && attempts < maxAttempts)
    {
        attempts++;

        // Small delay between attempts
        if (attempts > 1)
        {
            delay(5);
        }

        // Clear and prepare display
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);

        // Display content
        display.println("DRONE CONTROLLER");
        display.println("================");

        // Display ARM/DISARM status
        if (isArmed)
        {
            display.println("STATUS: ARMED");
        }
        else
        {
            display.println("STATUS: DISARMED");
        }

        // Display control values
        display.println();
        display.printf("T:%4d(%2d%%) Y:%4d\n",
                       virtualThrottle,
                       (virtualThrottle * 100) / 3000,
                       controlData.yaw);
        display.printf("R:%4d  P:%4d\n", controlData.roll, controlData.pitch);

        // Display connection status
        display.println();
        if (telemetryReceived)
        {
            display.println("CONN: OK");
        }
        else
        {
            display.println("CONN: LOST");
        }

        // Try to update the physical display with I2C error checking
        Wire.beginTransmission(SCREEN_ADDRESS);
        byte i2cError = Wire.endTransmission();

        if (i2cError == 0)
        {
            // I2C communication successful, update display
            display.display();
            updateSuccess = true;
            // Reset error count on success
            if (displayErrorCount > 0)
            {
                displayErrorCount = 0;
                Serial.println("✅ OLED display recovered successfully!");
            }
        }
        else
        {
            displayErrorCount++;
            if (displayErrorCount <= maxDisplayErrors)
            {
                Serial.printf("⚠️  OLED I2C error (attempt %d/%d, error count: %d, I2C error: %d)\n",
                              attempts, maxAttempts, displayErrorCount, i2cError);
            }
        }
    }

    if (!updateSuccess && displayErrorCount <= maxDisplayErrors)
    {
        Serial.printf("❌ OLED update failed after %d attempts (total errors: %d)\n",
                      maxAttempts, displayErrorCount);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== Complete Remote Controller - Stable Version ===");
    Serial.println("Compatible with droneFreeRTOS.ino");

// Initialize I2C only if OLED is enabled
#if OLED_ENABLED
    Serial.println("🔧 Initializing I2C for OLED display...");
    Wire.begin(21, 22);    // SDA=21, SCL=22 (ESP32 default)
    Wire.setClock(100000); // Set I2C clock to 100kHz for better reliability
    delay(200);            // Give I2C time to stabilize

    Serial.println("📍 I2C initialized - SDA:GPIO21, SCL:GPIO22, Clock:100kHz");

    // Scan for I2C devices first
    scanI2CDevices();

    // Initialize OLED Display with comprehensive error handling
    Serial.println("🖥️  Initializing OLED Display...");
    delay(100); // Additional delay before display initialization

    // Try multiple initialization attempts
    bool displayInitialized = false;
    int initAttempts = 0;
    const int maxInitAttempts = 3;

    while (!displayInitialized && initAttempts < maxInitAttempts)
    {
        initAttempts++;
        Serial.print("   Attempt ");
        Serial.print(initAttempts);
        Serial.print("/");
        Serial.print(maxInitAttempts);
        Serial.println("...");

        // Try address 0x3C first (most common)
        if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
        {
            Serial.println("✅ OLED Display initialized at address 0x3C!");
            displayInitialized = true;
            displayAvailable = true;
        }
        else
        {
            Serial.println("❌ Failed at address 0x3C, trying 0x3D...");

            // Try alternative address 0x3D
            if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
            {
                Serial.println("✅ OLED Display initialized at address 0x3D!");
                displayInitialized = true;
                displayAvailable = true;
            }
            else
            {
                Serial.print("❌ Failed at address 0x3D (attempt ");
                Serial.print(initAttempts);
                Serial.println(")");
                delay(500); // Wait before retry
            }
        }
    }

    if (!displayInitialized)
    {
        Serial.println("❌ OLED Display initialization FAILED after all attempts!");
        Serial.println("🔍 Troubleshooting steps:");
        Serial.println("   1. Check wiring connections:");
        Serial.println("      - VCC to 3.3V (NOT 5V for most displays)");
        Serial.println("      - GND to GND");
        Serial.println("      - SDA to GPIO 21");
        Serial.println("      - SCL to GPIO 22");
        Serial.println("   2. Verify display specifications (0.96\" SSD1306)");
        Serial.println("   3. Check if display requires different I2C address");
        Serial.println("   4. Ensure pull-up resistors on SDA/SCL (usually built-in)");
        Serial.println("⚠️  Continuing without display - drone control will work normally");
        displayAvailable = false;
    }
    else
    {
        // Show startup message if display is available
        Serial.println("🎉 Testing OLED display functionality...");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("DRONE CONTROLLER"));
        display.println(F("================"));
        display.println(F("System: ONLINE"));
        display.println(F("Display: OK"));
        display.println(F("Initializing..."));

        display.display(); // This function returns void
        Serial.println("✅ OLED display test successful!");
        delay(2000); // Show startup message for 2 seconds
    }
#else
    Serial.println("⚠️  OLED Display disabled in configuration");
    Serial.println("💡 System optimized for drone control without display");
    displayAvailable = false;
#endif // Initialize pins
    pinMode(JOY1_X_PIN, INPUT);
    pinMode(JOY1_Y_PIN, INPUT);
    pinMode(JOY1_BTN_PIN, INPUT_PULLUP); // Joystick 1 button (not functioning well)
    pinMode(JOY2_X_PIN, INPUT);
    pinMode(JOY2_Y_PIN, INPUT);
    pinMode(JOY2_BTN_PIN, INPUT_PULLUP); // Joystick 2 button
    pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP);

    // Initialize control data with safe defaults
    controlData.throttle = 0;
    controlData.roll = 0;
    controlData.pitch = 0;
    controlData.yaw = 0;
    controlData.joy1_btn = 1; // Released
    controlData.joy2_btn = 1; // Released
    controlData.toggle1 = 0;  // Off
    controlData.toggle2 = 0;  // Off

    // Initialize virtual throttle
    virtualThrottle = 0;

    // Initialize safety states
    isArmed = false;
    isStabilizedMode = true; // Default to stabilized mode
    lastToggle1State = false;
    lastToggle2State = false;

    // Initialize radio with PROVEN configuration
    SPI.begin();
    delay(100);

    if (!radio.begin() || !radio.isChipConnected())
    {
        Serial.println("ERROR: Radio initialization failed!");
        while (1)
            delay(1000);
    }

    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_HIGH);  // High power for range
    radio.setDataRate(RF24_250KBPS); // Keep 250kbps for robustness
    radio.setChannel(110);           // Match drone channel, away from Wi-Fi
    radio.setAutoAck(true);
    radio.setRetries(3, 5); // Lower retry latency; rely on 50Hz updates
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    // Additional range improvements
    radio.setCRCLength(RF24_CRC_16); // Use 16-bit CRC for better error detection
    radio.setAddressWidth(5);        // Use full 5-byte addresses

    radio.stopListening();

    Serial.println("✓ Radio configured successfully!");

    // Initialize WiFi and Firebase only if enabled
    if (FIREBASE_ENABLED)
    {
        initializeWiFi();
        if (wifiConnected)
        {
            initializeFirebase();
        }
    }
    else
    {
        Serial.println("Firebase disabled - running in offline mode");
    }

    Serial.print("Starting PROVEN control transmission (5Hz) with virtual throttle mode");
    if (FIREBASE_ENABLED)
    {
        Serial.print(" and cloud upload every ");
        Serial.print(FIREBASE_UPLOAD_INTERVAL / 1000);
        Serial.println(" seconds...");
    }
    else
    {
        Serial.println(" (Firebase disabled)...");
    }

    Serial.println("Virtual Throttle Mode Features:");
    Serial.print("  - Deadzone: ±");
    Serial.print(THROTTLE_DEADZONE);
    Serial.print(", Rate: ");
    Serial.print(THROTTLE_RATE);
    Serial.print("/update, Range: ");
    Serial.print(THROTTLE_MIN);
    Serial.print(" to ");
    Serial.print(THROTTLE_MAX);
    Serial.println(" (0-100% power)");

    Serial.println("Safety Features:");
    Serial.println("  - Toggle 1: ARM/DISARM (must be ON to control throttle)");
    Serial.println("  - Toggle 2: FLIGHT MODE (ON = Stabilized, OFF = Manual)");

    Serial.println("Flight Modes:");
    Serial.println("  - STABILIZED: Drone auto-levels and maintains attitude");
    Serial.println("  - MANUAL: Direct control, no auto-leveling (advanced users)");

    Serial.println("Throttle Mapping:");
    Serial.println("  - Virtual 0 = Transmitted -3000 (0% power)");
    Serial.println("  - Virtual 3000 = Transmitted +3000 (100% power)"); // Print radio configuration for debugging
    Serial.println("Radio Configuration:");
    Serial.print("  Power Level: RF24_PA_HIGH");
    Serial.print(", Data Rate: RF24_250KBPS");
    Serial.print(", Channel: 76");
    Serial.print(", CRC: 16-bit");
    Serial.println(", Address Width: 5 bytes");
}

void loop()
{
    unsigned long currentTime = millis();

    // Send control data at 5Hz (200ms intervals) - PROVEN WORKING RATE
    if (currentTime - lastControlSend >= 20)
    {
        readJoystickInputs();
        sendControlData();
        lastControlSend = currentTime;
    }

    // Print telemetry every 2 seconds if available
    if (telemetryReceived && (currentTime - lastTelemetryPrint >= 2000))
    {
        printTelemetryData();
        lastTelemetryPrint = currentTime;
    }

    // Print status every 10 seconds
    if (currentTime - lastStatusPrint >= 10000)
    {
        printStatusReport();
        lastStatusPrint = currentTime;
    }

    // Upload to Firebase at specified interval
    if (FIREBASE_ENABLED && firebaseReady && telemetryReceived &&
        (currentTime - lastFirebaseUpload >= FIREBASE_UPLOAD_INTERVAL))
    {
        uploadTelemetryToFirebase();
        lastFirebaseUpload = currentTime;
    }

    // Check WiFi connection periodically
    if (FIREBASE_ENABLED && (currentTime % 30000 == 0)) // Every 30 seconds
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            wifiConnected = false;
            firebaseReady = false;
            Serial.println("WiFi disconnected, attempting reconnection...");
            initializeWiFi();
            if (wifiConnected)
            {
                initializeFirebase();
            }
        }
        else
        {
            wifiConnected = true;
        }
    }

    // Small delay to prevent overwhelming the system
    delay(10);

    // Handle toggle switch safety and mode features
    handleSafetyToggleSwitches();

    // Update OLED display every 1 second (only if available) - reduced frequency to minimize I2C errors
    if (displayAvailable)
    {
        static unsigned long lastDisplayUpdate = 0;
        if (currentTime - lastDisplayUpdate >= 1000)
        {
            updateOLEDDisplay();
            lastDisplayUpdate = currentTime;
        }
    }
}

void readJoystickInputs()
{
    int joy1_x = analogRead(JOY1_X_PIN);
    int joy1_y = analogRead(JOY1_Y_PIN);
    int joy2_x = analogRead(JOY2_X_PIN);
    int joy2_y = analogRead(JOY2_Y_PIN);

    // Read joystick buttons (active LOW, so invert the reading)
    controlData.joy1_btn = digitalRead(JOY1_BTN_PIN); // Note: JOY1 button may not function well
    controlData.joy2_btn = digitalRead(JOY2_BTN_PIN);

    // Read toggle switches (active LOW, so invert the reading)
    controlData.toggle1 = !digitalRead(TOGGLE_SW1_PIN); // ARM/DISARM switch
    controlData.toggle2 = !digitalRead(TOGGLE_SW2_PIN); // STABILIZED/MANUAL mode switch

    // === FLIGHT MODE CONTROL ===
    // No emergency stop - removed for mode switching

    // === VIRTUAL THROTTLE MODE ===
    // Only process throttle if armed (Toggle 1 ON)
    if (isArmed)
    {
        // Map joystick Y to throttle rate (-3000 to 3000 range)
        int16_t throttleInput = map(joy1_y, 0, 4095, -3000, 3000);

        // Apply deadzone filtering for throttle
        if (abs(throttleInput) < THROTTLE_DEADZONE)
        {
            throttleInput = 0; // No change when in deadzone
        }

        // Calculate throttle change rate based on stick position
        int16_t throttleChange = 0;
        if (throttleInput > THROTTLE_DEADZONE)
        {
            // Stick pushed up = increase throttle
            throttleChange = map(throttleInput, THROTTLE_DEADZONE, 3000, 1, THROTTLE_RATE);
        }
        else if (throttleInput < -THROTTLE_DEADZONE)
        {
            // Stick pulled down = decrease throttle
            throttleChange = map(throttleInput, -3000, -THROTTLE_DEADZONE, -THROTTLE_RATE, -1);
        }

        // Update virtual throttle with rate limiting
        virtualThrottle += throttleChange;

        // Constrain virtual throttle to limits (0 to 3000)
        if (virtualThrottle > THROTTLE_MAX)
            virtualThrottle = THROTTLE_MAX;
        if (virtualThrottle < THROTTLE_MIN)
            virtualThrottle = THROTTLE_MIN;

        // Convert virtual throttle to absolute transmission value
        // Virtual 0-3000 maps to transmitted -3000 to +3000
        controlData.throttle = map(virtualThrottle, 0, 3000, -3000, 3000);
    }
    else
    {
        // Disarmed - throttle locked to minimum
        controlData.throttle = -3000; // Send minimum throttle value
        virtualThrottle = 0;          // Reset virtual throttle when disarmed
    }

    // === STANDARD CONTROLS WITH DEADZONE ===
    // Only process other controls if armed
    if (isArmed)
    {
        // Map other controls with deadzone filtering
        int16_t yawInput = map(joy1_x, 0, 4095, -3000, 3000);
        int16_t rollInput = map(joy2_x, 0, 4095, -3000, 3000);
        int16_t pitchInput = map(joy2_y, 0, 4095, -3000, 3000);

        // Apply deadzone filtering for other controls
        controlData.yaw = (abs(yawInput) < CONTROL_DEADZONE) ? 0 : yawInput;
        controlData.roll = (abs(rollInput) < CONTROL_DEADZONE) ? 0 : rollInput;
        controlData.pitch = (abs(pitchInput) < CONTROL_DEADZONE) ? 0 : pitchInput;
    }
    else
    {
        // Disarmed - all controls locked to 0
        controlData.yaw = 0;
        controlData.roll = 0;
        controlData.pitch = 0;
    }

    // Manual throttle reset with Joy2 button
    if (controlData.joy2_btn == 0) // JOY2 button pressed
    {
        virtualThrottle = 0;
        controlData.throttle = -3000; // Send minimum throttle value
        static unsigned long lastResetMessage = 0;
        if (millis() - lastResetMessage > 1000) // Print reset message every 1 second max
        {
            Serial.println("🔄 Manual throttle RESET to 0% (JOY2 button)");
            lastResetMessage = millis();
        }
    }
}

void sendControlData()
{
    bool result = false;
    int attempts = 0;
    const int maxAttempts = 3; // Proven working retry count

    // Try multiple times for reliability
    while (!result && attempts < maxAttempts)
    {
        result = radio.write(&controlData, sizeof(controlData));
        if (!result)
        {
            attempts++;
            delayMicroseconds(100); // Small delay between attempts
        }
    }

    if (result)
    {
        successCount++;

        // Check for ACK payload with telemetry
        if (radio.isAckPayloadAvailable())
        {
            uint8_t len = radio.getDynamicPayloadSize();
            if (len == sizeof(TelemetryPacket))
            {
                radio.read(&telemetryData, sizeof(telemetryData));
                telemetryReceived = true;
                telemetryCount++;
            }
        }
    }
    else
    {
        failCount++;
        // Print failures with better formatting
        if (failCount % 10 == 0) // Every 10 failures
        {
            Serial.print("❌ Communication Failed: Packet #");
            Serial.print(packetNumber);
            Serial.print(" failed after ");
            Serial.print(attempts);
            Serial.println(" attempts");
        }
    }

    packetNumber++;
}

void printTelemetryData()
{
    // === CONTROL STATUS LINE ===
    Serial.print("📊 Packet #");
    Serial.print(packetNumber);
    Serial.print(" | Controls: VT:");
    Serial.print(controlData.throttle);
    Serial.print("(");
    Serial.print((virtualThrottle * 100) / 3000); // Show percentage
    Serial.print("%) ");

    // Debug throttle mapping
    if (controlData.throttle >= 0)
    {
        Serial.print("✅POSITIVE");
    }
    else
    {
        Serial.print("❌NEGATIVE");
    }

    Serial.print(" R:");
    Serial.print(controlData.roll);
    Serial.print(" P:");
    Serial.print(controlData.pitch);
    Serial.print(" Y:");
    Serial.print(controlData.yaw);

    // === SAFETY STATUS ===
    Serial.print(" | Safety: ");
    if (isArmed)
        Serial.print("🔓ARMED");
    else
        Serial.print("🔒DISARMED");

    // === FLIGHT MODE STATUS ===
    Serial.print(" | Mode: ");
    if (isStabilizedMode)
        Serial.print("�️STAB");
    else
        Serial.print("🎯MAN");

    // === THROTTLE STATUS ===
    Serial.print(" | Thr: ");
    if (virtualThrottle == 0)
    {
        Serial.print("IDLE");
    }
    else if (virtualThrottle < 300)
    {
        Serial.print("LOW");
    }
    else if (virtualThrottle < 1500)
    {
        Serial.print("MED");
    }
    else
    {
        Serial.print("HIGH");
    }

    // === INPUT STATUS ===
    Serial.print(" | Inputs:");
    if (controlData.joy1_btn == 0)
        Serial.print(" JOY1");
    if (controlData.joy2_btn == 0)
        Serial.print(" JOY2");
    if (controlData.toggle1 == 1)
        Serial.print(" SW1");
    if (controlData.toggle2 == 1)
        Serial.print(" SW2");

    Serial.println(); // End control line

    // === ENVIRONMENTAL DATA LINE ===
    Serial.print("🌡️  Environment: ");
    Serial.print("Temp:");
    Serial.print(telemetryData.temperature / 100.0, 1);
    Serial.print("°C | Press:");
    Serial.print(telemetryData.pressure / 10.0, 1);
    Serial.print("hPa | Hum:");
    Serial.print(telemetryData.humidity);
    Serial.print("% | Alt:");
    Serial.print(telemetryData.altitude / 100.0, 2);
    Serial.println("m");

    // === POWER & GPS LINE ===
    Serial.print("⚡ Power & GPS: ");
    Serial.print("Batt:");
    Serial.print(telemetryData.battery);
    Serial.print("mV | GPS:");
    Serial.print(telemetryData.satellites);
    Serial.print("sats ");
    if (telemetryData.satellites > 0)
    {
        Serial.print("(");
        Serial.print(telemetryData.latitude / 100.0, 4);
        Serial.print(",");
        Serial.print(telemetryData.longitude / 100.0, 4);
        Serial.print(")");
    }
    else
    {
        Serial.print("(No Fix)");
    }
    Serial.println();

    // === AIR QUALITY LINE ===
    Serial.print("🌬️  Air Quality: ");
    Serial.print("Light:");
    Serial.print(telemetryData.lux);
    Serial.print("lx | UV:");
    Serial.print(telemetryData.uvIndex / 100.0, 2);
    Serial.print(" | CO2:");
    Serial.print(telemetryData.eCO2);
    Serial.print("ppm | TVOC:");
    Serial.print(telemetryData.TVOC);
    Serial.println("ppb");

    Serial.println("────────────────────────────────────────"); // Separator
}

void printStatusReport()
{
    Serial.println("════════════════ STATUS REPORT ════════════════");

    // === COMMUNICATION STATISTICS ===
    Serial.print("📡 Communication: Success:");
    Serial.print(successCount);
    Serial.print(" | Failed:");
    Serial.print(failCount);
    Serial.print(" | Telemetry:");
    Serial.print(telemetryCount);

    if (successCount + failCount > 0)
    {
        int successRate = (successCount * 100) / (successCount + failCount);
        Serial.print(" | Success Rate:");
        Serial.print(successRate);
        Serial.println("%");
    }
    else
    {
        Serial.println(" | Success Rate: N/A");
    }

    // === SYSTEM STATUS ===
    Serial.print("🔧 System: Free Heap:");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" bytes");

    if (FIREBASE_ENABLED)
    {
        Serial.print(" | WiFi:");
        Serial.print(wifiConnected ? "✅Connected" : "❌Disconnected");
        Serial.print(" | Firebase:");
        Serial.print(firebaseReady ? "✅Ready" : "❌Not Ready");
    }
    Serial.println();

    // === SAFETY STATUS ===
    Serial.print("🛡️  Safety: ");
    Serial.print(isArmed ? "🔓ARMED" : "🔒DISARMED");
    Serial.print(" | Flight Mode: ");
    Serial.print(isStabilizedMode ? "�️STABILIZED" : "🎯MANUAL");
    Serial.println();

    // === VIRTUAL THROTTLE STATUS ===
    Serial.print("🎮 Virtual Throttle: Current:");
    Serial.print(virtualThrottle);
    Serial.print("(");
    Serial.print((virtualThrottle * 100) / 3000);
    Serial.print("%) | Range:");
    Serial.print(THROTTLE_MIN);
    Serial.print(" to ");
    Serial.print(THROTTLE_MAX);
    Serial.print(" | Rate:");
    Serial.print(THROTTLE_RATE);
    Serial.print("/update | Transmitted:");
    Serial.print(map(virtualThrottle, 0, 3000, -3000, 3000));
    Serial.println();

    Serial.println("═══════════════════════════════════════════════");

    // Reset counters
    successCount = 0;
    failCount = 0;
}

void initializeWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.disconnect();
    delay(100);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        wifiConnected = true;
        Serial.println("\n✅ WiFi connected successfully!");
        Serial.print("🌐 IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("📶 Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");

        // Configure time for Firebase and wait for sync (TLS needs correct time)
        configTime(0, 0, "pool.ntp.org");
        time_t now = time(nullptr);
        uint32_t waited = 0;
        while (now < 1700000000 && waited < 5000)
        {
            delay(200);
            waited += 200;
            now = time(nullptr);
        }
        Serial.print("⏱ NTP time: ");
        Serial.println((unsigned long)now);
    }
    else
    {
        wifiConnected = false;
        Serial.println("\n❌ WiFi connection failed!");
        Serial.print("Status code: ");
        Serial.println(WiFi.status());
    }
}

void initializeFirebase()
{
    if (!wifiConnected)
    {
        Serial.println("Firebase init skipped - WiFi not connected");
        return;
    }

    Serial.println("Initializing Firebase connection...");

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    // Simple timeout settings
    config.timeout.serverResponse = 10 * 1000;   // 10 seconds
    config.timeout.socketConnection = 10 * 1000; // 10 seconds

    // Initialize Firebase
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    delay(1000); // Give Firebase time to initialize

    // Simple ready check
    if (Firebase.ready())
    {
        Serial.println("✅ Firebase connected successfully!");
        firebaseReady = true;

        // Test upload
        FirebaseJson testJson;
        testJson.set("test", "connection_test");
        testJson.set("timestamp", (int)time(nullptr));

        if (Firebase.setJSON(firebaseData, "/connection_test", testJson))
        {
            Serial.println("✅ Firebase write test successful!");
        }
        else
        {
            Serial.print("❌ Firebase write test failed: ");
            Serial.println(firebaseData.errorReason());
            firebaseReady = false;
        }
    }
    else
    {
        Serial.println("❌ Firebase not ready - will retry later");
        firebaseReady = false;
    }
}

void uploadTelemetryToFirebase()
{
    if (!telemetryReceived || !wifiConnected || !firebaseReady)
    {
        return; // Skip if conditions not met
    }

    // Create JSON payload with essential data
    FirebaseJson json;
    unsigned long timestamp = time(nullptr);

    json.set("timestamp", (int)timestamp);
    json.set("temperature", telemetryData.temperature / 100.0);
    json.set("humidity", telemetryData.humidity);
    json.set("pressure", telemetryData.pressure / 10.0);
    json.set("altitude", telemetryData.altitude / 100.0);
    json.set("battery", telemetryData.battery);
    json.set("lux", telemetryData.lux);
    json.set("uvIndex", telemetryData.uvIndex / 100.0);
    json.set("eCO2", telemetryData.eCO2);
    json.set("TVOC", telemetryData.TVOC);
    json.set("gps_satellites", telemetryData.satellites);
    json.set("gps_latitude", telemetryData.latitude / 100.0);
    json.set("gps_longitude", telemetryData.longitude / 100.0);
    json.set("status", telemetryData.status);

    // Upload to Firebase
    String dataPath = "/telemetry/" + String(timestamp);

    if (Firebase.setJSON(firebaseData, dataPath, json))
    {
        static unsigned long lastSuccessMessage = 0;
        if (millis() - lastSuccessMessage > 30000) // Print success every 30 seconds max
        {
            Serial.println("✅ Firebase upload successful");
            lastSuccessMessage = millis();
        }
        firebaseReady = true;
    }
    else
    {
        Serial.print("❌ Firebase upload failed: ");
        Serial.println(firebaseData.errorReason());

        // Mark Firebase as not ready if upload fails
        if (firebaseData.errorReason().indexOf("SSL") >= 0 ||
            firebaseData.errorReason().indexOf("connection") >= 0)
        {
            firebaseReady = false;
        }
    }
}

void handleSafetyToggleSwitches()
{
    // Handle ARM/DISARM toggle (Toggle 1) - edge detection
    if (controlData.toggle1 != lastToggle1State)
    {
        lastToggle1State = controlData.toggle1;

        if (controlData.toggle1 == 1) // Toggle 1 turned ON
        {
            isArmed = true;
            Serial.println("🔓 DRONE ARMED - Controls enabled");
        }
        else // Toggle 1 turned OFF
        {
            isArmed = false;
            virtualThrottle = 0;
            Serial.println("🔒 DRONE DISARMED - All controls locked to minimum");
        }
    }

    // Handle FLIGHT MODE toggle (Toggle 2) - edge detection
    if (controlData.toggle2 != lastToggle2State)
    {
        lastToggle2State = controlData.toggle2;

        if (controlData.toggle2 == 1) // Toggle 2 turned ON
        {
            isStabilizedMode = true;
            Serial.println("�️  FLIGHT MODE: STABILIZED - Auto-leveling enabled");
        }
        else // Toggle 2 turned OFF
        {
            isStabilizedMode = false;
            Serial.println("🎯 FLIGHT MODE: MANUAL - Direct control, no auto-leveling");
        }
    }
}
