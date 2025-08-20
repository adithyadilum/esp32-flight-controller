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
 * ⚠ EXPERIMENTAL - FOR DEVELOPMENT USE ONLY
 * Use stable_remote/remoteControllerStable.ino for production flights
 */

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Firebase configuration
#define FIREBASE_HOST "skyforge-4606b-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_DB_URL "https://" FIREBASE_HOST
#define FIREBASE_AUTH "HBgETEJwFCsLswgXH2SuzYupJPWhJqWnJdfkFvGO"
#define WIFI_SSID "Dialog 4G"
#define WIFI_PASSWORD "0N7NT00ANTQ"

// Upload frequency configuration (in milliseconds)
#define FIREBASE_UPLOAD_INTERVAL 1000 // 1 second (conservative for stability)
#define FIREBASE_ENABLED true         // Enabled with simple approach

// OLED Display Configuration - Set to false to completely disable OLED functionality
#define OLED_ENABLED true // Set to true to enable OLED display functionality

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

// Enhanced telemetry packet (30 bytes) - matches drone with lux, altitude, UV index, eCO2, and TVOC
struct TelemetryPacket
{
    int16_t temperature;  // x100 - Real BME280 data
    uint16_t pressureX10; // x10 - Real BME280 data (one decimal, avoids overflow)
    uint8_t humidity;     // % - Real BME280 data
    uint16_t battery;     // mV - Real battery voltage
    int32_t latitudeE7;   // latitude * 1e7
    int32_t longitudeE7;  // longitude * 1e7
    uint8_t satellites;   // GPS satellite count
    uint8_t status;       // System status
    uint16_t lux;         // Light level in lux
    int16_t altitude;     // Altitude in centimeters from BME280 (for 2 decimal precision)
    uint16_t uvIndex;     // UV index x100 from GUVA sensor
    uint16_t eCO2;        // Equivalent CO2 in ppm from ENS160
    uint16_t TVOC;        // Total VOC in ppb from ENS160
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
// Custom yaw deadzone (requested): applies only to yaw, wider than generic control deadzone
const int16_t YAW_DEADZONE = 250; // Deadzone for yaw (±250)

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
    Serial.println(" Scanning I2C bus for devices...");
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
            Serial.print("⚠  Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" (error code 4)");
        }

        // Skip extra per-address delay to speed up boot
        // delay(2); // Small delay between addresses for reliability
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
            Serial.println("⚠  No OLED display found at 0x3C or 0x3D");
        }
    }
    Serial.println("I2C scan complete\n");
}

// Helper: build a moving wave string like ".:::" with given width and phase
static void buildWave(char *buf, uint8_t width, uint8_t phase)
{
    for (uint8_t i = 0; i < width; i++)
        buf[i] = ':';
    buf[phase % width] = '.';
    buf[width] = '\0';
}

// Startup marquee: big scrolling "SKY FORGE" text (size 4)
static void showStartupMarquee()
{
    if (!displayAvailable)
        return;

    // Compute scale so both "SKY" (top-left) and "FORGE" (bottom-right) fit
    const int baseCharW = 6;
    const int baseCharH = 8;
    const char *wordTop = "SKY";
    const char *wordBottom = "FORGE";
    const int lenTop = 3;
    const int lenBottom = 5;

    // Avoid any wrapped artifacts during animation
    display.setTextWrap(false);

    int scale = 4; // prefer 4 if it fits
    auto fitsAtScale = [&](int sc)
    {
        int charW = baseCharW * sc;
        int charH = baseCharH * sc;
        int topW = lenTop * charW;
        int botW = lenBottom * charW;
        bool horizOK = (topW <= SCREEN_WIDTH) && (botW <= SCREEN_WIDTH);
        bool vertOK = (2 * charH) <= SCREEN_HEIGHT; // two rows (top and bottom)
        return horizOK && vertOK;
    };

    if (!fitsAtScale(scale))
        scale = 3; // fallback to 3 if 4 doesn't fit

    const int charW = baseCharW * scale;
    const int charH = baseCharH * scale;

    // Final anchored positions
    const int skyFinalX = 0;
    const int skyFinalY = 0;
    const int topW = lenTop * charW;
    const int forgeW = lenBottom * charW;
    const int forgeH = charH;
    const int forgeFinalX = SCREEN_WIDTH - forgeW;
    const int forgeFinalY = SCREEN_HEIGHT - forgeH;

    // Start positions (off-screen)
    // SKY: slide in horizontally from the left at the top row
    const int skyStartX = -topW;
    const int skyStartY = skyFinalY;
    // FORGE: slide in horizontally from the right at the bottom row
    const int forgeStartX = SCREEN_WIDTH;
    const int forgeStartY = forgeFinalY;

    // Animation frames based on max distance either word must travel
    int skyDx = abs(skyStartX - skyFinalX);
    int skyDy = abs(skyStartY - skyFinalY);
    int forgeDx = abs(forgeStartX - forgeFinalX);
    int forgeDy = abs(forgeStartY - forgeFinalY);
    int maxDist = max(max(skyDx, skyDy), max(forgeDx, forgeDy));
    // Step multiple pixels per frame to cut total frames significantly
    int stepPx = max(8, charW / 2);
    int frames = max(1, (maxDist + stepPx - 1) / stepPx);

    // Time-budget the animation to last ~1.5 seconds total
    const unsigned long targetAnimMs = 1500UL;
    unsigned long perFrameBudget = max(1UL, targetAnimMs / (unsigned long)max(1, frames));
    unsigned long nextDeadline = millis();

    for (int i = 0; i <= frames; ++i)
    {
        int skyX = skyStartX + (skyFinalX - skyStartX) * i / frames;
        int skyY = skyStartY + (skyFinalY - skyStartY) * i / frames;

        int forgeX = forgeStartX + (forgeFinalX - forgeStartX) * i / frames;
        int forgeY = forgeStartY + (forgeFinalY - forgeStartY) * i / frames;

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(scale);

        // Draw SKY at its current position
        display.setCursor(skyX, skyY);
        display.print(wordTop);

        // Draw FORGE at its current position
        display.setCursor(forgeX, forgeY);
        display.print(wordBottom);

        display.display();

        // Pace to meet the per-frame budget, absorbing drawing time
        unsigned long now = millis();
        unsigned long deadline = nextDeadline + perFrameBudget;
        if (now < deadline)
        {
            delay(deadline - now);
        }
        nextDeadline = deadline;
    }

    // Hold both words in place for 1 second
    {
        unsigned long holdStart = millis();
        while (millis() - holdStart < 1000)
        {
            display.clearDisplay();
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(scale);

            display.setCursor(skyFinalX, skyFinalY);
            display.print(wordTop);

            display.setCursor(forgeFinalX, forgeFinalY);
            display.print(wordBottom);

            display.display();
            delay(10);
        }
    }

    // After the SKY/FORGE hold, proceed directly to main screen
    return;
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
            Serial.println("  OLED Display temporarily disabled due to I2C errors");
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
        display.setTextColor(SSD1306_WHITE);
        // We'll control positions manually to add spacing and adjust sizes

        // --- Animated waves and status determination ---
        static uint8_t phase4 = 0; // for 4-char waves (WiFi/NRF)
        static uint8_t phase3 = 0; // retained (unused for Recv/Up now)
        phase4 = (phase4 + 1) % 4;
        phase3 = (phase3 + 1) % 3;

        char wave4[5]; // e.g., ".:::" for WiFi/NRF
        char wave3[4]; // legacy 3-char wave (kept for compatibility if needed)
        buildWave(wave4, 4, phase4);
        buildWave(wave3, 3, phase3);

        // Custom patterns for Recv (6 frames) and Up (7 frames)
        static uint8_t phaseRecv = 0;
        static uint8_t phaseUp = 0;
        phaseRecv = (phaseRecv + 1) % 5;
        phaseUp = (phaseUp + 1) % 6;

        char recvPat[4];
        char upPat[4];
        // Recv sequence: ":::", ":: ", ":  ", "  :", " ::"
        switch (phaseRecv)
        {
        case 0:
            strncpy(recvPat, ":::", 4);
            break;
        case 1:
            strncpy(recvPat, ":: ", 4);
            break;
        case 2:
            strncpy(recvPat, ":  ", 4);
            break;
        case 3:
            strncpy(recvPat, "  :", 4);
            break;
        case 4:
            strncpy(recvPat, " ::", 4);
            break;
        }
        // Up sequence: ":::", " ::", "  :", "   ", ":", ":: "
        switch (phaseUp)
        {
        case 0:
            strncpy(upPat, ":::", 4);
            break;
        case 1:
            strncpy(upPat, " ::", 4);
            break;
        case 2:
            strncpy(upPat, "  :", 4);
            break;
        case 3:
            strncpy(upPat, "   ", 4);
            break;
        case 4:
            strncpy(upPat, ":", 4);
            break;
        case 5:
            strncpy(upPat, ":: ", 4);
            break;
        }

        // Track recent NRF/telemetry activity using existing counters
        static int lastSuccessCount = -1;
        static unsigned long lastRFActivity = 0;
        static int lastTelemetryCountSeen = -1;
        static unsigned long lastTelemetryActivity = 0;

        unsigned long now = millis();
        if (successCount != lastSuccessCount)
        {
            lastSuccessCount = successCount;
            lastRFActivity = now;
        }
        if (telemetryCount != lastTelemetryCountSeen)
        {
            lastTelemetryCountSeen = telemetryCount;
            lastTelemetryActivity = now;
        }

        bool wifiOk = (WiFi.status() == WL_CONNECTED);       // use live WiFi status
        bool nrfOk = (now - lastRFActivity) <= 3000;         // recent successful radio write
        bool recvOk = (now - lastTelemetryActivity) <= 3000; // recent telemetry
        bool upOk = (wifiConnected && firebaseReady);        // upload readiness (no timestamp available)

        // Read live joystick values for display (independent of arm state)
        int joy1_x_disp = analogRead(JOY1_X_PIN);
        int joy2_x_disp = analogRead(JOY2_X_PIN);
        int joy2_y_disp = analogRead(JOY2_Y_PIN);
        int16_t yawDisp = map(joy1_x_disp, 0, 4095, -3000, 3000);
        int16_t rollDisp = map(joy2_x_disp, 0, 4095, -3000, 3000);
        // Inverted pitch display mapping (user request)
        int16_t pitchDisp = map(joy2_y_disp, 0, 4095, 3000, -3000);
        // Apply display deadzones (yaw uses its wider dedicated deadzone)
        yawDisp = (abs(yawDisp) < YAW_DEADZONE) ? 0 : yawDisp;
        rollDisp = (abs(rollDisp) < CONTROL_DEADZONE) ? 0 : rollDisp;
        pitchDisp = (abs(pitchDisp) < CONTROL_DEADZONE) ? 0 : pitchDisp;

        // Manual layout with spacing and slightly larger header
        int y = 0;

        // Helper lambdas for centering the '|' at screen middle
        const int xCenter = SCREEN_WIDTH / 2;
        auto charWidth = [](int scale)
        { return 6 * scale; };
        auto strLen = [](const char *s)
        {
            int n = 0;
            while (*s++)
                n++;
            return n;
        };

        // --- Line 1: Arm | Mode (larger text) ---
        {
            const int scale = 1;
            const int cW = charWidth(scale);
            const char *leftText = isArmed ? "Armed" : "Disarmed";
            const char *rightText = isStabilizedMode ? "STABILIZED" : "Manual";

            int leftW = strLen(leftText) * cW;
            int basePipeLeftX = xCenter - (cW / 2);
            int minPipeX = leftW + cW; // ensure space between left and pipe
            int pipeLeftX = (basePipeLeftX < minPipeX) ? minPipeX : basePipeLeftX;

            int leftStartX = 0; // anchor left label to the left edge

            display.setTextSize(scale);
            display.setCursor(leftStartX, y);
            display.print(leftText);
            display.print(' ');
            display.setCursor(pipeLeftX, y);
            display.print('|');
            display.print(' ');
            display.print(rightText);
        }

        y += 14; // keep existing spacing

        // --- Line 2: WIFI and NRF ---
        {
            const int scale = 1;
            const int cW = charWidth(scale);
            int basePipeLeftX = xCenter - (cW / 2);

            // Left: "WIFI " + (wave4 or '!')
            const char *lPrefix = "WIFI ";
            int lLen = 5 + (wifiOk ? 4 : 1);
            int leftW = lLen * cW;
            int minPipeX = leftW + cW;
            int pipeLeftX = (basePipeLeftX < minPipeX) ? minPipeX : basePipeLeftX;
            int leftStartX = 0; // anchor left label to the left edge

            display.setTextSize(scale);
            display.setCursor(leftStartX, y);
            display.print(lPrefix);
            if (wifiOk)
                display.print(wave4);
            else
                display.print("!");
            display.print(' ');
            display.setCursor(pipeLeftX, y);
            display.print('|');
            display.print(' ');
            display.print("NRF ");
            if (nrfOk)
                display.print(wave4);
            else
                display.print("!");
        }

        y += 12; // keep existing spacing

        // --- Line 3: Recv and Up ---
        {
            const int scale = 1;
            const int cW = charWidth(scale);
            int basePipeLeftX = xCenter - (cW / 2);

            // Left: "Recv " + (wave3 or '!')
            const char *lPrefix = "Recv ";
            int lLen = 5 + (recvOk ? 3 : 1);
            int leftW = lLen * cW;
            int minPipeX = leftW + cW;
            int pipeLeftX = (basePipeLeftX < minPipeX) ? minPipeX : basePipeLeftX;
            int leftStartX = 0; // anchor left label to the left edge

            display.setTextSize(scale);
            display.setCursor(leftStartX, y);
            display.print(lPrefix);
            if (recvOk)
                display.print(recvPat);
            else
                display.print("!");
            display.print(' ');
            display.setCursor(pipeLeftX, y);
            display.print('|');
            display.print(' ');
            display.print("Up ");
            if (upOk)
                display.print(upPat);
            else
                display.print("!");
        }

        y += 16; // add spacing between lines

        // --- Line 4: Throttle and Pitch (swapped placement) ---
        // Left: T at x=0; Right: Y aligned with right-hand column
        display.setTextSize(1); // keep size 1 here to fit width reliably
        display.setCursor(0, y);
        display.printf("T:%4d", controlData.throttle);

        // Compute right-hand column X (aligned with lines 2/3 right content)
        {
            const int scaleR = 1;
            const int cWR = charWidth(scaleR);
            int basePipeLeftX = xCenter - (cWR / 2);
            // Consider both line 2 (WIFI...) and line 3 (Recv...) left widths
            int minPipeX2 = (5 + (wifiOk ? 4 : 1)) * cWR + cWR; // "WIFI " + wave4/!
            int minPipeX3 = (5 + (recvOk ? 3 : 1)) * cWR + cWR; // "Recv " + wave3/!
            int pipeL2 = (basePipeLeftX < minPipeX2) ? minPipeX2 : basePipeLeftX;
            int pipeL3 = (basePipeLeftX < minPipeX3) ? minPipeX3 : basePipeLeftX;
            int pipeLX = (pipeL2 > pipeL3) ? pipeL2 : pipeL3;
            int rightColX = pipeLX + 2 * cWR; // after '|' and a space

            display.setCursor(rightColX, y);
            display.printf("P:%4d", pitchDisp);
        }

        y += 12; // add spacing between lines

        // --- Line 5: Yaw and Roll (swapped placement) ---
        // Left: Y at x=0; Right: R aligned with right-hand column
        display.setCursor(0, y);
        display.setTextSize(1);
        display.printf("Y:%4d", yawDisp);

        {
            const int scaleR = 1;
            const int cWR = charWidth(scaleR);
            int basePipeLeftX = xCenter - (cWR / 2);
            int minPipeX2 = (5 + (wifiOk ? 4 : 1)) * cWR + cWR;
            int minPipeX3 = (5 + (recvOk ? 3 : 1)) * cWR + cWR;
            int pipeL2 = (basePipeLeftX < minPipeX2) ? minPipeX2 : basePipeLeftX;
            int pipeL3 = (basePipeLeftX < minPipeX3) ? minPipeX3 : basePipeLeftX;
            int pipeLX = (pipeL2 > pipeL3) ? pipeL2 : pipeL3;
            int rightColX = pipeLX + 2 * cWR;

            display.setCursor(rightColX, y);
            display.printf("R:%4d", rollDisp);
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
                Serial.printf("⚠  OLED I2C error (attempt %d/%d, error count: %d, I2C error: %d)\n",
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
    // Shorter serial warm-up for faster boot
    delay(200);

    Serial.println("=== Complete Remote Controller - Stable Version ===");
    Serial.println("Compatible with droneFreeRTOS.ino");

// Initialize I2C only if OLED is enabled
#if OLED_ENABLED
    Serial.println("🔧 Initializing I2C for OLED display...");
    Wire.begin(21, 22);    // SDA=21, SCL=22 (ESP32 default)
    Wire.setClock(400000); // Speed up I2C to 400kHz for faster display updates
    delay(200);            // Give I2C time to stabilize

    Serial.println("📍 I2C initialized - SDA:GPIO21, SCL:GPIO22, Clock:400kHz");

    // Scan for I2C devices first
    scanI2CDevices();

    // Initialize OLED Display with comprehensive error handling
    Serial.println("🖥  Initializing OLED Display...");
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
        Serial.println("⚠  Continuing without display - drone control will work normally");
        displayAvailable = false;
    }
    else
    {
        Serial.println("✅ OLED display ready");
        // Ensure the panel starts from a clean state and no wrapped text
        display.clearDisplay();
        display.display();
        delay(20);
        display.setTextWrap(false);

        // Start animation immediately on power-up
        Serial.println("🎉 Startup animation...");
        showStartupMarquee();
        Serial.println("✅ Startup animation done");

        // Draw initial values screen right away (before WiFi/Firebase)
        updateOLEDDisplay();
    }
#else
    Serial.println("⚠  OLED Display disabled in configuration");
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
    radio.setPALevel(RF24_PA_HIGH); // High power for range
    // Switched to 1MBPS for lower latency (must match drone configuration)
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(110); // Match drone channel, away from Wi-Fi
    radio.setAutoAck(true);
    // Shorter retry window to cap worst-case latency (delay:3 => ~750us; count:5 attempts)
    radio.setRetries(3, 5);
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

    // Startup animation already shown immediately after OLED init

    Serial.print("Starting low-latency control transmission (50Hz) with virtual throttle mode");
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
    Serial.print(", Data Rate: RF24_1MBPS");
    Serial.print(", Channel: ");
    Serial.print(radio.getChannel());
    Serial.print(", CRC: 16-bit");
    Serial.println(", Address Width: 5 bytes");
}

void loop()
{
    unsigned long currentTime = millis();

    // Send control data at 50Hz (20ms intervals) for low latency
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

    // Upload to Firebase at specified interval ONLY when telemetry is available
    if (FIREBASE_ENABLED && firebaseReady && telemetryReceived &&
        (currentTime - lastFirebaseUpload >= FIREBASE_UPLOAD_INTERVAL))
    {
        uploadTelemetryToFirebase();
        lastFirebaseUpload = currentTime;
    }

    // Check WiFi/Firebase connection every 30 seconds
    if (FIREBASE_ENABLED)
    {
        static unsigned long lastConnCheck = 0;
        if (currentTime - lastConnCheck >= 30000)
        {
            lastConnCheck = currentTime;
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
            else if (!firebaseReady)
            {
                // WiFi is up but Firebase not ready, try to re-init
                Serial.println("Firebase not ready, attempting re-initialization...");
                initializeFirebase();
            }
            else
            {
                wifiConnected = true;
            }
        }
    }

    // Removed fixed delay to reduce control latency; loop paced by 20ms send interval

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
        // Inverted pitch control mapping (user request)
        int16_t pitchInput = map(joy2_y, 0, 4095, 3000, -3000);

        // Apply deadzone filtering (yaw uses its own wider deadzone)
        controlData.yaw = (abs(yawInput) < YAW_DEADZONE) ? 0 : yawInput;
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
    const int maxAttempts = 5; // Try a bit harder for reliability

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

        // Drain all available ACK payloads to avoid stale data and FIFO buildup
        while (radio.isAckPayloadAvailable())
        {
            uint8_t len = radio.getDynamicPayloadSize();
            if (len >= sizeof(TelemetryPacket))
            {
                radio.read(&telemetryData, sizeof(telemetryData));
                telemetryReceived = true;
                telemetryCount++;
            }
            else if (len > 0)
            {
                // Read and discard unexpected short payloads
                uint8_t scratch[32];
                radio.read(scratch, len);
            }
            else
            {
                // No size reported; break to avoid tight loop
                break;
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
        Serial.print(" STABILIZED");
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
    Serial.print("🌡  Environment: ");
    Serial.print("Temp:");
    Serial.print(telemetryData.temperature / 100.0, 1);
    Serial.print("°C | Press:");
    Serial.print(telemetryData.pressureX10 / 10.0, 1);
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
        Serial.print(telemetryData.latitudeE7 / 10000000.0, 7);
        Serial.print(",");
        Serial.print(telemetryData.longitudeE7 / 10000000.0, 7);
        Serial.print(")");
    }
    else
    {
        Serial.print("(No Fix)");
    }
    Serial.println();

    // === AIR QUALITY LINE ===
    Serial.print("🌬  Air Quality: ");
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
    Serial.print("🛡  Safety: ");
    Serial.print(isArmed ? "🔓ARMED" : "🔒DISARMED");
    Serial.print(" | Flight Mode: ");
    Serial.print(isStabilizedMode ? " STABILIZED" : "🎯MANUAL");
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

        // Configure time for Firebase
        configTime(0, 0, "pool.ntp.org");
        // Wait briefly for time to sync (needed for TLS)
        time_t now = time(nullptr);
        uint32_t waitMs = 0;
        while (now < 1700000000 && waitMs < 5000) // ~2023-11-14 epoch threshold
        {
            delay(200);
            waitMs += 200;
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

    config.database_url = FIREBASE_DB_URL;
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

        if (Firebase.RTDB.setJSON(&firebaseData, "/connection_test", &testJson))
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
    if (!wifiConnected || !firebaseReady)
    {
        return; // Need WiFi + Firebase
    }

    // Create JSON payload with real telemetry only
    FirebaseJson json;
    FirebaseJson ctrl;
    unsigned long timestamp = time(nullptr);

    json.set("timestamp", (int)timestamp);
    ctrl.set("rssi", WiFi.RSSI());
    ctrl.set("armed", isArmed);
    ctrl.set("mode", isStabilizedMode ? "STABILIZED" : "MANUAL");
    ctrl.set("success", successCount);
    ctrl.set("failed", failCount);
    json.set("controller", ctrl);

    // We only call this when telemetryReceived is true
    json.set("temperature", telemetryData.temperature / 100.0);
    json.set("humidity", telemetryData.humidity);
    json.set("pressure", telemetryData.pressureX10 / 10.0);
    json.set("altitude", telemetryData.altitude / 100.0);
    json.set("battery", telemetryData.battery);
    json.set("lux", telemetryData.lux);
    json.set("uvIndex", telemetryData.uvIndex / 100.0);
    json.set("eCO2", telemetryData.eCO2);
    json.set("TVOC", telemetryData.TVOC);
    json.set("gps_satellites", telemetryData.satellites);
    json.set("gps_latitude", telemetryData.latitudeE7 / 10000000.0);
    json.set("gps_longitude", telemetryData.longitudeE7 / 10000000.0);
    json.set("status", telemetryData.status);

    String dataPath = "/telemetry/" + String(timestamp);

    if (Firebase.RTDB.setJSON(&firebaseData, dataPath.c_str(), &json))
    {
        static unsigned long lastSuccessMessage = 0;
        if (millis() - lastSuccessMessage > 10000)
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
            Serial.println("   FLIGHT MODE: STABILIZED - Auto-leveling enabled");
        }
        else // Toggle 2 turned OFF
        {
            isStabilizedMode = false;
            Serial.println("🎯 FLIGHT MODE: MANUAL - Direct control, no auto-leveling");
        }
    }
}