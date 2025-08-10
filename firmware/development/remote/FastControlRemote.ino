/*
 * Complete Remote Controller - Proven Control
 * Reliable control at 200ms intervals (5Hz) for stable drone communication
 */

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

// Firebase configuration
#define FIREBASE_HOST "skyforge-4606b-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "HBgETEJwFCsLswgXH2SuzYupJPWhJqWnJdfkFvGO"
#define WIFI_SSID "Dialog 4G"
#define WIFI_PASSWORD "0N7NT00ANTQ"

// Upload frequency configuration (in milliseconds)
#define FIREBASE_UPLOAD_INTERVAL 1000 // 10 seconds (change this value for testing)
// Common testing intervals:
// 2000   = 2 seconds (fast testing)
// 5000   = 5 seconds (medium testing)
// 10000  = 10 seconds (normal)
// 30000  = 30 seconds (slow/production)
// 60000  = 1 minute (very slow)

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5

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
bool telemetryReceived = false;
bool firebaseReady = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("=== Complete Remote Controller - Fast Control ===");

    // Initialize pins
    pinMode(JOY1_X_PIN, INPUT);
    pinMode(JOY1_Y_PIN, INPUT);
    pinMode(JOY1_BTN_PIN, INPUT_PULLUP); // Joystick 1 button (not functioning well)
    pinMode(JOY2_X_PIN, INPUT);
    pinMode(JOY2_Y_PIN, INPUT);
    pinMode(JOY2_BTN_PIN, INPUT_PULLUP); // Joystick 2 button
    pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP);

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

    Serial.println("Radio configured successfully!");

    // Initialize WiFi and Firebase
    initializeWiFi();
    initializeFirebase();

    Serial.print("Starting PROVEN control transmission (5Hz) with telemetry and cloud upload every ");
    Serial.print(FIREBASE_UPLOAD_INTERVAL / 1000);
    Serial.println(" seconds...");

    // Print radio configuration for debugging
    Serial.println("Radio Configuration:");
    Serial.print("  Power Level: RF24_PA_HIGH");
    Serial.print(", Data Rate: RF24_250KBPS");
    Serial.print(", Channel: ");
    Serial.print(radio.getChannel());
    Serial.print(", CRC: 16-bit");
    Serial.println(", Address Width: 5 bytes");
}

void printRadioDiagnostics()
{
    // Print radio status for troubleshooting
    Serial.println("=== Radio Diagnostics ===");
    Serial.print("Is chip connected: ");
    Serial.println(radio.isChipConnected() ? "YES" : "NO");
    Serial.print("Is P variant: ");
    Serial.println(radio.isPVariant() ? "YES" : "NO");
    Serial.print("Data rate: ");
    rf24_datarate_e datarate = radio.getDataRate();
    Serial.println(datarate == RF24_250KBPS ? "250KBPS" : datarate == RF24_1MBPS ? "1MBPS"
                                                                                 : "2MBPS");
    Serial.print("Power level: ");
    uint8_t power = radio.getPALevel();
    Serial.println(power == RF24_PA_MIN ? "MIN" : power == RF24_PA_LOW ? "LOW"
                                              : power == RF24_PA_HIGH  ? "HIGH"
                                                                       : "MAX");
    Serial.print("Channel: ");
    Serial.println(radio.getChannel());
    Serial.println("========================");
}

void loop()
{
    // Check WiFi connection and reconnect if needed
    static unsigned long lastWiFiCheck = 0;
    if (millis() - lastWiFiCheck > 30000) // Check every 30 seconds
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi disconnected, attempting reconnection...");
            initializeWiFi();
            if (WiFi.status() == WL_CONNECTED)
            {
                initializeFirebase();
            }
        }
        lastWiFiCheck = millis();
    }

    // Send control data every 20ms (50Hz) for low latency
    if (millis() - lastControlSend >= 20)
    {
        readJoystickInputs();
        sendControlData();
        lastControlSend = millis();
    }

    // Upload to Firebase at configurable interval
    if (millis() - lastFirebaseUpload > FIREBASE_UPLOAD_INTERVAL && telemetryReceived)
    {
        uploadTelemetryToFirebase();
        lastFirebaseUpload = millis();
    }

    delay(10); // Reduced delay for faster response
}

void sendControlData()
{
    bool result = false;
    int attempts = 0;
    const int maxAttempts = 3; // Restore proven working retry count

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

    static int packetNumber = 1;
    static int successCount = 0, failCount = 0, telemetryCount = 0;
    static unsigned long lastDetailedPrint = 0;

    if (result)
    {
        successCount++;

        // Check for ACK payload with telemetry
        if (radio.isAckPayloadAvailable())
        {
            uint8_t len = radio.getDynamicPayloadSize();
            if (len == sizeof(telemetryData))
            {
                radio.read(&telemetryData, sizeof(telemetryData));
                telemetryReceived = true;
                telemetryCount++;
            }
        }

        // Print detailed info only every 2 seconds (to avoid spam)
        if (millis() - lastDetailedPrint > 2000 && telemetryReceived)
        {
            Serial.print("Packet #");
            Serial.print(packetNumber);
            Serial.print(" SUCCESS");
            if (attempts > 1)
            {
                Serial.print(" (retry #");
                Serial.print(attempts);
                Serial.print(")");
            }
            Serial.print(" - T:");
            Serial.print(controlData.throttle);
            Serial.print(" R:");
            Serial.print(controlData.roll);
            Serial.print(" P:");
            Serial.print(controlData.pitch);
            Serial.print(" Y:");
            Serial.print(controlData.yaw);

            // Show button states when pressed
            if (controlData.joy1_btn == 0)
                Serial.print(" [JOY1-BTN]");
            if (controlData.joy2_btn == 0)
                Serial.print(" [JOY2-BTN]");

            // Show toggle switch states
            if (controlData.toggle1 == 1)
                Serial.print(" [SW1-ON]");
            if (controlData.toggle2 == 1)
                Serial.print(" [SW2-ON]");

            Serial.print(" | Telemetry - Temp:");
            Serial.print(telemetryData.temperature / 100.0);
            Serial.print("°C Press:");
            Serial.print(telemetryData.pressure / 10.0);
            Serial.print("hPa Alt:");
            Serial.print(telemetryData.altitude / 100.0, 2); // Display altitude in meters with 2 decimals
            Serial.print("m Hum:");
            Serial.print(telemetryData.humidity);
            Serial.print("% GPS:");
            Serial.print(telemetryData.satellites);
            Serial.print(" sats Lat:");
            Serial.print(telemetryData.latitude / 100.0, 2);
            Serial.print(" Lng:");
            Serial.print(telemetryData.longitude / 100.0, 2);
            Serial.print(" Batt:");
            Serial.print(telemetryData.battery);
            Serial.print("mV Lux:");
            Serial.print(telemetryData.lux);
            Serial.print("lx UV:");
            Serial.print(telemetryData.uvIndex / 100.0);
            Serial.print(" CO2:");
            Serial.print(telemetryData.eCO2);
            Serial.print("ppm TVOC:");
            Serial.print(telemetryData.TVOC);
            Serial.println("ppb");
            lastDetailedPrint = millis();
        }
    }
    else
    {
        failCount++;
        // Print failures with attempt count for debugging
        Serial.print("Packet #");
        Serial.print(packetNumber);
        Serial.print(" FAILED after ");
        Serial.print(attempts);
        Serial.println(" attempts");
    }

    packetNumber++;

    // Print status every 25 packets (5 seconds at 5Hz)
    if (packetNumber % 25 == 0)
    {
        Serial.print("Control Status - Success: ");
        Serial.print(successCount);
        Serial.print(", Failed: ");
        Serial.print(failCount);
        Serial.print(", Telemetry: ");
        Serial.print(telemetryCount);
        Serial.print(", Success rate: ");
        if (successCount + failCount > 0)
        {
            int successRate = (successCount * 100) / (successCount + failCount);
            Serial.print(successRate);
            Serial.print("%");

            // Print diagnostics if success rate is low
            if (successRate < 80)
            {
                Serial.println();
                printRadioDiagnostics();
            }
        }
        Serial.print(", Rate: 5Hz, Firebase: ");
        Serial.print(firebaseReady ? "Connected" : "Disconnected");
        Serial.print(" (Upload every ");
        Serial.print(FIREBASE_UPLOAD_INTERVAL / 1000);
        Serial.println("s)");
        successCount = 0;
        failCount = 0;
        telemetryCount = 0;
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
    controlData.toggle1 = !digitalRead(TOGGLE_SW1_PIN); // Invert: 0=off, 1=on
    controlData.toggle2 = !digitalRead(TOGGLE_SW2_PIN); // Invert: 0=off, 1=on

    // Map analog readings to control values
    controlData.throttle = map(joy1_y, 0, 4095, -1000, 1000);
    controlData.yaw = map(joy1_x, 0, 4095, -1000, 1000);
    controlData.roll = map(joy2_x, 0, 4095, -1000, 1000);
    controlData.pitch = map(joy2_y, 0, 4095, -1000, 1000);
}

void initializeWiFi()
{
    Serial.println("Connecting to WiFi...");

    // Disconnect any existing connection first
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
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        configTime(0, 0, "pool.ntp.org");
        // Wait for NTP time (TLS requires correct time)
        time_t now = time(nullptr);
        uint32_t waited = 0;
        while (now < 1700000000 && waited < 5000)
        {
            delay(200);
            waited += 200;
            now = time(nullptr);
        }
        Serial.print("Time synced: ");
        Serial.println((unsigned long)now);
    }
    else
    {
        Serial.println("\nWiFi connection failed!");
    }
}

void initializeFirebase()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    // Add timeout and SSL settings for better reliability
    config.timeout.serverResponse = 10 * 1000;   // 10 seconds timeout
    config.timeout.socketConnection = 10 * 1000; // 10 seconds socket timeout

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    if (Firebase.ready())
    {
        Serial.println("Firebase connected!");
        firebaseReady = true;
    }
    else
    {
        Serial.println("Firebase connection failed, will retry later");
        firebaseReady = false;
    }
}

void uploadTelemetryToFirebase()
{
    if (!telemetryReceived)
        return;

    // Check Firebase connection and try to reconnect if needed
    if (!Firebase.ready())
    {
        static unsigned long lastReconnectAttempt = 0;
        if (millis() - lastReconnectAttempt > 30000) // Try reconnect every 30 seconds
        {
            Serial.println("Firebase disconnected, attempting reconnection...");
            initializeFirebase();
            lastReconnectAttempt = millis();
        }
        return;
    }

    // Create JSON payload - ONLY sensor data from drone, no control data
    FirebaseJson json;
    unsigned long timestamp = time(nullptr);
    json.set("timestamp", (int)timestamp);
    json.set("temperature", telemetryData.temperature / 100.0);
    json.set("humidity", telemetryData.humidity);
    json.set("pressure", telemetryData.pressure / 10.0);
    json.set("altitude", telemetryData.altitude / 100.0); // Store altitude in meters with 2 decimal precision
    json.set("battery", telemetryData.battery);
    json.set("lux", telemetryData.lux);
    json.set("uvIndex", telemetryData.uvIndex / 100.0);
    json.set("eCO2", telemetryData.eCO2);
    json.set("TVOC", telemetryData.TVOC);
    json.set("gps_satellites", telemetryData.satellites);
    json.set("gps_latitude", telemetryData.latitude / 100.0);
    json.set("gps_longitude", telemetryData.longitude / 100.0);
    json.set("status", telemetryData.status);
    // Note: Control data and button states are NOT uploaded to cloud

    // Store data with timestamp - single node, latest record is most recent
    String dataPath = "/telemetry/" + String(timestamp);

    bool uploadSuccess = Firebase.setJSON(firebaseData, dataPath, json);

    // Simple upload result reporting
    if (uploadSuccess)
    {
        static unsigned long lastSuccessTime = millis();
        if (millis() - lastSuccessTime > 5000) // Only print success every 5 seconds to reduce spam
        {
            Serial.println("✓ Firebase upload successful");
            lastSuccessTime = millis();
        }
        firebaseReady = true; // Confirm connection is working
    }
    else
    {
        Serial.print("✗ Firebase upload failed: ");
        Serial.println(firebaseData.errorReason());

        // If it's an SSL error, mark Firebase as not ready for reconnection attempt
        if (firebaseData.errorReason().indexOf("ssl") >= 0 ||
            firebaseData.errorReason().indexOf("SSL") >= 0 ||
            firebaseData.errorReason().indexOf("engine") >= 0)
        {
            Serial.println("SSL error detected, will attempt reconnection");
            firebaseReady = false;
        }
    }
}