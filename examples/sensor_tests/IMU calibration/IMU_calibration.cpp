#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Calibration configuration
const int SAMPLE_RATE_HZ = 200;                        // Sampling rate (Hz)
const int DURATION_SEC = 30;                           // Increased duration (seconds) for better averaging (was 15)
const int NUM_SAMPLES = SAMPLE_RATE_HZ * DURATION_SEC; // Total samples collected

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }
    Serial.println();
    Serial.println("=== IMU CALIBRATION TOOL ===");
    Serial.printf("Sampling %d seconds at %d Hz (%d samples)\r\n", DURATION_SEC, SAMPLE_RATE_HZ, NUM_SAMPLES);
    if (!mpu.begin())
    {
        Serial.println("MPU6050 init FAILED");
        while (true)
            delay(1000);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);

    Serial.println("Place drone FLAT and STILL. Starting in 3s...");
    delay(3000);

    double axSum = 0, aySum = 0, azSum = 0;
    double gxSum = 0, gySum = 0, gzSum = 0;
    int valid = 0;

    unsigned long interval = 1000UL / SAMPLE_RATE_HZ;
    unsigned long next = millis();

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp))
        {
            axSum += a.acceleration.x;
            aySum += a.acceleration.y;
            azSum += a.acceleration.z;
            gxSum += g.gyro.x;
            gySum += g.gyro.y;
            gzSum += g.gyro.z;
            valid++;
        }
        if (i % (SAMPLE_RATE_HZ * 3) == 0)
        {
            Serial.printf("Progress: %d/%d (%.0f%%)\r\n", i, NUM_SAMPLES, (i * 100.0) / NUM_SAMPLES);
        }
        next += interval;
        long wait = (long)next - (long)millis();
        if (wait > 0)
            delay(wait);
    }

    if (valid < NUM_SAMPLES * 0.5)
    {
        Serial.println("Too few valid samples. Retry.");
        return;
    }

    double axOff = axSum / valid;
    double ayOff = aySum / valid;
    double azOff = azSum / valid; // includes +gravity on Z (≈ +9.81 if sensor Z up)
    double gxOff = gxSum / valid;
    double gyOff = gySum / valid;
    double gzOff = gzSum / valid;

    // Compute mounting roll/pitch (same method as main firmware)
    double rollDeg = atan2(ayOff, sqrt(axOff * axOff + azOff * azOff)) * 180.0 / PI;
    double pitchDeg = atan2(-axOff, sqrt(ayOff * ayOff + azOff * azOff)) * 180.0 / PI;

    Serial.println();
    Serial.println("=== RESULTS ===");
    Serial.printf("Accel offsets (m/s^2): X=%.5f Y=%.5f Z=%.5f (raw Z includes gravity)\r\n", axOff, ayOff, azOff);
    Serial.printf("Gyro offsets (rad/s):  X=%.6f Y=%.6f Z=%.6f\r\n", gxOff, gyOff, gzOff);
    Serial.printf("Mounting angle (deg):  Roll=%.4f Pitch=%.4f\r\n", rollDeg, pitchDeg);

    Serial.println();
    Serial.println("Paste into flight_controller-v2.cpp or droneFreeRTOS.ino:");
    Serial.printf("const float IMU_ROLL_OFFSET_DEG = %.6ff;\r\n", rollDeg);
    Serial.printf("const float IMU_PITCH_OFFSET_DEG = %.6ff;\r\n", pitchDeg);
    Serial.printf("const float IMU_ACCEL_X_OFFSET = %.6ff;\r\n", axOff);
    Serial.printf("const float IMU_ACCEL_Y_OFFSET = %.6ff;\r\n", ayOff);
    Serial.printf("const float IMU_ACCEL_Z_OFFSET = %.6ff;\r\n", (azOff - 9.81)); // subtract gravity for stored offset
    Serial.printf("const float IMU_GYRO_X_OFFSET = %.6ff;\r\n", gxOff);
    Serial.printf("const float IMU_GYRO_Y_OFFSET = %.6ff;\r\n", gyOff);
    Serial.printf("const float IMU_GYRO_Z_OFFSET = %.6ff;\r\n", gzOff);
    Serial.println();
    Serial.println("Done.");
}

void loop() {}