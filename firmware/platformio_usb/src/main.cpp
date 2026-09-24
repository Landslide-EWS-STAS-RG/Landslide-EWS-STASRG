#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "SparkFunLSM6DS3.h"

// ===============================
// LSM6DS3
// ===============================
LSM6DS3 myIMU;

// ===============================
// GPS
// ===============================
TinyGPSPlus gps;
HardwareSerial GPSserial(2);   // UART2

#define GPS_RX 16              // ESP32 RX2 menerima dari TX GPS
#define GPS_TX 17              // ESP32 TX2 ke RX GPS
#define GPS_BAUD 9600          // GY-GPSV3-NEO umum memakai 9600 bps

void displayLocationInfo()
{
    Serial.println("-------------------------------------");
    Serial.println("Location Info:");

    Serial.print("Latitude:  ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" ");
    Serial.println(gps.location.rawLat().negative ? "S" : "N");

    Serial.print("Longitude: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print(" ");
    Serial.println(gps.location.rawLng().negative ? "W" : "E");

    Serial.print("Fix Quality: ");
    Serial.println(gps.location.isValid() ? "Valid" : "Invalid");

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.print("Altitude:   ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");

    Serial.print("Speed:      ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");

    Serial.print("Course:     ");
    Serial.print(gps.course.deg());
    Serial.println("°");

    Serial.print("Date:       ");
    if (gps.date.isValid())
    {
        Serial.printf("%02d/%02d/%04d\n", gps.date.day(), gps.date.month(), gps.date.year());
    }
    else
    {
        Serial.println("Invalid");
    }

    Serial.print("Time (UTC): ");
    if (gps.time.isValid())
    {
        Serial.printf("%02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    else
    {
        Serial.println("Invalid");
    }

    Serial.println("-------------------------------------");
}

// ===============================
// Piezo Sensor
// ===============================
const int piezoPin = 34;       // GPIO34 (input-only, cocok untuk analog)
const int ledPin   = 2;        // LED onboard ESP32 Dev Module
const int piezoThreshold = 100;
const int piezoSamples = 8;
unsigned long lastPiezoPrint = 0;

int readPiezoFiltered()
{
    long sum = 0;
    for (int i = 0; i < piezoSamples; i++)
    {
        sum += analogRead(piezoPin);
        delay(2);
    }

    int avg = sum / piezoSamples;

    // Batasi noise ekstrem yang sering muncul dari piezo mentah
    if (avg < 20) return 0;
    if (avg > 4000) return 4095;

    return avg;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(ledPin, OUTPUT);
    pinMode(piezoPin, INPUT);
    analogSetPinAttenuation(piezoPin, ADC_11db);

    // ===============================
    // Start I2C for LSM6DS3
    // SDA = GPIO21, SCL = GPIO22
    // ===============================
    Wire.begin(21, 22);

    // ===============================
    // Start IMU
    // ===============================
    if (myIMU.begin() != 0)
    {
        Serial.println("LSM6DS3 initialization failed!");
        while (1);
    }
    Serial.println("LSM6DS3 initialized.");

    // ===============================
    // Start GPS
    // ===============================
    GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("GPS initialized.");
}

void loop()
{
    // ===============================
    // Read GPS continuously
    // ===============================
    while (GPSserial.available())
    {
        char c = static_cast<char>(GPSserial.read());
        gps.encode(c);

        if (gps.location.isUpdated())
        {
            displayLocationInfo();
        }
    }

    // ===============================
    // Read IMU
    // ===============================
    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();

    float gx = myIMU.readFloatGyroX();
    float gy = myIMU.readFloatGyroY();
    float gz = myIMU.readFloatGyroZ();

    // ===============================
    // Read Piezo (realtime)
    // ===============================
    int piezo = readPiezoFiltered();

    // Nyalakan LED kalau piezo di atas threshold yang lebih sensitif
    digitalWrite(ledPin, piezo > piezoThreshold ? HIGH : LOW);

    if (millis() - lastPiezoPrint >= 200)
    {
        lastPiezoPrint = millis();
        Serial.println("--------------------------------");
        Serial.print("Piezo realtime: ");
        Serial.println(piezo);
    }

    // ===============================
    // Print IMU
    // ===============================
    Serial.print("Accel (g): ");
    Serial.print(ax, 4);
    Serial.print(", ");
    Serial.print(ay, 4);
    Serial.print(", ");
    Serial.println(az, 4);

    Serial.print("Gyro (dps): ");
    Serial.print(gx, 4);
    Serial.print(", ");
    Serial.print(gy, 4);
    Serial.print(", ");
    Serial.println(gz, 4);

    // ===============================
    // Print GPS
    // ===============================
    if (!gps.location.isValid())
    {
        Serial.println("GPS Location: Waiting for fix...");
    }

    // Jika setelah 10 detik belum ada data GPS masuk
    if (millis() > 10000 && gps.charsProcessed() < 20)
    {
        Serial.println("GPS module not detected or no data received.");
    }

    delay(1000);
}