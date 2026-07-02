#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "SparkFunLSM6DS3.h"

// ===============================
// Backup of the original main program
// containing IMU + GPS + piezo logic
// ===============================

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

void setup()
{
    Serial.begin(115200);
    delay(1000);
bua
    pinMode(ledPin, OUTPUT);

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
        gps.encode(GPSserial.read());
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
    // Read Piezo
    // ===============================
    int piezo = analogRead(piezoPin);

    // Nyalakan LED kalau piezo di atas threshold
    digitalWrite(ledPin, piezo > 2000 ? HIGH : LOW);

    // ===============================
    // Print separator
    // ===============================
    Serial.println("================================");

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
    // Print Piezo
    // ===============================
    Serial.print("Piezo: ");
    Serial.println(piezo);

    // ===============================
    // Print GPS
    // ===============================
    if (gps.location.isValid())
    {
        displayLocationInfo();
    }
    else
    {
        Serial.println("GPS Location: Waiting for fix...");
    }

    // Jika setelah 5 detik belum ada karakter GPS masuk
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println("ERROR: GPS module not detected!");
    }

    delay(1000);
}
