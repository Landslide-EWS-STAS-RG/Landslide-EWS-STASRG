#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// GPS-only test sketch for GY-GPSV3-NEO on ESP32
// Pins:
// ESP32 TX2 (GPIO17) -> GPS RX
// ESP32 RX2 (GPIO16) -> GPS TX

TinyGPSPlus gps;
HardwareSerial GPSserial(2);

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

void setup() {
    Serial.begin(115200);
    delay(1000);

    GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

    Serial.println("GPS test started");
    Serial.println("Waiting for GPS data...");
}

void loop() {
    while (GPSserial.available()) {
        uint8_t c = GPSserial.read();
        Serial.print((char)c);
        gps.encode(c);
    }

    if (gps.location.isUpdated()) {
        Serial.println("-------------------------------------");
        Serial.print("Latitude:  ");
        Serial.print(gps.location.lat(), 6);
        Serial.println(gps.location.rawLat().negative ? " S" : " N");

        Serial.print("Longitude: ");
        Serial.print(gps.location.lng(), 6);
        Serial.println(gps.location.rawLng().negative ? " W" : " E");

        Serial.print("Fix Quality: ");
        Serial.println(gps.location.isValid() ? "Valid" : "Invalid");

        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());

        Serial.print("Altitude: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" m");

        Serial.print("Speed: ");
        Serial.print(gps.speed.kmph());
        Serial.println(" km/h");

        Serial.print("Date: ");
        if (gps.date.isValid()) {
            Serial.printf("%02d/%02d/%04d\n", gps.date.day(), gps.date.month(), gps.date.year());
        } else {
            Serial.println("Invalid");
        }

        Serial.print("Time (UTC): ");
        if (gps.time.isValid()) {
            Serial.printf("%02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
        } else {
            Serial.println("Invalid");
        }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println("GPS module not detected or no data received.");
    }

    delay(1000);
}
