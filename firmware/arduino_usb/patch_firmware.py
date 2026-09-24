import re

with open('/Users/tonihandoko/Landslide-EWS-STASRG/arduino/LandslideMonitoringSystem_Improved/LandslideMonitoringSystem_Improved.ino.bak', 'r') as f:
    code = f.read()

# 1. Add Includes and Global Variables
includes_str = """
// ======================================================
// WIFI, MQTT, & GPS ADDITIONS
// ======================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

const char* ssid = "WIFI_SSID_ANDA";
const char* password = "WIFI_PASSWORD_ANDA";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "stasrg/landslide/data";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
TinyGPSPlus gps;

// Pin Definitions for new components
const uint8_t PIEZO2_ANALOG_PIN = 35;
const uint8_t RELAY_PIN = 14;

// Store latest GPS data
float lastLat = 0.0;
float lastLng = 0.0;

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0, 0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
"""

code = code.replace('#include "SparkFunLSM6DS3.h"', '#include "SparkFunLSM6DS3.h"\n' + includes_str)

# 2. Add Relay logic and Piezo2 inside the loop and publish to MQTT
# We need to find the `void printText(const DataRecord &record)` and create a new function `publishMQTT` next to it.
publish_str = """
void publishMQTT(const DataRecord &record) {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Read Piezo 2 for redundancy check
  int rawPiezo2 = analogRead(PIEZO2_ANALOG_PIN);
  float piezo2Voltage = (float)rawPiezo2 * ADC_REFERENCE_VOLTAGE / ADC_MAX_COUNT;
  
  // Update GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
  }

  // Create JSON payload
  StaticJsonDocument<512> doc;
  
  doc["tilt"] = record.orientation.tiltDeg;
  doc["gravity"] = record.motion.gravityDeviationG;
  doc["rotation"] = record.motion.rotationDps;
  doc["piezo"] = record.piezo.vibrationIndex;
  doc["piezo2_raw"] = rawPiezo2;
  doc["hazard"] = record.hazard.hazardIndex;
  
  String stateStr = "UNKNOWN";
  switch(record.hazard.state) {
    case STATE_NORMAL: stateStr = "NORMAL"; break;
    case STATE_WATCH: stateStr = "WATCH"; break;
    case STATE_WARNING: stateStr = "WARNING"; break;
    case STATE_DANGER: stateStr = "DANGER"; break;
    case STATE_CRITICAL: stateStr = "CRITICAL"; break;
  }
  doc["state"] = stateStr;
  
  // GPS Location
  if (lastLat != 0.0 && lastLng != 0.0) {
    doc["lat"] = lastLat;
    doc["lng"] = lastLng;
  }

  // Handle Sirene Relay based on Hazard State
  if (record.hazard.state >= STATE_WARNING) {
     digitalWrite(RELAY_PIN, HIGH); // Sirene ON
  } else {
     digitalWrite(RELAY_PIN, LOW); // Sirene OFF
  }

  String output;
  serializeJson(doc, output);
  
  // Publish
  mqttClient.publish(mqtt_topic, output.c_str());
  
  // Also print to Serial for local debugging
  Serial.print("[MQTT] Published: ");
  Serial.println(output);
}
"""

code = code.replace('void printText(const DataRecord &record) {', publish_str + '\nvoid printText(const DataRecord &record) {')


# 3. Inject into setup()
setup_additions = """
  // Initialize Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  // Initialize Piezo 2
  pinMode(PIEZO2_ANALOG_PIN, INPUT);

  // Initialize GPS on UART1 (RX=25, TX=26)
  Serial1.begin(9600, SERIAL_8N1, 25, 26);

  // Initialize WiFi & MQTT
  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
"""

setup_regex = re.compile(r'(void setup\(\) \{)(.*?)(configureIMUSettings\(\);)', re.DOTALL)
match = setup_regex.search(code)
if match:
    code = code[:match.start(2)] + match.group(2) + setup_additions + "\n  " + match.group(3) + code[match.end(3):]
else:
    print("Warning: could not find setup injection point")


# 4. Inject into loop() to actually call publishMQTT
# find where printText is called
loop_regex = re.compile(r'(#ifdef MODE_TEXT\s*printText\(record\);\s*#endif)')
match = loop_regex.search(code)
if match:
    code = code[:match.start()] + match.group(1) + "\n\n  // Publish to Cloud\n  publishMQTT(record);\n" + code[match.end():]
else:
    print("Warning: could not find loop injection point")


with open('/Users/tonihandoko/Landslide-EWS-STASRG/arduino/LandslideMonitoringSystem_Improved/LandslideMonitoringSystem_Improved.ino', 'w') as f:
    f.write(code)

print("Patching complete!")
