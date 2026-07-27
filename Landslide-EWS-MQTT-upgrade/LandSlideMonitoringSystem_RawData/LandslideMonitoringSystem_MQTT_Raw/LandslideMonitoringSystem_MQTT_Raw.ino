// LandslideMonitoringSystem_MQTT_Raw.ino
// Target: ESP32 + SparkFun LSM6DS3 IMU + DFRobot SEN0209 piezo film
//
// RAW-DATA REWORK
// ----------------
// This version removes EVERY software filter from the original sketch:
//   - No median filter, no moving average, no low-pass, no high-pass (IMU).
//   - No piezo moving average, median, or high-pass.
//   - No complementary filter: roll/pitch come straight from the raw
//     accelerometer of the current sample.
//   - No trend smoothing (EMA) and no energy/RMS moving windows.
//   - No accelerometer offset or gyro bias subtraction: the values used and
//     published are the raw sensor readings.
//
// Calibration is still performed once at startup, but ONLY to learn reference
// values (gravity reference vector, initial roll/pitch, piezo baseline, noise
// statistics for adaptive thresholds). The runtime data itself is never
// modified.
//
// MQTT REWORK
// -----------
// The device now publishes on ONE channel only: the full DataRecord as a JSON
// document on TOPIC_DATARECORD. The old telemetry/event/status topics are
// removed. Point the Node-RED "mqtt in" node at the new topic.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "SparkFunLSM6DS3.h"

// Network telemetry (WiFi + MQTT). PubSubClient and ArduinoJson are already
// declared in platformio.ini; Arduino IDE users must install both libraries.
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======================================================
// DISPLAY MODE
// ======================================================
// Uncomment one mode only. Plot mode prints one label header, then numeric rows.
// Text mode prints calibration, diagnostics, thresholds, events, and state.

#define MODE_TEXT
// #define MODE_PLOT

#if defined(MODE_TEXT) && defined(MODE_PLOT)
#error "Select MODE_TEXT or MODE_PLOT, not both."
#endif

#if !defined(MODE_TEXT) && !defined(MODE_PLOT)
#error "Select one display mode."
#endif

// ======================================================
// HARDWARE FEATURE SELECTION
// ======================================================
// Connect LSM6DS3 INT1 to IMU_INT1_PIN to use the interrupt path. The loop still
// has a timed polling fallback, which keeps monitoring alive if the wire is not
// connected during early testing.

#define USE_IMU_INTERRUPT
#define USE_FIFO_BUFFER
#define USE_WAKE_ACTIVITY_DETECTION

// ======================================================
// CONFIGURATION
// ======================================================

// SparkFun LSM6DS3 boards commonly use 0x6B. Use 0x6A if SA0 is pulled low.
const uint8_t IMU_I2C_ADDRESS = 0x6B;

// ESP32 interrupt input connected to LSM6DS3 INT1.
const uint8_t IMU_INT1_PIN = 27;

// DFRobot SEN0209 piezo film analog output.
const uint8_t PIEZO_ANALOG_PIN = 34;
const uint8_t ADC_RESOLUTION_BITS = 12;
const uint16_t ADC_MAX_COUNT = 4095;
const float ADC_REFERENCE_VOLTAGE = 3.3f;

// IMU output configuration.
const uint16_t IMU_ODR_HZ = 104;          // Output Data Rate in Hz.
const uint16_t ACCEL_FULL_SCALE_G = 2;    // Valid library ranges: 2, 4, 8, 16 g.
const uint16_t GYRO_FULL_SCALE_DPS = 245; // Valid ranges: 125/245/500/1000/2000.

// Main loop period.
const uint16_t SAMPLE_INTERVAL_MS = 100;

// Calibration. Samples are collected while the sensor is still. The standard
// deviations measured here become the baseline for adaptive thresholds. The
// measured offsets are NOT applied to runtime data in this raw version.
const int CAL_SAMPLES = 300;
const uint16_t CAL_SAMPLE_DELAY_MS = 10;
const float STILLNESS_ACCEL_STD_GOOD = 0.006f; // g, empirical quality reference.
const float STILLNESS_GYRO_STD_GOOD = 0.35f;   // dps, empirical quality reference.

// Adaptive threshold multipliers. Threshold = learned mean + K * learned stddev.
// NOTE: raw (unfiltered) data is noisier than filtered data, so expect these
// thresholds to land closer to their maximum clamps than before.
const float WATCH_TILT_K = 3.0f;
const float WARNING_TILT_K = 7.0f;
const float DANGER_TILT_K = 13.0f;
const float TILT_MIN_WATCH_DEG = 1.5f;
const float TILT_MIN_WARNING_DEG = 4.0f;
const float TILT_MIN_DANGER_DEG = 8.0f;
const float TILT_MAX_WATCH_DEG = 8.0f;
const float TILT_MAX_WARNING_DEG = 18.0f;
const float TILT_MAX_DANGER_DEG = 35.0f;

const float SHOCK_K = 6.0f;
const float SHOCK_MIN_G = 0.05f;
const float SHOCK_MAX_G = 1.00f;

const float ROTATION_K = 6.0f;
const float ROTATION_MIN_DPS = 10.0f;
const float ROTATION_MAX_DPS = 250.0f;

const float GRAVITY_DIFF_K = 6.0f;
const float GRAVITY_DIFF_MIN_G = 0.035f;
const float GRAVITY_DIFF_MAX_G = 0.50f;

const float TILT_VELOCITY_K = 8.0f;
const float TILT_VELOCITY_MIN_DPS = 0.20f;
const float TILT_VELOCITY_MAX_DPS = 20.0f;

const float GRAVITY_RATE_K = 8.0f;
const float GRAVITY_RATE_MIN_GPS = 0.003f;
const float GRAVITY_RATE_MAX_GPS = 0.20f;

// Piezo adaptive thresholds. ADC counts are used internally because the ESP32
// ADC is the direct measurement and the SEN0209 output depends on the divider.
const float PIEZO_LOW_K = 3.0f;
const float PIEZO_MODERATE_K = 6.0f;
const float PIEZO_HIGH_K = 10.0f;
const float PIEZO_SEVERE_K = 18.0f;
const float PIEZO_MIN_LOW_ADC = 8.0f;
const float PIEZO_MIN_MODERATE_ADC = 20.0f;
const float PIEZO_MIN_HIGH_ADC = 45.0f;
const float PIEZO_MIN_SEVERE_ADC = 90.0f;
const float PIEZO_MAX_SEVERE_ADC = 1800.0f;

// Weighted hazard index. Keep the weights summing to 100 for easy
// interpretation. The long-drift weight from the filtered version is gone
// because trend smoothing was removed; its weight moved to tilt/shock/gravity.
const float WEIGHT_TILT = 35.0f;
const float WEIGHT_ROTATION = 15.0f;
const float WEIGHT_SHOCK = 15.0f;
const float WEIGHT_GRAVITY = 15.0f;
const float WEIGHT_PIEZO_VIBRATION = 15.0f;
const float WEIGHT_SENSOR_FUSION = 5.0f;

// Hazard-index class thresholds. The finite-state machine adds sample-count
// hysteresis on top of these values (hysteresis is state logic, not a data
// filter: the published data stays raw).
const float HAZARD_WATCH_INDEX = 20.0f;
const float HAZARD_WARNING_INDEX = 45.0f;
const float HAZARD_DANGER_INDEX = 70.0f;
const float HAZARD_CRITICAL_INDEX = 90.0f;
const uint8_t ESCALATE_SAMPLES = 3;
const uint8_t DEESCALATE_SAMPLES = 8;

// Event persistence. Continuous vibration and progressive tilt require multiple
// samples so one spike does not become an event by itself.
const uint8_t VIBRATION_CONSECUTIVE_SAMPLES = 8;
const uint8_t PROGRESSIVE_TILT_SAMPLES = 10;
const float VIBRATION_FACTOR = 0.75f;
const float EARTHQUAKE_ROTATION_FACTOR = 0.60f;
const float SENSOR_DISTURBANCE_LOW_G = 0.55f;
const float SENSOR_DISTURBANCE_HIGH_G = 1.45f;
const uint8_t PIEZO_REPEATED_PEAKS_PER_SECOND = 4;
const uint8_t PIEZO_CONTINUOUS_SAMPLES = 10;

// FIFO and wake/activity register settings.
const uint8_t FIFO_WATERMARK_LSB = 24; // 12-bit FIFO watermark, low byte.
const uint8_t FIFO_WATERMARK_MSB = 0;
const uint8_t WAKE_THRESHOLD_REGISTER_VALUE = 0x02;
const uint8_t WAKE_DURATION_REGISTER_VALUE = 0x02;

// ======================================================
// TYPES
// ======================================================

struct Vec3 {
  float x;
  float y;
  float z;
};

struct SensorSample {
  uint32_t timestampMs;
  Vec3 accelRaw;
  Vec3 gyroRaw;
};

struct PiezoSample {
  uint32_t timestampMs;
  uint16_t rawAdc;
  float voltage;
};

struct OrientationData {
  float rollDeg;
  float pitchDeg;
  float tiltDeg;
  float tiltVelocityDps;
};

struct MotionData {
  float accelMagnitudeG;
  float gravityDiffG;
  float gravitySlopeGps; // signed instantaneous rate of gravityDiffG
  float shockG;
  float rotationDps;
};

struct AdaptiveThresholds {
  float watchTiltDeg;
  float warningTiltDeg;
  float dangerTiltDeg;
  float shockG;
  float rotationDps;
  float gravityDiffG;
  float tiltVelocityDps;
  float gravityRateGps;
};

struct PiezoThresholds {
  float lowAmplitudeAdc;
  float moderateAmplitudeAdc;
  float highAmplitudeAdc;
  float severeAmplitudeAdc;
};

struct CalibrationData {
  Vec3 accelMeanRaw;
  Vec3 gyroMeanRaw;
  Vec3 accelStdRaw;
  Vec3 gyroStdRaw;
  Vec3 gravityReference; // mean raw accel while still (reference only)
  float gravityMagnitudeG;
  float initialRollDeg;
  float initialPitchDeg;
  float accelMagnitudeMeanG;
  float accelMagnitudeStdG;
  float gyroMagnitudeMeanDps;
  float gyroMagnitudeStdDps;
  float tiltNoiseStdDeg;
  float tiltVelocityNoiseStdDps;
  float gravityRateNoiseStdGps;
  float qualityPercent;
};

struct PiezoCalibrationData {
  float baselineAdc;
  float baselineVoltage;
  float noiseStdAdc;
  float noiseMeanAbsAdc;
  float qualityPercent;
};

enum AlertState {
  STATE_NORMAL = 0,
  STATE_WATCH,
  STATE_WARNING,
  STATE_DANGER,
  STATE_CRITICAL
};

enum VibrationState {
  VIBRATION_VERY_QUIET = 0,
  VIBRATION_LOW,
  VIBRATION_MODERATE,
  VIBRATION_HIGH,
  VIBRATION_SEVERE
};

enum EventFlag {
  EVENT_NONE = 0,
  EVENT_SUDDEN_IMPACT = 1 << 0,
  EVENT_CONTINUOUS_VIBRATION = 1 << 1,
  EVENT_RAPID_TILT = 1 << 2,
  EVENT_PROGRESSIVE_TILT = 1 << 3,
  EVENT_SENSOR_DISTURBANCE = 1 << 4,
  EVENT_EARTHQUAKE_LIKE = 1 << 5,
  EVENT_SLOW_LANDSLIDE = 1 << 6,
  EVENT_SUDDEN_LANDSLIDE = 1 << 7,
  EVENT_CONTINUOUS_CREEP = 1 << 8,
  EVENT_FIFO_OVERFLOW = 1 << 9,
  EVENT_PIEZO_REPEATED_VIBRATION = 1 << 10,
  EVENT_PIEZO_CONTINUOUS_VIBRATION = 1 << 11,
  EVENT_PIEZO_IMPACT = 1 << 12,
  EVENT_PIEZO_ROCKFALL_LIKE = 1 << 13,
  EVENT_PIEZO_ENVIRONMENTAL = 1 << 14,
  EVENT_PIEZO_MECHANICAL = 1 << 15
};

const uint16_t EVENT_LAST_BIT = EVENT_PIEZO_MECHANICAL;

struct VibrationData {
  float amplitudeAdc;       // |raw ADC - calibration baseline| of this sample
  float peakAmplitudeAdc;   // highest amplitude in the current 1 s window
  uint16_t peakCount;       // total threshold crossings since boot
  float peaksPerSecond;
  float vibrationIndex;     // 0..100 from the raw amplitude
  float rateAdcPerSecond;   // |amplitude - previous amplitude| / dt
  VibrationState state;
};

// The full record built every sample. This is exactly what gets published on
// the single MQTT DataRecord channel.
struct DataRecord {
  uint32_t timestampMs;
  Vec3 accelRaw;
  Vec3 gyroRaw;
  float accelMagnitudeG;
  float rollDeg;
  float pitchDeg;
  float tiltDeg;
  float tiltVelocityDps;
  float gravityDiffG;
  float gravitySlopeGps;
  float shockG;
  float rotationDps;
  uint16_t piezoRawAdc;
  float piezoVoltage;
  float piezoAmplitudeAdc;
  float piezoPeakAdc;
  uint16_t piezoPeakCount;
  float piezoPeaksPerSecond;
  float vibrationIndex;
  VibrationState vibrationState;
  float fusionConfidence;
  float hazardIndex;
  AlertState state;
  uint16_t eventMask;
  uint16_t newEventMask;
};

struct RunningStats {
  uint32_t n;
  float mean;
  float m2;
};

// ======================================================
// GLOBAL STATE
// ======================================================

// Konfigurasi Auto-Calibration & Buzzer
const uint8_t BUZZER_PIN = 14;              // Ganti dengan pin GPIO tempat Buzzer terhubung
const uint16_t AUTO_CALIBRATION_DELAY_MS = 10000; // 10 detik
uint32_t stillnessTimerStartMs = 0;

LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);

volatile bool imuInterruptFlag = true;
bool plotHeaderPrinted = false;
bool orientationInitialized = false;
uint32_t lastSampleMs = 0;
uint32_t lastTransitionMs = 0;

CalibrationData calibration;
AdaptiveThresholds thresholds;
PiezoCalibrationData piezoCalibration;
PiezoThresholds piezoThresholds;
AlertState currentState = STATE_NORMAL;

float previousTiltDeg = 0.0f;
float previousGravityDiffG = 0.0f;
float previousPiezoAmplitudeAdc = 0.0f;

uint16_t piezoTotalPeakCount = 0;
uint16_t piezoPeakWindowCount = 0;
uint32_t piezoPeakWindowStartMs = 0;
float piezoPeaksPerSecondValue = 0.0f;
float piezoWindowPeakAdc = 0.0f;

uint8_t escalateCount = 0;
uint8_t deescalateCount = 0;
uint8_t vibrationCount = 0;
uint8_t piezoContinuousCount = 0;
uint8_t progressiveTiltCount = 0;
uint16_t previousEventMask = EVENT_NONE;
uint16_t hardwareEventMask = EVENT_NONE;

// ======================================================
// UTILITY FUNCTIONS
// ======================================================

float squareFloat(float value) {
  return value * value;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float safeDivide(float numerator, float denominator, float fallback) {
  if (fabsf(denominator) < 0.000001f) return fallback;
  return numerator / denominator;
}

float vecMagnitude(Vec3 v) {
  return sqrtf(squareFloat(v.x) + squareFloat(v.y) + squareFloat(v.z));
}

Vec3 vecSub(Vec3 a, Vec3 b) {
  Vec3 out = {a.x - b.x, a.y - b.y, a.z - b.z};
  return out;
}

float angleDifferenceDeg(float current, float reference) {
  float diff = current - reference;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

const char *stateName(AlertState state) {
  switch (state) {
    case STATE_NORMAL: return "NORMAL";
    case STATE_WATCH: return "WATCH";
    case STATE_WARNING: return "WARNING";
    case STATE_DANGER: return "DANGER";
    case STATE_CRITICAL: return "CRITICAL";
  }
  return "UNKNOWN";
}

const char *vibrationStateName(VibrationState state) {
  switch (state) {
    case VIBRATION_VERY_QUIET: return "VERY QUIET";
    case VIBRATION_LOW: return "LOW";
    case VIBRATION_MODERATE: return "MODERATE";
    case VIBRATION_HIGH: return "HIGH";
    case VIBRATION_SEVERE: return "SEVERE";
  }
  return "UNKNOWN";
}

void resetStats(RunningStats &stats) {
  stats.n = 0;
  stats.mean = 0.0f;
  stats.m2 = 0.0f;
}

void pushStats(RunningStats &stats, float value) {
  stats.n++;
  float delta = value - stats.mean;
  stats.mean += delta / stats.n;
  float delta2 = value - stats.mean;
  stats.m2 += delta * delta2;
}

float statsStdDev(const RunningStats &stats) {
  if (stats.n < 2) return 0.0f;
  return sqrtf(stats.m2 / (stats.n - 1));
}

// ======================================================
// LOW-LEVEL LSM6DS3 REGISTER ACCESS
// ======================================================

bool writeImuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readImuRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(IMU_I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)IMU_I2C_ADDRESS, 1) != 1) return false;
  value = Wire.read();
  return true;
}

// Configure FIFO, data-ready interrupt, and wake/activity detection.
void configureHardwareFeatures() {
#ifdef USE_FIFO_BUFFER
  writeImuRegister(0x06, FIFO_WATERMARK_LSB);             // FIFO_CTRL1
  writeImuRegister(0x07, FIFO_WATERMARK_MSB & 0x0F);      // FIFO_CTRL2
  writeImuRegister(0x08, 0x09);                           // FIFO_CTRL3: gyro/accel decimation.
  writeImuRegister(0x09, 0x00);                           // FIFO_CTRL4: no extra sensors.
  writeImuRegister(0x0A, 0x26);                           // FIFO_CTRL5: 104 Hz continuous mode.
#endif

#ifdef USE_IMU_INTERRUPT
  writeImuRegister(0x0D, 0x03);                           // INT1_CTRL: accel + gyro data ready.
#endif

#ifdef USE_WAKE_ACTIVITY_DETECTION
  writeImuRegister(0x5B, WAKE_THRESHOLD_REGISTER_VALUE);  // WAKE_UP_THS.
  writeImuRegister(0x5C, WAKE_DURATION_REGISTER_VALUE);   // WAKE_UP_DUR.
  writeImuRegister(0x58, 0x20);                           // TAP_CFG: inactivity engine, no tap use.
  writeImuRegister(0x5E, 0x20);                           // MD1_CFG: wake-up event on INT1.
#endif
}

void checkFifoStatus() {
#ifdef USE_FIFO_BUFFER
  uint8_t status2 = 0;
  if (readImuRegister(0x3B, status2)) {
    if (status2 & 0x40) {
      hardwareEventMask |= EVENT_FIFO_OVERFLOW;
    }
  }
#endif
}

void IRAM_ATTR imuInterruptServiceRoutine() {
  imuInterruptFlag = true;
}

// ======================================================
// IMU SETUP AND SAMPLING
// ======================================================

void configureIMUSettings() {
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = ACCEL_FULL_SCALE_G;
  myIMU.settings.accelSampleRate = IMU_ODR_HZ;
  myIMU.settings.accelBandWidth = 100;
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = GYRO_FULL_SCALE_DPS;
  myIMU.settings.gyroSampleRate = IMU_ODR_HZ;
  myIMU.settings.gyroBandWidth = 200;
#ifdef USE_FIFO_BUFFER
  myIMU.settings.accelFifoEnabled = 1;
  myIMU.settings.gyroFifoEnabled = 1;
  myIMU.settings.accelFifoDecimation = 1;
  myIMU.settings.gyroFifoDecimation = 1;
#endif
}

// Pure raw read. No offset correction, no bias subtraction, no filtering.
void readSensor(SensorSample &sample) {
  sample.timestampMs = millis();
  sample.accelRaw.x = myIMU.readFloatAccelX();
  sample.accelRaw.y = myIMU.readFloatAccelY();
  sample.accelRaw.z = myIMU.readFloatAccelZ();
  sample.gyroRaw.x = myIMU.readFloatGyroX();
  sample.gyroRaw.y = myIMU.readFloatGyroY();
  sample.gyroRaw.z = myIMU.readFloatGyroZ();
}

void configurePiezoInput() {
  pinMode(PIEZO_ANALOG_PIN, INPUT);
  analogReadResolution(ADC_RESOLUTION_BITS);
  analogSetPinAttenuation(PIEZO_ANALOG_PIN, ADC_11db);
}

void readPiezo(PiezoSample &sample) {
  sample.timestampMs = millis();
  sample.rawAdc = analogRead(PIEZO_ANALOG_PIN);
  sample.voltage = (sample.rawAdc * ADC_REFERENCE_VOLTAGE) / ADC_MAX_COUNT;
}

// ======================================================
// CALIBRATION (reference values only -- runtime data stays raw)
// ======================================================

float calculateRollFromAccel(Vec3 accel) {
  return atan2f(accel.y, accel.z) * 180.0f / PI;
}

float calculatePitchFromAccel(Vec3 accel) {
  float denominator = sqrtf(squareFloat(accel.y) + squareFloat(accel.z));
  return atan2f(-accel.x, denominator) * 180.0f / PI;
}

float calculateCalibrationQuality(float accelStd, float gyroStd, float gravityMag) {
  float accelPenalty = clampFloat(accelStd / STILLNESS_ACCEL_STD_GOOD, 0.0f, 4.0f) * 18.0f;
  float gyroPenalty = clampFloat(gyroStd / STILLNESS_GYRO_STD_GOOD, 0.0f, 4.0f) * 12.0f;
  float gravityPenalty = clampFloat(fabsf(gravityMag - 1.0f) / 0.08f, 0.0f, 4.0f) * 8.0f;
  return clampFloat(100.0f - accelPenalty - gyroPenalty - gravityPenalty, 0.0f, 100.0f);
}

void printCalibrationInfo() {
#ifdef MODE_TEXT
  Serial.println();
  Serial.println("CALIBRATION (reference only, data stays raw)");
  Serial.println("------------------------------------------------------");
  Serial.printf("Samples              : %d\n", CAL_SAMPLES);
  Serial.printf("Accel mean raw       : %8.5f %8.5f %8.5f g\n",
                calibration.accelMeanRaw.x, calibration.accelMeanRaw.y, calibration.accelMeanRaw.z);
  Serial.printf("Accel std raw        : %8.5f %8.5f %8.5f g\n",
                calibration.accelStdRaw.x, calibration.accelStdRaw.y, calibration.accelStdRaw.z);
  Serial.printf("Gyro mean raw        : %8.4f %8.4f %8.4f dps\n",
                calibration.gyroMeanRaw.x, calibration.gyroMeanRaw.y, calibration.gyroMeanRaw.z);
  Serial.printf("Gyro std raw         : %8.4f %8.4f %8.4f dps\n",
                calibration.gyroStdRaw.x, calibration.gyroStdRaw.y, calibration.gyroStdRaw.z);
  Serial.printf("Gravity reference    : %8.5f %8.5f %8.5f g\n",
                calibration.gravityReference.x, calibration.gravityReference.y, calibration.gravityReference.z);
  Serial.printf("Gravity magnitude    : %8.5f g\n", calibration.gravityMagnitudeG);
  Serial.printf("Initial roll/pitch   : %8.3f %8.3f deg\n",
                calibration.initialRollDeg, calibration.initialPitchDeg);
  Serial.printf("Accel mag mean/std   : %8.5f %8.5f g\n",
                calibration.accelMagnitudeMeanG, calibration.accelMagnitudeStdG);
  Serial.printf("Gyro mag mean/std    : %8.4f %8.4f dps\n",
                calibration.gyroMagnitudeMeanDps, calibration.gyroMagnitudeStdDps);
  Serial.printf("Tilt noise std       : %8.4f deg\n", calibration.tiltNoiseStdDeg);
  Serial.printf("Calibration quality  : %8.1f %%\n", calibration.qualityPercent);
  Serial.println();
#endif
}

void calibrateSensor() {
  RunningStats axStats, ayStats, azStats, gxStats, gyStats, gzStats;
  RunningStats accelMagStats, gyroMagStats, rollStats, pitchStats;
  resetStats(axStats); resetStats(ayStats); resetStats(azStats);
  resetStats(gxStats); resetStats(gyStats); resetStats(gzStats);
  resetStats(accelMagStats); resetStats(gyroMagStats);
  resetStats(rollStats); resetStats(pitchStats);

#ifdef MODE_TEXT
  Serial.println("Calibrating... keep the sensor still.");
#endif

  for (int i = 0; i < CAL_SAMPLES; i++) {
    Vec3 accel = {
      myIMU.readFloatAccelX(),
      myIMU.readFloatAccelY(),
      myIMU.readFloatAccelZ()
    };
    Vec3 gyro = {
      myIMU.readFloatGyroX(),
      myIMU.readFloatGyroY(),
      myIMU.readFloatGyroZ()
    };

    pushStats(axStats, accel.x);
    pushStats(ayStats, accel.y);
    pushStats(azStats, accel.z);
    pushStats(gxStats, gyro.x);
    pushStats(gyStats, gyro.y);
    pushStats(gzStats, gyro.z);
    pushStats(accelMagStats, vecMagnitude(accel));
    pushStats(gyroMagStats, vecMagnitude(gyro));
    pushStats(rollStats, calculateRollFromAccel(accel));
    pushStats(pitchStats, calculatePitchFromAccel(accel));
    delay(CAL_SAMPLE_DELAY_MS);
  }

  calibration.accelMeanRaw = {axStats.mean, ayStats.mean, azStats.mean};
  calibration.gyroMeanRaw = {gxStats.mean, gyStats.mean, gzStats.mean};
  calibration.accelStdRaw = {statsStdDev(axStats), statsStdDev(ayStats), statsStdDev(azStats)};
  calibration.gyroStdRaw = {statsStdDev(gxStats), statsStdDev(gyStats), statsStdDev(gzStats)};
  calibration.gravityMagnitudeG = vecMagnitude(calibration.accelMeanRaw);

  // The gravity reference is the mean raw accel vector while still. It is only
  // used as a comparison reference; raw samples are never corrected with it.
  calibration.gravityReference = calibration.accelMeanRaw;
  calibration.initialRollDeg = calculateRollFromAccel(calibration.gravityReference);
  calibration.initialPitchDeg = calculatePitchFromAccel(calibration.gravityReference);
  calibration.accelMagnitudeMeanG = accelMagStats.mean;
  calibration.accelMagnitudeStdG = statsStdDev(accelMagStats);
  calibration.gyroMagnitudeMeanDps = gyroMagStats.mean;
  calibration.gyroMagnitudeStdDps = statsStdDev(gyroMagStats);
  calibration.tiltNoiseStdDeg = sqrtf(squareFloat(statsStdDev(rollStats)) +
                                      squareFloat(statsStdDev(pitchStats)));
  calibration.tiltVelocityNoiseStdDps = calibration.tiltNoiseStdDeg / (CAL_SAMPLE_DELAY_MS / 1000.0f);
  calibration.gravityRateNoiseStdGps = calibration.accelMagnitudeStdG / (CAL_SAMPLE_DELAY_MS / 1000.0f);
  calibration.qualityPercent = calculateCalibrationQuality(calibration.accelMagnitudeStdG,
                                                           calibration.gyroMagnitudeStdDps,
                                                           calibration.gravityMagnitudeG);

  previousTiltDeg = 0.0f;
  orientationInitialized = true;

  printCalibrationInfo();
}

void printPiezoCalibrationInfo() {
#ifdef MODE_TEXT
  Serial.println("PIEZO CALIBRATION (baseline reference only)");
  Serial.println("------------------------------------------------------");
  Serial.printf("Analog pin           : GPIO %u\n", PIEZO_ANALOG_PIN);
  Serial.printf("Baseline             : %8.2f ADC, %6.3f V\n",
                piezoCalibration.baselineAdc, piezoCalibration.baselineVoltage);
  Serial.printf("Noise std / mean abs : %8.3f %8.3f ADC\n",
                piezoCalibration.noiseStdAdc, piezoCalibration.noiseMeanAbsAdc);
  Serial.printf("Amplitude L/M/H/S    : %8.2f %8.2f %8.2f %8.2f ADC\n",
                piezoThresholds.lowAmplitudeAdc, piezoThresholds.moderateAmplitudeAdc,
                piezoThresholds.highAmplitudeAdc, piezoThresholds.severeAmplitudeAdc);
  Serial.printf("Calibration quality  : %8.1f %%\n", piezoCalibration.qualityPercent);
  Serial.println();
#endif
}

void calculatePiezoThresholds() {
  float noiseStd = fmaxf(piezoCalibration.noiseStdAdc, 1.0f);
  piezoThresholds.lowAmplitudeAdc = fmaxf(noiseStd * PIEZO_LOW_K, PIEZO_MIN_LOW_ADC);
  piezoThresholds.moderateAmplitudeAdc = fmaxf(noiseStd * PIEZO_MODERATE_K, PIEZO_MIN_MODERATE_ADC);
  piezoThresholds.highAmplitudeAdc = fmaxf(noiseStd * PIEZO_HIGH_K, PIEZO_MIN_HIGH_ADC);
  piezoThresholds.severeAmplitudeAdc = clampFloat(noiseStd * PIEZO_SEVERE_K,
                                                  PIEZO_MIN_SEVERE_ADC,
                                                  PIEZO_MAX_SEVERE_ADC);

  piezoThresholds.moderateAmplitudeAdc = fmaxf(piezoThresholds.moderateAmplitudeAdc,
                                               piezoThresholds.lowAmplitudeAdc + 4.0f);
  piezoThresholds.highAmplitudeAdc = fmaxf(piezoThresholds.highAmplitudeAdc,
                                           piezoThresholds.moderateAmplitudeAdc + 8.0f);
  piezoThresholds.severeAmplitudeAdc = fmaxf(piezoThresholds.severeAmplitudeAdc,
                                             piezoThresholds.highAmplitudeAdc + 16.0f);
}

void calibratePiezo() {
  RunningStats rawStats, deviationStats;
  resetStats(rawStats);
  resetStats(deviationStats);

#ifdef MODE_TEXT
  Serial.println("Calibrating piezo... keep the sensor mechanically quiet.");
#endif

  for (int i = 0; i < CAL_SAMPLES; i++) {
    pushStats(rawStats, analogRead(PIEZO_ANALOG_PIN));
    delay(CAL_SAMPLE_DELAY_MS);
  }

  piezoCalibration.baselineAdc = rawStats.mean;
  piezoCalibration.baselineVoltage = (piezoCalibration.baselineAdc * ADC_REFERENCE_VOLTAGE) /
                                     ADC_MAX_COUNT;
  piezoCalibration.noiseStdAdc = statsStdDev(rawStats);

  for (int i = 0; i < CAL_SAMPLES; i++) {
    pushStats(deviationStats, fabsf(analogRead(PIEZO_ANALOG_PIN) - piezoCalibration.baselineAdc));
    delay(CAL_SAMPLE_DELAY_MS);
  }

  piezoCalibration.noiseMeanAbsAdc = deviationStats.mean;
  piezoCalibration.qualityPercent = clampFloat(100.0f - piezoCalibration.noiseStdAdc * 1.5f,
                                               0.0f, 100.0f);

  piezoPeakWindowStartMs = millis();
  calculatePiezoThresholds();
  printPiezoCalibrationInfo();
}

void calculateAdaptiveThresholds() {
  float tiltStd = calibration.tiltNoiseStdDeg;
  thresholds.watchTiltDeg = clampFloat(WATCH_TILT_K * tiltStd, TILT_MIN_WATCH_DEG, TILT_MAX_WATCH_DEG);
  thresholds.warningTiltDeg = clampFloat(WARNING_TILT_K * tiltStd, TILT_MIN_WARNING_DEG, TILT_MAX_WARNING_DEG);
  thresholds.dangerTiltDeg = clampFloat(DANGER_TILT_K * tiltStd, TILT_MIN_DANGER_DEG, TILT_MAX_DANGER_DEG);

  thresholds.warningTiltDeg = fmaxf(thresholds.warningTiltDeg, thresholds.watchTiltDeg + 1.0f);
  thresholds.dangerTiltDeg = fmaxf(thresholds.dangerTiltDeg, thresholds.warningTiltDeg + 2.0f);

  thresholds.shockG = clampFloat(calibration.accelMagnitudeStdG * SHOCK_K, SHOCK_MIN_G, SHOCK_MAX_G);
  thresholds.rotationDps = clampFloat(calibration.gyroMagnitudeMeanDps +
                                      ROTATION_K * calibration.gyroMagnitudeStdDps,
                                      ROTATION_MIN_DPS, ROTATION_MAX_DPS);
  thresholds.gravityDiffG = clampFloat(calibration.accelMagnitudeStdG * GRAVITY_DIFF_K,
                                       GRAVITY_DIFF_MIN_G, GRAVITY_DIFF_MAX_G);
  thresholds.tiltVelocityDps = clampFloat(calibration.tiltVelocityNoiseStdDps * TILT_VELOCITY_K,
                                          TILT_VELOCITY_MIN_DPS, TILT_VELOCITY_MAX_DPS);
  thresholds.gravityRateGps = clampFloat(calibration.gravityRateNoiseStdGps * GRAVITY_RATE_K,
                                         GRAVITY_RATE_MIN_GPS, GRAVITY_RATE_MAX_GPS);
}

// ======================================================
// AUTO-CALIBRATION & BUZZER
// ======================================================

void checkAutoCalibration(const SensorSample &sample, const OrientationData &orientation,
                          const MotionData &motion, uint32_t nowMs) {
  // Kondisi sensor dianggap "diam" (gerakan di bawah 30% ambang batas danger)
  bool isStill = (orientation.tiltVelocityDps < (thresholds.tiltVelocityDps * 0.3f)) &&
                 (motion.rotationDps < (thresholds.rotationDps * 0.3f)) &&
                 (motion.shockG < (thresholds.shockG * 0.3f));

  if (isStill) {
    // Mulai timer jika baru saja berhenti bergerak
    if (stillnessTimerStartMs == 0) {
      stillnessTimerStartMs = nowMs;
    }
    // Jika sudah diam selama 10 detik, lakukan kalibrasi ulang
    else if ((nowMs - stillnessTimerStartMs) >= AUTO_CALIBRATION_DELAY_MS) {

      // Set posisi saat ini sebagai titik acuan (kalibrasi) baru
      calibration.initialRollDeg = orientation.rollDeg;
      calibration.initialPitchDeg = orientation.pitchDeg;
      calibration.gravityReference = sample.accelRaw;

      // Reset state machine ke NORMAL agar buzzer berhenti
      currentState = STATE_NORMAL;
      escalateCount = 0;
      deescalateCount = 0;
      stillnessTimerStartMs = 0; // Reset timer

#ifdef MODE_TEXT
      Serial.println("\n>>> AUTO-CALIBRATION: Sensor diam selama 10 detik. Titik referensi diperbarui ke posisi baru. <<<\n");
#endif
    }
  } else {
    // Reset timer jika sensor kembali bergerak
    if (stillnessTimerStartMs != 0) {
      stillnessTimerStartMs = 0;
    }
  }
}

void controlBuzzer() {
  // Buzzer berbunyi jika state mencapai WARNING, DANGER, atau CRITICAL
  if (currentState >= STATE_WARNING) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ======================================================
// ORIENTATION, MOTION, AND VIBRATION (all raw, no filtering)
// ======================================================

// Roll/pitch come directly from the raw accelerometer of the current sample.
// No complementary filter, no gyro integration, no smoothing.
void calculateTilt(const SensorSample &sample, float dtSeconds, OrientationData &orientation) {
  orientation.rollDeg = calculateRollFromAccel(sample.accelRaw);
  orientation.pitchDeg = calculatePitchFromAccel(sample.accelRaw);

  float rollDelta = angleDifferenceDeg(orientation.rollDeg, calibration.initialRollDeg);
  float pitchDelta = angleDifferenceDeg(orientation.pitchDeg, calibration.initialPitchDeg);
  orientation.tiltDeg = sqrtf(squareFloat(rollDelta) + squareFloat(pitchDelta));

  if (!orientationInitialized || dtSeconds <= 0.0f) {
    orientation.tiltVelocityDps = 0.0f;
    orientationInitialized = true;
  } else {
    orientation.tiltVelocityDps = fabsf(orientation.tiltDeg - previousTiltDeg) /
                                  fmaxf(dtSeconds, 0.001f);
  }
}

void calculateMotion(const SensorSample &sample, float dtSeconds, MotionData &motion) {
  motion.accelMagnitudeG = vecMagnitude(sample.accelRaw);

  // Deviation of the raw accel vector from the still-state reference.
  Vec3 gravityDelta = vecSub(sample.accelRaw, calibration.gravityReference);
  motion.gravityDiffG = vecMagnitude(gravityDelta);

  // Signed instantaneous slope (no EMA trend smoothing).
  motion.gravitySlopeGps = (motion.gravityDiffG - previousGravityDiffG) /
                           fmaxf(dtSeconds, 0.001f);

  // Shock relative to the calibrated gravity magnitude.
  motion.shockG = fabsf(motion.accelMagnitudeG - calibration.gravityMagnitudeG);

  // Rotation straight from the raw gyro.
  motion.rotationDps = vecMagnitude(sample.gyroRaw);
}

float normalizedScore(float value, float start, float fullScale) {
  if (value <= start) return 0.0f;
  if (value >= fullScale) return 100.0f;
  return 100.0f * (value - start) / fmaxf(fullScale - start, 0.001f);
}

VibrationState classifyVibration(float vibrationIndex) {
  if (vibrationIndex >= 85.0f) return VIBRATION_SEVERE;
  if (vibrationIndex >= 65.0f) return VIBRATION_HIGH;
  if (vibrationIndex >= 35.0f) return VIBRATION_MODERATE;
  if (vibrationIndex >= 10.0f) return VIBRATION_LOW;
  return VIBRATION_VERY_QUIET;
}

// Peak bookkeeping: counts threshold crossings and tracks the raw peak of the
// current one-second window. Counting events is not filtering; the amplitude
// values themselves stay raw.
void updatePiezoPeaks(float amplitudeAdc, uint32_t timestampMs, VibrationData &vibration) {
  if (amplitudeAdc > piezoWindowPeakAdc) {
    piezoWindowPeakAdc = amplitudeAdc;
  }

  bool crossedPeakThreshold = amplitudeAdc >= piezoThresholds.moderateAmplitudeAdc &&
                              previousPiezoAmplitudeAdc < piezoThresholds.moderateAmplitudeAdc;
  if (crossedPeakThreshold) {
    piezoTotalPeakCount++;
    piezoPeakWindowCount++;
  }

  uint32_t elapsedMs = timestampMs - piezoPeakWindowStartMs;
  if (elapsedMs >= 1000) {
    piezoPeaksPerSecondValue = piezoPeakWindowCount * 1000.0f / elapsedMs;
    piezoPeakWindowCount = 0;
    piezoPeakWindowStartMs = timestampMs;
    piezoWindowPeakAdc = amplitudeAdc;
  }

  vibration.peakAmplitudeAdc = piezoWindowPeakAdc;
  vibration.peakCount = piezoTotalPeakCount;
  vibration.peaksPerSecond = piezoPeaksPerSecondValue;
}

void calculateVibration(const PiezoSample &sample, float dtSeconds, VibrationData &vibration) {
  // Amplitude is simply the raw ADC deviation from the calibration baseline.
  vibration.amplitudeAdc = fabsf((float)sample.rawAdc - piezoCalibration.baselineAdc);

  updatePiezoPeaks(vibration.amplitudeAdc, sample.timestampMs, vibration);

  // Index scales the raw amplitude between the low and severe thresholds.
  vibration.vibrationIndex = clampFloat(normalizedScore(vibration.amplitudeAdc,
                                                        piezoThresholds.lowAmplitudeAdc,
                                                        piezoThresholds.severeAmplitudeAdc),
                                        0.0f, 100.0f);

  vibration.rateAdcPerSecond = fabsf(vibration.amplitudeAdc - previousPiezoAmplitudeAdc) /
                               fmaxf(dtSeconds, 0.001f);
  vibration.state = classifyVibration(vibration.vibrationIndex);
}

// ======================================================
// FUSION AND HAZARD
// ======================================================

float calculateFusionConfidence(const OrientationData &orientation, const MotionData &motion,
                                const VibrationData &vibration) {
  float confidence = 0.0f;

  if (orientation.tiltDeg > thresholds.watchTiltDeg) confidence += 20.0f;
  if (motion.gravityDiffG > thresholds.gravityDiffG * 0.6f) confidence += 20.0f;
  if (motion.rotationDps > thresholds.rotationDps) confidence += 15.0f;
  if (vibration.vibrationIndex > 35.0f) confidence += 20.0f;
  if (vibration.peaksPerSecond >= PIEZO_REPEATED_PEAKS_PER_SECOND) confidence += 10.0f;
  if (orientation.tiltVelocityDps > thresholds.tiltVelocityDps) confidence += 15.0f;

  if (vibration.vibrationIndex > 65.0f &&
      orientation.tiltDeg < thresholds.watchTiltDeg &&
      motion.rotationDps < thresholds.rotationDps) {
    confidence *= 0.45f;
  }

  return clampFloat(confidence, 0.0f, 100.0f);
}

float calculateHazard(const OrientationData &orientation, const MotionData &motion,
                      const VibrationData &vibration, float fusionConfidence) {
  float tiltScore = normalizedScore(orientation.tiltDeg,
                                    thresholds.watchTiltDeg,
                                    thresholds.dangerTiltDeg);
  float rotationScore = normalizedScore(motion.rotationDps,
                                        thresholds.rotationDps,
                                        thresholds.rotationDps * 3.0f);
  float shockScore = normalizedScore(motion.shockG,
                                     thresholds.shockG,
                                     thresholds.shockG * 4.0f);
  float gravityScore = normalizedScore(motion.gravityDiffG,
                                       thresholds.gravityDiffG,
                                       thresholds.gravityDiffG * 3.0f);
  float vibrationScore = vibration.vibrationIndex;

  float weighted = (WEIGHT_TILT * tiltScore +
                    WEIGHT_ROTATION * rotationScore +
                    WEIGHT_SHOCK * shockScore +
                    WEIGHT_GRAVITY * gravityScore +
                    WEIGHT_PIEZO_VIBRATION * vibrationScore +
                    WEIGHT_SENSOR_FUSION * fusionConfidence) / 100.0f;
  return clampFloat(weighted, 0.0f, 100.0f);
}

// ======================================================
// EVENTS AND HAZARD STATE MACHINE
// ======================================================

AlertState classifyHazardIndex(float hazardIndex) {
  if (hazardIndex >= HAZARD_CRITICAL_INDEX) return STATE_CRITICAL;
  if (hazardIndex >= HAZARD_DANGER_INDEX) return STATE_DANGER;
  if (hazardIndex >= HAZARD_WARNING_INDEX) return STATE_WARNING;
  if (hazardIndex >= HAZARD_WATCH_INDEX) return STATE_WATCH;
  return STATE_NORMAL;
}

AlertState updateState(float hazardIndex, uint32_t timestampMs) {
  AlertState target = classifyHazardIndex(hazardIndex);
  if (target > currentState) {
    escalateCount++;
    deescalateCount = 0;
    if (escalateCount >= ESCALATE_SAMPLES) {
      currentState = (AlertState)((int)currentState + 1);
      lastTransitionMs = timestampMs;
      escalateCount = 0;
    }
  } else if (target < currentState) {
    deescalateCount++;
    escalateCount = 0;
    if (deescalateCount >= DEESCALATE_SAMPLES) {
      currentState = (AlertState)((int)currentState - 1);
      lastTransitionMs = timestampMs;
      deescalateCount = 0;
    }
  } else {
    escalateCount = 0;
    deescalateCount = 0;
  }
  return currentState;
}

uint16_t detectEvents(const OrientationData &orientation, const MotionData &motion,
                      const VibrationData &vibration, float fusionConfidence) {
  uint16_t events = hardwareEventMask;
  hardwareEventMask = EVENT_NONE;

  if (motion.shockG > thresholds.shockG) {
    events |= EVENT_SUDDEN_IMPACT;
  }

  if (motion.gravityDiffG > thresholds.shockG * VIBRATION_FACTOR) {
    vibrationCount++;
  } else {
    vibrationCount = 0;
  }
  if (vibrationCount >= VIBRATION_CONSECUTIVE_SAMPLES) {
    events |= EVENT_CONTINUOUS_VIBRATION;
  }

  if (vibration.vibrationIndex >= 65.0f) {
    piezoContinuousCount++;
  } else {
    piezoContinuousCount = 0;
  }
  if (piezoContinuousCount >= PIEZO_CONTINUOUS_SAMPLES) {
    events |= EVENT_PIEZO_CONTINUOUS_VIBRATION;
  }

  if (vibration.peaksPerSecond >= PIEZO_REPEATED_PEAKS_PER_SECOND) {
    events |= EVENT_PIEZO_REPEATED_VIBRATION;
  }

  if (vibration.peakAmplitudeAdc > piezoThresholds.highAmplitudeAdc) {
    events |= EVENT_PIEZO_IMPACT;
  }

  if (vibration.peakAmplitudeAdc > piezoThresholds.severeAmplitudeAdc &&
      orientation.tiltDeg < thresholds.watchTiltDeg) {
    events |= EVENT_PIEZO_ROCKFALL_LIKE;
  }

  if (orientation.tiltVelocityDps > thresholds.tiltVelocityDps) {
    events |= EVENT_RAPID_TILT;
  }

  if (orientation.tiltDeg > thresholds.watchTiltDeg &&
      motion.gravitySlopeGps > thresholds.gravityRateGps * 0.25f) {
    progressiveTiltCount++;
  } else {
    progressiveTiltCount = 0;
  }
  if (progressiveTiltCount >= PROGRESSIVE_TILT_SAMPLES) {
    events |= EVENT_PROGRESSIVE_TILT;
  }

  if (motion.accelMagnitudeG < SENSOR_DISTURBANCE_LOW_G ||
      motion.accelMagnitudeG > SENSOR_DISTURBANCE_HIGH_G) {
    events |= EVENT_SENSOR_DISTURBANCE;
  }

  if (motion.gravityDiffG > thresholds.shockG &&
      motion.rotationDps < thresholds.rotationDps * EARTHQUAKE_ROTATION_FACTOR) {
    events |= EVENT_EARTHQUAKE_LIKE;
  }

  if (motion.gravityDiffG > thresholds.gravityDiffG &&
      orientation.tiltVelocityDps < thresholds.tiltVelocityDps) {
    events |= EVENT_SLOW_LANDSLIDE;
  }

  if (orientation.tiltDeg > thresholds.warningTiltDeg &&
      orientation.tiltVelocityDps > thresholds.tiltVelocityDps) {
    events |= EVENT_SUDDEN_LANDSLIDE;
  }

  if (motion.gravityDiffG > thresholds.gravityDiffG * 0.6f &&
      motion.gravitySlopeGps > 0.0f &&
      orientation.tiltDeg > thresholds.watchTiltDeg) {
    events |= EVENT_CONTINUOUS_CREEP;
  }

  if (vibration.vibrationIndex > 65.0f &&
      orientation.tiltDeg < thresholds.watchTiltDeg &&
      motion.rotationDps < thresholds.rotationDps) {
    events |= EVENT_PIEZO_ENVIRONMENTAL;
  }

  if (vibration.peaksPerSecond >= PIEZO_REPEATED_PEAKS_PER_SECOND &&
      fusionConfidence < 35.0f) {
    events |= EVENT_PIEZO_MECHANICAL;
  }

  return events;
}

void printEventName(uint16_t eventBit) {
  switch (eventBit) {
    case EVENT_SUDDEN_IMPACT: Serial.print("Sudden impact"); break;
    case EVENT_CONTINUOUS_VIBRATION: Serial.print("Continuous vibration"); break;
    case EVENT_RAPID_TILT: Serial.print("Rapid tilt"); break;
    case EVENT_PROGRESSIVE_TILT: Serial.print("Progressive tilt"); break;
    case EVENT_SENSOR_DISTURBANCE: Serial.print("Sensor disturbance"); break;
    case EVENT_EARTHQUAKE_LIKE: Serial.print("Earthquake-like vibration"); break;
    case EVENT_SLOW_LANDSLIDE: Serial.print("Slow landslide"); break;
    case EVENT_SUDDEN_LANDSLIDE: Serial.print("Sudden landslide"); break;
    case EVENT_CONTINUOUS_CREEP: Serial.print("Continuous creep"); break;
    case EVENT_FIFO_OVERFLOW: Serial.print("FIFO overflow"); break;
    case EVENT_PIEZO_REPEATED_VIBRATION: Serial.print("Repeated piezo vibration"); break;
    case EVENT_PIEZO_CONTINUOUS_VIBRATION: Serial.print("Continuous piezo vibration"); break;
    case EVENT_PIEZO_IMPACT: Serial.print("Piezo impact"); break;
    case EVENT_PIEZO_ROCKFALL_LIKE: Serial.print("Rockfall-like impact"); break;
    case EVENT_PIEZO_ENVIRONMENTAL: Serial.print("Environmental vibration"); break;
    case EVENT_PIEZO_MECHANICAL: Serial.print("Mechanical vibration"); break;
  }
}

// ======================================================
// LOGGING AND OUTPUT
// ======================================================

void makeDataRecord(const SensorSample &sample, const PiezoSample &piezoSample,
                    const OrientationData &orientation, const MotionData &motion,
                    const VibrationData &vibration, float fusionConfidence,
                    float hazardIndex, uint16_t eventMask, DataRecord &record) {
  record.timestampMs = sample.timestampMs;
  record.accelRaw = sample.accelRaw;
  record.gyroRaw = sample.gyroRaw;
  record.accelMagnitudeG = motion.accelMagnitudeG;
  record.rollDeg = orientation.rollDeg;
  record.pitchDeg = orientation.pitchDeg;
  record.tiltDeg = orientation.tiltDeg;
  record.tiltVelocityDps = orientation.tiltVelocityDps;
  record.gravityDiffG = motion.gravityDiffG;
  record.gravitySlopeGps = motion.gravitySlopeGps;
  record.shockG = motion.shockG;
  record.rotationDps = motion.rotationDps;
  record.piezoRawAdc = piezoSample.rawAdc;
  record.piezoVoltage = piezoSample.voltage;
  record.piezoAmplitudeAdc = vibration.amplitudeAdc;
  record.piezoPeakAdc = vibration.peakAmplitudeAdc;
  record.piezoPeakCount = vibration.peakCount;
  record.piezoPeaksPerSecond = vibration.peaksPerSecond;
  record.vibrationIndex = vibration.vibrationIndex;
  record.vibrationState = vibration.state;
  record.fusionConfidence = fusionConfidence;
  record.hazardIndex = hazardIndex;
  record.state = currentState;
  record.eventMask = eventMask;
  record.newEventMask = eventMask & ~previousEventMask;
}

void printText(const DataRecord &record) {
#ifdef MODE_TEXT

  // Ringkasan utama (1 baris) - semua nilai mentah
  Serial.printf("[%8lu ms] ", (unsigned long)record.timestampMs);

  Serial.printf("ACC=%.4f,%.4f,%.4fg | ",
                record.accelRaw.x, record.accelRaw.y, record.accelRaw.z);

  Serial.printf("GYR=%.2f,%.2f,%.2fdps | ",
                record.gyroRaw.x, record.gyroRaw.y, record.gyroRaw.z);

  Serial.printf("TILT=%.2f\u00b0 | ", record.tiltDeg);

  Serial.printf("PIEZO=%u | ", record.piezoRawAdc);

  Serial.printf("HAZARD=%.0f | ", record.hazardIndex);

  Serial.printf("STATE=%s\n", stateName(record.state));

  // Hanya tampilkan event jika memang ada
  if (record.newEventMask != EVENT_NONE) {

    Serial.print("EVENT : ");

    bool first = true;

    for (uint32_t bit = 1; bit <= EVENT_LAST_BIT; bit <<= 1) {

      if (record.newEventMask & bit) {

        if (!first)
          Serial.print(", ");

        printEventName((uint16_t)bit);

        first = false;
      }
    }

    Serial.println();
  }

#endif
}

#ifdef MODE_PLOT
void printPlotHeader() {
  if (plotHeaderPrinted) return;
  plotHeaderPrinted = true;
}
#endif

void printPlotValue(const char *label, float value, uint8_t precision, bool lastValue) {
#ifdef MODE_PLOT
  Serial.print(label);
  Serial.print(':');
  Serial.print(value, precision);
  if (lastValue) {
    Serial.println();
  } else {
    Serial.print('\t');
  }
#endif
}

void printPlot(const DataRecord &record) {
#ifdef MODE_PLOT
  printPlotHeader();
  printPlotValue("AccelX_g", record.accelRaw.x, 5, false);
  printPlotValue("AccelY_g", record.accelRaw.y, 5, false);
  printPlotValue("AccelZ_g", record.accelRaw.z, 5, false);
  printPlotValue("GyroX_dps", record.gyroRaw.x, 3, false);
  printPlotValue("GyroY_dps", record.gyroRaw.y, 3, false);
  printPlotValue("GyroZ_dps", record.gyroRaw.z, 3, false);
  printPlotValue("Roll_deg", record.rollDeg, 3, false);
  printPlotValue("Pitch_deg", record.pitchDeg, 3, false);
  printPlotValue("Tilt_deg", record.tiltDeg, 3, false);
  printPlotValue("GravityDiff_g", record.gravityDiffG, 5, false);
  printPlotValue("Shock_g", record.shockG, 5, false);
  printPlotValue("Rotation_dps", record.rotationDps, 3, false);
  printPlotValue("PiezoRaw_adc", (float)record.piezoRawAdc, 0, false);
  printPlotValue("PiezoAmp_adc", record.piezoAmplitudeAdc, 2, false);
  printPlotValue("PiezoIndex", record.vibrationIndex, 2, false);
  printPlotValue("FusionConfidence", record.fusionConfidence, 2, false);
  printPlotValue("HazardIndex", record.hazardIndex, 2, true);
#endif
}

void logRecord(const DataRecord &record) {
  printText(record);
  printPlot(record);
}

// ======================================================
// NETWORK TELEMETRY (WiFi + MQTT) -- DataRecord channel only
// ======================================================
// The ESP32 publishes the FULL DataRecord as one JSON document on a single
// topic. Point the Node-RED "mqtt in" node at TOPIC_DATARECORD and adapt the
// "Route to Dashboard" / "Format for InfluxDB" functions to the new field
// names. Edit the values below before flashing.

const char *WIFI_SSID     = "vivo-Y33T";
const char *WIFI_PASSWORD = "ur3c00lp3rs0n";

// Host running the MQTT broker: the LAN IP of the Mosquitto/Node-RED machine,
// or a cloud broker hostname.
const char *MQTT_HOST = "10.191.99.25";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_USER = ""; // leave empty when the broker allows anonymous
const char *MQTT_PASS = "";

const char *DEVICE_ID        = "ews-esp32-01";
const char *TOPIC_DATARECORD = "landslide/ews-esp32-01/DataRecord";

// Publish throttle so the 10 Hz sample loop does not flood the broker.
// New events bypass the throttle so alerts are never delayed.
const uint32_t DATARECORD_PUBLISH_INTERVAL_MS = 1000;
const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
const uint32_t MQTT_RETRY_INTERVAL_MS = 5000;

WiFiClient wifiTransport;
PubSubClient mqttClient(wifiTransport);
uint32_t lastWifiAttemptMs = 0;
uint32_t lastMqttAttemptMs = 0;
uint32_t lastDataRecordPublishMs = 0;
bool wifiStarted = false;

// Non-blocking connection manager. Monitoring, the buzzer, and the hazard
// state machine keep working even when WiFi or the broker is down.
void maintainNetwork() {
  uint32_t tNowMs = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiStarted || tNowMs - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
      wifiStarted = true;
      lastWifiAttemptMs = tNowMs;
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#ifdef MODE_TEXT
      Serial.println("WIFI  : connecting...");
#endif
    }
    return;
  }

  if (!mqttClient.connected()) {
    if (lastMqttAttemptMs == 0 || tNowMs - lastMqttAttemptMs >= MQTT_RETRY_INTERVAL_MS) {
      lastMqttAttemptMs = tNowMs;
      mqttClient.setServer(MQTT_HOST, MQTT_PORT);
      // The full DataRecord payload is larger than the PubSubClient default.
      mqttClient.setBufferSize(1024);
      bool connected = mqttClient.connect(DEVICE_ID, MQTT_USER, MQTT_PASS);
      if (connected) {
#ifdef MODE_TEXT
        Serial.println("MQTT  : connected");
#endif
      }
    }
    return;
  }

  mqttClient.loop();
}

// Publish the complete DataRecord on the single DataRecord channel.
void publishDataRecord(const DataRecord &record) {
  if (!mqttClient.connected()) {
    return; // never block monitoring on the network
  }

  uint32_t tNowMs = millis();
  bool hasNewEvent = record.newEventMask != EVENT_NONE;
  if (!hasNewEvent &&
      tNowMs - lastDataRecordPublishMs < DATARECORD_PUBLISH_INTERVAL_MS) {
    return;
  }
  lastDataRecordPublishMs = tNowMs;

  JsonDocument doc; // ArduinoJson v7
  doc["device"]        = DEVICE_ID;
  doc["ts"]            = record.timestampMs;
  doc["accelX"]        = record.accelRaw.x;
  doc["accelY"]        = record.accelRaw.y;
  doc["accelZ"]        = record.accelRaw.z;
  doc["gyroX"]         = record.gyroRaw.x;
  doc["gyroY"]         = record.gyroRaw.y;
  doc["gyroZ"]         = record.gyroRaw.z;
  doc["accelMag"]      = record.accelMagnitudeG;
  doc["roll"]          = record.rollDeg;
  doc["pitch"]         = record.pitchDeg;
  doc["tilt"]          = record.tiltDeg;
  doc["tiltVelocity"]  = record.tiltVelocityDps;
  doc["gravityDiff"]   = record.gravityDiffG;
  doc["gravitySlope"]  = record.gravitySlopeGps;
  doc["shock"]         = record.shockG;
  doc["rotation"]      = record.rotationDps;
  doc["piezoRaw"]      = record.piezoRawAdc;
  doc["piezoVoltage"]  = record.piezoVoltage;
  doc["piezoAmp"]      = record.piezoAmplitudeAdc;
  doc["piezoPeak"]     = record.piezoPeakAdc;
  doc["piezoPeakCount"] = record.piezoPeakCount;
  doc["piezoPps"]      = record.piezoPeaksPerSecond;
  doc["vibIndex"]      = record.vibrationIndex;
  doc["vibState"]      = vibrationStateName(record.vibrationState);
  doc["confidence"]    = record.fusionConfidence;
  doc["hazard"]        = record.hazardIndex;
  doc["state"]         = stateName(record.state);
  doc["events"]        = record.eventMask;
  doc["newEvents"]     = record.newEventMask;
  doc["rssi"]          = WiFi.RSSI();

  char payload[1024];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  mqttClient.publish(TOPIC_DATARECORD, (const uint8_t *)payload, len, false);
}

// ======================================================
// ARDUINO SETUP AND LOOP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  configurePiezoInput();

  // Inisialisasi Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  configureIMUSettings();
  if (myIMU.begin() != 0) {
#ifdef MODE_TEXT
    Serial.println("IMU failed");
#endif
    while (1) {
      delay(1000);
    }
  }

  configureHardwareFeatures();

#ifdef USE_IMU_INTERRUPT
  pinMode(IMU_INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imuInterruptServiceRoutine, RISING);
#endif

  // Start WiFi early so it can associate while sensor calibration runs.
  maintainNetwork();

  calibrateSensor();
  calibratePiezo();
  calculateAdaptiveThresholds();
  lastSampleMs = millis();
}

void loop() {
  uint32_t nowMs = millis();

  // Keep WiFi/MQTT alive on every pass; this call never blocks sampling.
  maintainNetwork();

  bool sampleDue = (nowMs - lastSampleMs) >= SAMPLE_INTERVAL_MS;

#ifdef USE_IMU_INTERRUPT
  if (!imuInterruptFlag && !sampleDue) {
    delay(2);
    return;
  }
  imuInterruptFlag = false;
#else
  if (!sampleDue) {
    delay(2);
    return;
  }
#endif

  SensorSample sample;
  OrientationData orientation;
  MotionData motion;
  PiezoSample piezoSample;
  static VibrationData vibration = {0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f,
                                    VIBRATION_VERY_QUIET};

  readSensor(sample);
  readPiezo(piezoSample);
  checkFifoStatus();

  float dtSeconds = (sample.timestampMs - lastSampleMs) / 1000.0f;
  if (dtSeconds <= 0.0f || dtSeconds > 5.0f) {
    dtSeconds = SAMPLE_INTERVAL_MS / 1000.0f;
  }
  lastSampleMs = sample.timestampMs;

  // All calculations run directly on the raw sample. No filtering anywhere.
  calculateTilt(sample, dtSeconds, orientation);
  calculateMotion(sample, dtSeconds, motion);
  calculateVibration(piezoSample, dtSeconds, vibration);

  // === Cek auto-calibration ===
  checkAutoCalibration(sample, orientation, motion, sample.timestampMs);

  float fusionConfidence = calculateFusionConfidence(orientation, motion, vibration);
  float hazardIndex = calculateHazard(orientation, motion, vibration, fusionConfidence);
  updateState(hazardIndex, sample.timestampMs);

  uint16_t eventMask = detectEvents(orientation, motion, vibration, fusionConfidence);

  DataRecord record;
  makeDataRecord(sample, piezoSample, orientation, motion, vibration,
                 fusionConfidence, hazardIndex, eventMask, record);
  logRecord(record);
  publishDataRecord(record);

  // === Kontrol Buzzer ===
  controlBuzzer();

  previousTiltDeg = orientation.tiltDeg;
  previousGravityDiffG = motion.gravityDiffG;
  previousPiezoAmplitudeAdc = vibration.amplitudeAdc;
  previousEventMask = eventMask;
}
