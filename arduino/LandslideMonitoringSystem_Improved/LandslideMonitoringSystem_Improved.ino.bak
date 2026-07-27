// LandslideMonitoringSystem_Improved.ino
// Target: ESP32 + SparkFun LSM6DS3 IMU
//
// This sketch keeps the original project architecture: read IMU data, filter it,
// estimate tilt, calculate gravity/shock/rotation/trend signals, classify hazard,
// and print either text diagnostics or Arduino Serial Plotter data.
//
// The monitoring logic is intentionally separated from event detection. The
// hazard finite-state machine uses hysteresis so noisy samples do not cause
// alert chatter, while event flags preserve important short-lived signals.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "SparkFunLSM6DS3.h"

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
// FILTER SELECTION
// ======================================================
// Each filter can be enabled or disabled at compile time.
// Low-pass estimates gravity. High-pass estimates vibration. Moving average
// reduces random noise. Median rejects single-sample spikes.

#define USE_LOWPASS
#define USE_MOVING_AVERAGE
#define USE_MEDIAN
#define USE_HIGHPASS

// Piezo vibration filtering is kept separate from the IMU filters because the
// SEN0209 signal is a one-dimensional analog waveform, not a gravity vector.
#define USE_PIEZO_MOVING_AVERAGE
#define USE_PIEZO_MEDIAN
#define USE_PIEZO_HIGHPASS

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

// DFRobot SEN0209 piezo film analog output. Use an ADC-capable ESP32 input-only
// pin where possible. The digital output can be wired later, but analog is the
// primary measurement for research-quality vibration intensity.
const uint8_t PIEZO_ANALOG_PIN = 34;
const uint8_t ADC_RESOLUTION_BITS = 12;
const uint16_t ADC_MAX_COUNT = 4095;
const float ADC_REFERENCE_VOLTAGE = 3.3f;

// IMU output configuration. The SparkFun library accepts these human-readable
// values and converts them to sensor register codes in begin().
const uint16_t IMU_ODR_HZ = 104;          // Output Data Rate in Hz.
const uint16_t ACCEL_FULL_SCALE_G = 2;    // Valid library ranges: 2, 4, 8, 16 g.
const uint16_t GYRO_FULL_SCALE_DPS = 245; // Valid ranges: 125/245/500/1000/2000.

// Main loop period. For long deployments, keep this no faster than needed to
// reduce log size and power use. The IMU can still sample faster internally.
const uint16_t SAMPLE_INTERVAL_MS = 100;

// Calibration. Samples are collected while the sensor is still. The standard
// deviations measured here become the baseline for adaptive thresholds.
const int CAL_SAMPLES = 300;
const uint16_t CAL_SAMPLE_DELAY_MS = 10;
const float STILLNESS_ACCEL_STD_GOOD = 0.006f; // g, empirical quality reference.
const float STILLNESS_GYRO_STD_GOOD = 0.35f;   // dps, empirical quality reference.

// Complementary filter. Higher values trust the gyro over short periods and the
// accelerometer over long periods. This smooths tilt while limiting gyro drift.
const float COMPLEMENTARY_ALPHA = 0.98f;

// Low-pass and high-pass filter constants. Smaller low-pass alpha gives a
// smoother gravity estimate. High-pass alpha near 1 preserves vibration changes.
const float LOWPASS_ALPHA = 0.18f;
const float HIGHPASS_ALPHA = 0.85f;

// Moving average and median windows. Odd median windows give a true center
// sample. These small windows are cheap on ESP32 and keep latency modest.
const uint8_t MOVING_AVERAGE_WINDOW = 8;
const uint8_t MEDIAN_WINDOW = 5;

// Piezo windows. Energy is the moving sum of absolute vibration amplitude.
// RMS is calculated from the same window using the square mean.
const uint8_t PIEZO_MOVING_AVERAGE_WINDOW = 8;
const uint8_t PIEZO_MEDIAN_WINDOW = 5;
const uint8_t PIEZO_ENERGY_WINDOW = 20;
const float PIEZO_HIGHPASS_ALPHA = 0.92f;

// Adaptive threshold multipliers. Threshold = learned mean + K * learned stddev.
// Minimums stop thresholds from becoming hypersensitive after a very quiet
// calibration. Maximums catch bad calibrations and keep scale predictable.
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

// Trend smoothing. Short trend reacts to recent movement. Long trend represents
// slow background deformation. Their slopes are useful for creep detection.
const float SHORT_TREND_ALPHA = 0.12f;
const float LONG_TREND_ALPHA = 0.015f;
const float PIEZO_SHORT_TREND_ALPHA = 0.20f;
const float PIEZO_LONG_TREND_ALPHA = 0.04f;

// Weighted hazard index. Keep the weights summing to 100 for easy interpretation.
const float WEIGHT_TILT = 30.0f;
const float WEIGHT_ROTATION = 15.0f;
const float WEIGHT_SHOCK = 10.0f;
const float WEIGHT_LONG_DRIFT = 15.0f;
const float WEIGHT_GRAVITY = 10.0f;
const float WEIGHT_PIEZO_VIBRATION = 15.0f;
const float WEIGHT_SENSOR_FUSION = 5.0f;

// Hazard-index class thresholds. The finite-state machine adds sample-count
// hysteresis on top of these values.
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
const float PIEZO_INCREASING_SLOPE_ADC_PER_SECOND = 3.0f;

// FIFO and wake/activity register settings. The LSM6DS3 register values are
// intentionally localized so they can be revised if a board uses a different
// interrupt route.
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
  Vec3 accel;
  Vec3 gyro;
};

struct PiezoSample {
  uint32_t timestampMs;
  uint16_t rawAdc;
  float voltage;
};

struct FilteredData {
  Vec3 accelForTilt;
  Vec3 gravity;
  Vec3 vibration;
  float accelMagnitude;
  float vibrationMagnitude;
};

struct PiezoFilteredData {
  float smoothedAdc;
  float highpassAdc;
  float amplitudeAdc;
};

struct OrientationData {
  float rollDeg;
  float pitchDeg;
  float tiltDeg;
  float tiltVelocityDps;
};

struct MotionData {
  float gravityDiffG;
  float shockG;
  float rotationDps;
};

struct TrendData {
  float shortGravityTrend;
  float longGravityTrend;
  float gravitySlopeGps;
  float gravityRateGps;
  float tiltRateDps;
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
  float energyAdc;
  float rmsAdc;
};

struct CalibrationData {
  Vec3 accelMeanRaw;
  Vec3 gyroMeanRaw;
  Vec3 accelStdRaw;
  Vec3 gyroStdRaw;
  Vec3 accelOffset;
  Vec3 gyroBias;
  Vec3 gravityReference;
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
  float amplitudeAdc;
  float rmsAdc;
  float energyAdc;
  float peakAmplitudeAdc;
  uint16_t peakCount;
  float peaksPerSecond;
  float vibrationIndex;
  float shortTrend;
  float longTrend;
  float trendSlopeAdcPerSecond;
  float rateAdcPerSecond;
  VibrationState state;
};

struct DataRecord {
  uint32_t timestampMs;
  Vec3 accelRaw;
  Vec3 gyroRaw;
  Vec3 accelFiltered;
  Vec3 vibration;
  float rollDeg;
  float pitchDeg;
  float tiltDeg;
  float tiltVelocityDps;
  float gravityDiffG;
  float shockG;
  float rotationDps;
  float shortTrend;
  float longTrend;
  float gravitySlopeGps;
  float gravityRateGps;
  float vibrationAmplitudeAdc;
  float vibrationRmsAdc;
  float vibrationEnergyAdc;
  float vibrationPeakAdc;
  uint16_t vibrationPeakCount;
  float vibrationPeaksPerSecond;
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
bool filtersInitialized = false;
uint32_t lastSampleMs = 0;
uint32_t lastTransitionMs = 0;

CalibrationData calibration;
AdaptiveThresholds thresholds;
PiezoCalibrationData piezoCalibration;
PiezoThresholds piezoThresholds;
AlertState currentState = STATE_NORMAL;
AlertState lastPrintedState = STATE_NORMAL;

Vec3 gravityLp = {0.0f, 0.0f, 0.0f};
Vec3 highpassPrevInput = {0.0f, 0.0f, 0.0f};
Vec3 highpassPrevOutput = {0.0f, 0.0f, 0.0f};

Vec3 movingBuffer[MOVING_AVERAGE_WINDOW];
Vec3 movingSum = {0.0f, 0.0f, 0.0f};
uint8_t movingIndex = 0;
uint8_t movingCount = 0;

Vec3 medianBuffer[MEDIAN_WINDOW];
uint8_t medianIndex = 0;
uint8_t medianCount = 0;

float piezoMovingBuffer[PIEZO_MOVING_AVERAGE_WINDOW];
float piezoMovingSum = 0.0f;
uint8_t piezoMovingIndex = 0;
uint8_t piezoMovingCount = 0;

float piezoMedianBuffer[PIEZO_MEDIAN_WINDOW];
uint8_t piezoMedianIndex = 0;
uint8_t piezoMedianCount = 0;

float piezoEnergyBuffer[PIEZO_ENERGY_WINDOW];
float piezoSquareBuffer[PIEZO_ENERGY_WINDOW];
float piezoEnergySum = 0.0f;
float piezoSquareSum = 0.0f;
uint8_t piezoEnergyIndex = 0;
uint8_t piezoEnergyCount = 0;

float piezoHighpassPrevInput = 0.0f;
float piezoHighpassPrevOutput = 0.0f;
float previousPiezoAmplitudeAdc = 0.0f;
float previousPiezoLongTrend = 0.0f;
uint16_t piezoTotalPeakCount = 0;
uint16_t piezoPeakWindowCount = 0;
uint32_t piezoPeakWindowStartMs = 0;
float piezoPeaksPerSecond = 0.0f;

OrientationData previousOrientation = {0.0f, 0.0f, 0.0f, 0.0f};
float previousGravityDiffG = 0.0f;
float previousLongTrend = 0.0f;

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

Vec3 vecAdd(Vec3 a, Vec3 b) {
  Vec3 out = {a.x + b.x, a.y + b.y, a.z + b.z};
  return out;
}

Vec3 vecSub(Vec3 a, Vec3 b) {
  Vec3 out = {a.x - b.x, a.y - b.y, a.z - b.z};
  return out;
}

Vec3 vecScale(Vec3 v, float scale) {
  Vec3 out = {v.x * scale, v.y * scale, v.z * scale};
  return out;
}

Vec3 vecLerp(Vec3 previous, Vec3 input, float alpha) {
  Vec3 out = {
    previous.x + alpha * (input.x - previous.x),
    previous.y + alpha * (input.y - previous.y),
    previous.z + alpha * (input.z - previous.z)
  };
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

// Configure FIFO, data-ready interrupt, and wake/activity detection. These
// hardware features reduce missed transients and provide an interrupt-driven
// path without using tap, double-tap, pedometer, or 6D orientation features.
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

uint16_t readFifoSampleCount() {
  uint8_t status1 = 0;
  uint8_t status2 = 0;
  if (!readImuRegister(0x3A, status1)) return 0;
  if (!readImuRegister(0x3B, status2)) return 0;
  return ((uint16_t)(status2 & 0x0F) << 8) | status1;
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

void readSensor(SensorSample &sample) {
  sample.timestampMs = millis();
  sample.accelRaw.x = myIMU.readFloatAccelX();
  sample.accelRaw.y = myIMU.readFloatAccelY();
  sample.accelRaw.z = myIMU.readFloatAccelZ();
  sample.gyroRaw.x = myIMU.readFloatGyroX();
  sample.gyroRaw.y = myIMU.readFloatGyroY();
  sample.gyroRaw.z = myIMU.readFloatGyroZ();

  // Correct acceleration to a calibrated 1 g reference vector while preserving
  // physical changes in gravity direction. Gyro bias is subtracted directly.
  sample.accel = vecSub(sample.accelRaw, calibration.accelOffset);
  sample.gyro = vecSub(sample.gyroRaw, calibration.gyroBias);
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
// CALIBRATION
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
  Serial.println("CALIBRATION");
  Serial.println("------------------------------------------------------");
  Serial.printf("Samples              : %d\n", CAL_SAMPLES);
  Serial.printf("Accel mean raw       : %8.5f %8.5f %8.5f g\n",
                calibration.accelMeanRaw.x, calibration.accelMeanRaw.y, calibration.accelMeanRaw.z);
  Serial.printf("Accel std raw        : %8.5f %8.5f %8.5f g\n",
                calibration.accelStdRaw.x, calibration.accelStdRaw.y, calibration.accelStdRaw.z);
  Serial.printf("Accel offsets        : %8.5f %8.5f %8.5f g\n",
                calibration.accelOffset.x, calibration.accelOffset.y, calibration.accelOffset.z);
  Serial.printf("Gyro bias            : %8.4f %8.4f %8.4f dps\n",
                calibration.gyroBias.x, calibration.gyroBias.y, calibration.gyroBias.z);
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
  calibration.gyroBias = calibration.gyroMeanRaw;
  calibration.gravityMagnitudeG = vecMagnitude(calibration.accelMeanRaw);

  Vec3 unitGravity = vecScale(calibration.accelMeanRaw,
                              safeDivide(1.0f, calibration.gravityMagnitudeG, 1.0f));
  calibration.accelOffset = vecSub(calibration.accelMeanRaw, unitGravity);
  calibration.gravityReference = unitGravity;
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

  gravityLp = calibration.gravityReference;
  highpassPrevInput = calibration.gravityReference;
  highpassPrevOutput = {0.0f, 0.0f, 0.0f};
  previousOrientation.rollDeg = calibration.initialRollDeg;
  previousOrientation.pitchDeg = calibration.initialPitchDeg;
  filtersInitialized = true;
  orientationInitialized = true;

  printCalibrationInfo();
}

void printPiezoCalibrationInfo() {
#ifdef MODE_TEXT
  Serial.println("PIEZO CALIBRATION");
  Serial.println("------------------------------------------------------");
  Serial.printf("Analog pin           : GPIO %u\n", PIEZO_ANALOG_PIN);
  Serial.printf("Baseline             : %8.2f ADC, %6.3f V\n",
                piezoCalibration.baselineAdc, piezoCalibration.baselineVoltage);
  Serial.printf("Noise std / mean abs : %8.3f %8.3f ADC\n",
                piezoCalibration.noiseStdAdc, piezoCalibration.noiseMeanAbsAdc);
  Serial.printf("Amplitude L/M/H/S    : %8.2f %8.2f %8.2f %8.2f ADC\n",
                piezoThresholds.lowAmplitudeAdc, piezoThresholds.moderateAmplitudeAdc,
                piezoThresholds.highAmplitudeAdc, piezoThresholds.severeAmplitudeAdc);
  Serial.printf("Energy / RMS thresh  : %8.2f %8.2f ADC\n",
                piezoThresholds.energyAdc, piezoThresholds.rmsAdc);
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
  piezoThresholds.energyAdc = piezoThresholds.moderateAmplitudeAdc * PIEZO_ENERGY_WINDOW;
  piezoThresholds.rmsAdc = piezoThresholds.moderateAmplitudeAdc;
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

  piezoHighpassPrevInput = piezoCalibration.baselineAdc;
  piezoHighpassPrevOutput = 0.0f;
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

void checkAutoCalibration(const FilteredData &filtered, const OrientationData &orientation, const MotionData &motion, uint32_t nowMs) {
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
      calibration.gravityReference = filtered.gravity;

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
// FILTERING
// ======================================================

Vec3 updateMovingAverage(Vec3 input) {
  if (movingCount < MOVING_AVERAGE_WINDOW) {
    movingBuffer[movingIndex] = input;
    movingSum = vecAdd(movingSum, input);
    movingCount++;
  } else {
    movingSum = vecSub(movingSum, movingBuffer[movingIndex]);
    movingBuffer[movingIndex] = input;
    movingSum = vecAdd(movingSum, input);
  }
  movingIndex = (movingIndex + 1) % MOVING_AVERAGE_WINDOW;
  return vecScale(movingSum, safeDivide(1.0f, movingCount, 1.0f));
}

float medianOfArray(float values[], uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }
  return values[count / 2];
}

Vec3 updateMedianFilter(Vec3 input) {
  medianBuffer[medianIndex] = input;
  medianIndex = (medianIndex + 1) % MEDIAN_WINDOW;
  if (medianCount < MEDIAN_WINDOW) medianCount++;

  float xs[MEDIAN_WINDOW];
  float ys[MEDIAN_WINDOW];
  float zs[MEDIAN_WINDOW];
  for (uint8_t i = 0; i < medianCount; i++) {
    xs[i] = medianBuffer[i].x;
    ys[i] = medianBuffer[i].y;
    zs[i] = medianBuffer[i].z;
  }

  Vec3 out = {
    medianOfArray(xs, medianCount),
    medianOfArray(ys, medianCount),
    medianOfArray(zs, medianCount)
  };
  return out;
}

Vec3 updateHighpass(Vec3 input) {
  Vec3 out = {
    HIGHPASS_ALPHA * (highpassPrevOutput.x + input.x - highpassPrevInput.x),
    HIGHPASS_ALPHA * (highpassPrevOutput.y + input.y - highpassPrevInput.y),
    HIGHPASS_ALPHA * (highpassPrevOutput.z + input.z - highpassPrevInput.z)
  };
  highpassPrevInput = input;
  highpassPrevOutput = out;
  return out;
}

void filterData(const SensorSample &sample, FilteredData &filtered) {
  Vec3 workingAccel = sample.accel;

#ifdef USE_MEDIAN
  workingAccel = updateMedianFilter(workingAccel);
#endif

#ifdef USE_MOVING_AVERAGE
  workingAccel = updateMovingAverage(workingAccel);
#endif

#ifdef USE_LOWPASS
  if (!filtersInitialized) {
    gravityLp = workingAccel;
    filtersInitialized = true;
  } else {
    gravityLp = vecLerp(gravityLp, workingAccel, LOWPASS_ALPHA);
  }
  filtered.gravity = gravityLp;
  filtered.accelForTilt = gravityLp;
#else
  filtered.gravity = workingAccel;
  filtered.accelForTilt = workingAccel;
#endif

#ifdef USE_HIGHPASS
  filtered.vibration = updateHighpass(sample.accel);
#else
  filtered.vibration = vecSub(sample.accel, filtered.gravity);
#endif

  filtered.accelMagnitude = vecMagnitude(sample.accel);
  filtered.vibrationMagnitude = vecMagnitude(filtered.vibration);
}

float updatePiezoMovingAverage(float input) {
  if (piezoMovingCount < PIEZO_MOVING_AVERAGE_WINDOW) {
    piezoMovingBuffer[piezoMovingIndex] = input;
    piezoMovingSum += input;
    piezoMovingCount++;
  } else {
    piezoMovingSum -= piezoMovingBuffer[piezoMovingIndex];
    piezoMovingBuffer[piezoMovingIndex] = input;
    piezoMovingSum += input;
  }
  piezoMovingIndex = (piezoMovingIndex + 1) % PIEZO_MOVING_AVERAGE_WINDOW;
  return safeDivide(piezoMovingSum, piezoMovingCount, input);
}

float updatePiezoMedian(float input) {
  piezoMedianBuffer[piezoMedianIndex] = input;
  piezoMedianIndex = (piezoMedianIndex + 1) % PIEZO_MEDIAN_WINDOW;
  if (piezoMedianCount < PIEZO_MEDIAN_WINDOW) piezoMedianCount++;

  float values[PIEZO_MEDIAN_WINDOW];
  for (uint8_t i = 0; i < piezoMedianCount; i++) {
    values[i] = piezoMedianBuffer[i];
  }
  return medianOfArray(values, piezoMedianCount);
}

float updatePiezoHighpass(float input) {
  float output = PIEZO_HIGHPASS_ALPHA * (piezoHighpassPrevOutput + input - piezoHighpassPrevInput);
  piezoHighpassPrevInput = input;
  piezoHighpassPrevOutput = output;
  return output;
}

void filterPiezo(const PiezoSample &sample, PiezoFilteredData &filtered) {
  float workingAdc = sample.rawAdc;

#ifdef USE_PIEZO_MOVING_AVERAGE
  workingAdc = updatePiezoMovingAverage(workingAdc);
#endif

#ifdef USE_PIEZO_MEDIAN
  workingAdc = updatePiezoMedian(workingAdc);
#endif

  filtered.smoothedAdc = workingAdc;

#ifdef USE_PIEZO_HIGHPASS
  filtered.highpassAdc = updatePiezoHighpass(workingAdc);
  filtered.amplitudeAdc = fabsf(filtered.highpassAdc);
#else
  filtered.highpassAdc = workingAdc - piezoCalibration.baselineAdc;
  filtered.amplitudeAdc = fabsf(filtered.highpassAdc);
#endif
}

// ======================================================
// ORIENTATION, MOTION, AND TREND
// ======================================================

void calculateTilt(const SensorSample &sample, const FilteredData &filtered,
                   float dtSeconds, OrientationData &orientation) {
  float rollAccel = calculateRollFromAccel(filtered.accelForTilt);
  float pitchAccel = calculatePitchFromAccel(filtered.accelForTilt);

  if (!orientationInitialized || dtSeconds <= 0.0f) {
    orientation.rollDeg = rollAccel;
    orientation.pitchDeg = pitchAccel;
    orientationInitialized = true;
  } else {
    float gyroRoll = previousOrientation.rollDeg + sample.gyro.x * dtSeconds;
    float gyroPitch = previousOrientation.pitchDeg + sample.gyro.y * dtSeconds;
    orientation.rollDeg = COMPLEMENTARY_ALPHA * gyroRoll + (1.0f - COMPLEMENTARY_ALPHA) * rollAccel;
    orientation.pitchDeg = COMPLEMENTARY_ALPHA * gyroPitch + (1.0f - COMPLEMENTARY_ALPHA) * pitchAccel;
  }

  float rollDelta = angleDifferenceDeg(orientation.rollDeg, calibration.initialRollDeg);
  float pitchDelta = angleDifferenceDeg(orientation.pitchDeg, calibration.initialPitchDeg);
  orientation.tiltDeg = sqrtf(squareFloat(rollDelta) + squareFloat(pitchDelta));
  orientation.tiltVelocityDps = fabsf(orientation.tiltDeg - previousOrientation.tiltDeg) /
                                fmaxf(dtSeconds, 0.001f);
}

void calculateGravity(const FilteredData &filtered, MotionData &motion) {
  Vec3 gravityDelta = vecSub(filtered.gravity, calibration.gravityReference);
  motion.gravityDiffG = vecMagnitude(gravityDelta);
}

void calculateShock(const FilteredData &filtered, MotionData &motion) {
  motion.shockG = fabsf(filtered.accelMagnitude - 1.0f);
}

void calculateRotation(const SensorSample &sample, MotionData &motion) {
  motion.rotationDps = vecMagnitude(sample.gyro);
}

void calculateTrend(const OrientationData &orientation, const MotionData &motion,
                    float dtSeconds, TrendData &trend) {
  trend.shortGravityTrend = SHORT_TREND_ALPHA * motion.gravityDiffG +
                            (1.0f - SHORT_TREND_ALPHA) * trend.shortGravityTrend;
  trend.longGravityTrend = LONG_TREND_ALPHA * motion.gravityDiffG +
                           (1.0f - LONG_TREND_ALPHA) * trend.longGravityTrend;
  trend.gravitySlopeGps = (trend.longGravityTrend - previousLongTrend) / fmaxf(dtSeconds, 0.001f);
  trend.gravityRateGps = fabsf(motion.gravityDiffG - previousGravityDiffG) / fmaxf(dtSeconds, 0.001f);
  trend.tiltRateDps = orientation.tiltVelocityDps;
}

float normalizedScore(float value, float start, float fullScale);

VibrationState classifyVibration(float vibrationIndex) {
  if (vibrationIndex >= 85.0f) return VIBRATION_SEVERE;
  if (vibrationIndex >= 65.0f) return VIBRATION_HIGH;
  if (vibrationIndex >= 35.0f) return VIBRATION_MODERATE;
  if (vibrationIndex >= 10.0f) return VIBRATION_LOW;
  return VIBRATION_VERY_QUIET;
}

float calculateVibrationIndex(float amplitudeAdc, float rmsAdc, float energyAdc) {
  float amplitudeScore = normalizedScore(amplitudeAdc,
                                         piezoThresholds.lowAmplitudeAdc,
                                         piezoThresholds.severeAmplitudeAdc);
  float rmsScore = normalizedScore(rmsAdc,
                                   piezoThresholds.rmsAdc,
                                   piezoThresholds.severeAmplitudeAdc);
  float energyScore = normalizedScore(energyAdc,
                                      piezoThresholds.energyAdc,
                                      piezoThresholds.severeAmplitudeAdc * PIEZO_ENERGY_WINDOW);
  return clampFloat(fmaxf(amplitudeScore, fmaxf(rmsScore, energyScore)), 0.0f, 100.0f);
}

void updatePiezoEnergy(float amplitudeAdc, VibrationData &vibration) {
  float square = squareFloat(amplitudeAdc);
  if (piezoEnergyCount < PIEZO_ENERGY_WINDOW) {
    piezoEnergyBuffer[piezoEnergyIndex] = amplitudeAdc;
    piezoSquareBuffer[piezoEnergyIndex] = square;
    piezoEnergySum += amplitudeAdc;
    piezoSquareSum += square;
    piezoEnergyCount++;
  } else {
    piezoEnergySum -= piezoEnergyBuffer[piezoEnergyIndex];
    piezoSquareSum -= piezoSquareBuffer[piezoEnergyIndex];
    piezoEnergyBuffer[piezoEnergyIndex] = amplitudeAdc;
    piezoSquareBuffer[piezoEnergyIndex] = square;
    piezoEnergySum += amplitudeAdc;
    piezoSquareSum += square;
  }
  piezoEnergyIndex = (piezoEnergyIndex + 1) % PIEZO_ENERGY_WINDOW;

  vibration.energyAdc = piezoEnergySum;
  vibration.rmsAdc = sqrtf(safeDivide(piezoSquareSum, piezoEnergyCount, 0.0f));
}

void updatePiezoPeaks(float amplitudeAdc, uint32_t timestampMs, VibrationData &vibration) {
  vibration.peakAmplitudeAdc = fmaxf(vibration.peakAmplitudeAdc * 0.92f, amplitudeAdc);

  bool crossedPeakThreshold = amplitudeAdc >= piezoThresholds.moderateAmplitudeAdc &&
                              previousPiezoAmplitudeAdc < piezoThresholds.moderateAmplitudeAdc;
  if (crossedPeakThreshold) {
    piezoTotalPeakCount++;
    piezoPeakWindowCount++;
  }

  uint32_t elapsedMs = timestampMs - piezoPeakWindowStartMs;
  if (elapsedMs >= 1000) {
    piezoPeaksPerSecond = piezoPeakWindowCount * 1000.0f / elapsedMs;
    piezoPeakWindowCount = 0;
    piezoPeakWindowStartMs = timestampMs;
  }

  vibration.peakCount = piezoTotalPeakCount;
  vibration.peaksPerSecond = piezoPeaksPerSecond;
}

void calculateVibration(const PiezoSample &sample, const PiezoFilteredData &filtered,
                        float dtSeconds, VibrationData &vibration) {
  vibration.amplitudeAdc = filtered.amplitudeAdc;
  updatePiezoEnergy(vibration.amplitudeAdc, vibration);
  updatePiezoPeaks(vibration.amplitudeAdc, sample.timestampMs, vibration);

  vibration.vibrationIndex = calculateVibrationIndex(vibration.amplitudeAdc,
                                                     vibration.rmsAdc,
                                                     vibration.energyAdc);
  vibration.shortTrend = PIEZO_SHORT_TREND_ALPHA * vibration.vibrationIndex +
                         (1.0f - PIEZO_SHORT_TREND_ALPHA) * vibration.shortTrend;
  vibration.longTrend = PIEZO_LONG_TREND_ALPHA * vibration.vibrationIndex +
                        (1.0f - PIEZO_LONG_TREND_ALPHA) * vibration.longTrend;
  vibration.trendSlopeAdcPerSecond = (vibration.longTrend - previousPiezoLongTrend) /
                                     fmaxf(dtSeconds, 0.001f);
  vibration.rateAdcPerSecond = fabsf(vibration.amplitudeAdc - previousPiezoAmplitudeAdc) /
                               fmaxf(dtSeconds, 0.001f);
  vibration.state = classifyVibration(vibration.vibrationIndex);
}

float normalizedScore(float value, float start, float fullScale) {
  if (value <= start) return 0.0f;
  if (value >= fullScale) return 100.0f;
  return 100.0f * (value - start) / fmaxf(fullScale - start, 0.001f);
}

float calculateFusionConfidence(const OrientationData &orientation, const MotionData &motion,
                                const TrendData &trend, const VibrationData &vibration) {
  float confidence = 0.0f;

  if (orientation.tiltDeg > thresholds.watchTiltDeg) confidence += 20.0f;
  if (trend.longGravityTrend > thresholds.gravityDiffG * 0.6f) confidence += 20.0f;
  if (motion.rotationDps > thresholds.rotationDps) confidence += 15.0f;
  if (vibration.vibrationIndex > 35.0f) confidence += 20.0f;
  if (vibration.trendSlopeAdcPerSecond > PIEZO_INCREASING_SLOPE_ADC_PER_SECOND) confidence += 10.0f;
  if (orientation.tiltVelocityDps > thresholds.tiltVelocityDps) confidence += 15.0f;

  if (vibration.vibrationIndex > 65.0f &&
      orientation.tiltDeg < thresholds.watchTiltDeg &&
      motion.rotationDps < thresholds.rotationDps) {
    confidence *= 0.45f;
  }

  return clampFloat(confidence, 0.0f, 100.0f);
}

float calculateHazard(const OrientationData &orientation, const MotionData &motion,
                      const TrendData &trend, const VibrationData &vibration,
                      float fusionConfidence) {
  float tiltScore = normalizedScore(orientation.tiltDeg,
                                    thresholds.watchTiltDeg,
                                    thresholds.dangerTiltDeg);
  float rotationScore = normalizedScore(motion.rotationDps,
                                        thresholds.rotationDps,
                                        thresholds.rotationDps * 3.0f);
  float shockScore = normalizedScore(motion.shockG,
                                     thresholds.shockG,
                                     thresholds.shockG * 4.0f);
  float driftScore = normalizedScore(trend.longGravityTrend,
                                     thresholds.gravityDiffG * 0.5f,
                                     thresholds.gravityDiffG * 2.5f);
  float gravityScore = normalizedScore(motion.gravityDiffG,
                                       thresholds.gravityDiffG,
                                       thresholds.gravityDiffG * 3.0f);
  float vibrationScore = vibration.vibrationIndex;

  float weighted = (WEIGHT_TILT * tiltScore +
                    WEIGHT_ROTATION * rotationScore +
                    WEIGHT_SHOCK * shockScore +
                    WEIGHT_LONG_DRIFT * driftScore +
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

uint16_t detectEvents(const SensorSample &sample, const FilteredData &filtered,
                      const OrientationData &orientation, const MotionData &motion,
                      const TrendData &trend, const VibrationData &vibration,
                      float fusionConfidence) {
  uint16_t events = hardwareEventMask;
  hardwareEventMask = EVENT_NONE;

  if (motion.shockG > thresholds.shockG) {
    events |= EVENT_SUDDEN_IMPACT;
  }

  if (filtered.vibrationMagnitude > thresholds.shockG * VIBRATION_FACTOR) {
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
      trend.gravitySlopeGps > thresholds.gravityRateGps * 0.25f) {
    progressiveTiltCount++;
  } else {
    progressiveTiltCount = 0;
  }
  if (progressiveTiltCount >= PROGRESSIVE_TILT_SAMPLES) {
    events |= EVENT_PROGRESSIVE_TILT;
  }

  if (filtered.accelMagnitude < SENSOR_DISTURBANCE_LOW_G ||
      filtered.accelMagnitude > SENSOR_DISTURBANCE_HIGH_G) {
    events |= EVENT_SENSOR_DISTURBANCE;
  }

  if (filtered.vibrationMagnitude > thresholds.shockG &&
      motion.rotationDps < thresholds.rotationDps * EARTHQUAKE_ROTATION_FACTOR) {
    events |= EVENT_EARTHQUAKE_LIKE;
  }

  if (trend.longGravityTrend > thresholds.gravityDiffG &&
      orientation.tiltVelocityDps < thresholds.tiltVelocityDps) {
    events |= EVENT_SLOW_LANDSLIDE;
  }

  if (orientation.tiltDeg > thresholds.warningTiltDeg &&
      orientation.tiltVelocityDps > thresholds.tiltVelocityDps) {
    events |= EVENT_SUDDEN_LANDSLIDE;
  }

  if (trend.longGravityTrend > thresholds.gravityDiffG * 0.6f &&
      trend.gravitySlopeGps > 0.0f &&
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

void makeDataRecord(const SensorSample &sample, const FilteredData &filtered,
                    const OrientationData &orientation, const MotionData &motion,
                    const TrendData &trend, const VibrationData &vibration,
                    float fusionConfidence, float hazardIndex,
                    uint16_t eventMask, DataRecord &record) {
  record.timestampMs = sample.timestampMs;
  record.accelRaw = sample.accelRaw;
  record.gyroRaw = sample.gyroRaw;
  record.accelFiltered = filtered.accelForTilt;
  record.vibration = filtered.vibration;
  record.rollDeg = orientation.rollDeg;
  record.pitchDeg = orientation.pitchDeg;
  record.tiltDeg = orientation.tiltDeg;
  record.tiltVelocityDps = orientation.tiltVelocityDps;
  record.gravityDiffG = motion.gravityDiffG;
  record.shockG = motion.shockG;
  record.rotationDps = motion.rotationDps;
  record.shortTrend = trend.shortGravityTrend;
  record.longTrend = trend.longGravityTrend;
  record.gravitySlopeGps = trend.gravitySlopeGps;
  record.gravityRateGps = trend.gravityRateGps;
  record.vibrationAmplitudeAdc = vibration.amplitudeAdc;
  record.vibrationRmsAdc = vibration.rmsAdc;
  record.vibrationEnergyAdc = vibration.energyAdc;
  record.vibrationPeakAdc = vibration.peakAmplitudeAdc;
  record.vibrationPeakCount = vibration.peakCount;
  record.vibrationPeaksPerSecond = vibration.peaksPerSecond;
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

  // Ringkasan utama (1 baris)
  Serial.printf("[%8lu ms] ",
                (unsigned long)record.timestampMs);

  Serial.printf("TILT=%.2f° | ",
                record.tiltDeg);

  Serial.printf("GRAV=%.3fg | ",
                record.gravityDiffG);

  Serial.printf("ROT=%.2fdps | ",
                record.rotationDps);

  Serial.printf("PIEZO=%.0f | ",
                record.vibrationIndex);

  Serial.printf("HAZARD=%.0f | ",
                record.hazardIndex);

  Serial.printf("STATE=%s\n",
                stateName(record.state));

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
  printPlotValue("AccelX_g", record.accelFiltered.x, 5, false);
  printPlotValue("AccelY_g", record.accelFiltered.y, 5, false);
  printPlotValue("AccelZ_g", record.accelFiltered.z, 5, false);
  printPlotValue("Roll_deg", record.rollDeg, 3, false);
  printPlotValue("Pitch_deg", record.pitchDeg, 3, false);
  printPlotValue("Tilt_deg", record.tiltDeg, 3, false);
  printPlotValue("GravityDiff_g", record.gravityDiffG, 5, false);
  printPlotValue("Shock_g", record.shockG, 5, false);
  printPlotValue("Rotation_dps", record.rotationDps, 3, false);
  printPlotValue("PiezoAmp_adc", record.vibrationAmplitudeAdc, 2, false);
  printPlotValue("PiezoRMS_adc", record.vibrationRmsAdc, 2, false);
  printPlotValue("PiezoEnergy_adc", record.vibrationEnergyAdc, 2, false);
  printPlotValue("PiezoIndex", record.vibrationIndex, 2, false);
  printPlotValue("GravityShortTrend_g", record.shortTrend, 5, false);
  printPlotValue("GravityLongTrend_g", record.longTrend, 5, false);
  printPlotValue("FusionConfidence", record.fusionConfidence, 2, false);
  printPlotValue("HazardIndex", record.hazardIndex, 2, false);
  printPlotValue("WatchTilt_deg", thresholds.watchTiltDeg, 3, false);
  printPlotValue("WarningTilt_deg", thresholds.warningTiltDeg, 3, false);
  printPlotValue("DangerTilt_deg", thresholds.dangerTiltDeg, 3, true);
#endif
}

// Structured logging is separated from printing so the same record can later be
// redirected to SD, CSV, WiFi, MQTT, or a database without changing the monitor.
void logRecord(const DataRecord &record) {
  printText(record);
  printPlot(record);
}

void readFutureSensors() {
  // Reserved extension point for rain, soil moisture, GPS, barometer,
  // temperature, LoRa, WiFi, MQTT, and SD-card metadata.
}

void writeStorageRecord(const DataRecord &record) {
  (void)record;
  // Reserved extension point for SD-card or flash logging.
}

void publishNetworkTelemetry(const DataRecord &record) {
  (void)record;
  // Reserved extension point for WiFi, MQTT, LoRa, or database upload.
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

  calibrateSensor();
  calibratePiezo();
  calculateAdaptiveThresholds();
  lastSampleMs = millis();
}

void loop() {
  uint32_t nowMs = millis();
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
  FilteredData filtered;
  OrientationData orientation;
  MotionData motion;
  PiezoSample piezoSample;
  PiezoFilteredData piezoFiltered;
  static TrendData trend = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  static VibrationData vibration = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f,
                                    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                    VIBRATION_VERY_QUIET};

  readSensor(sample);
  readPiezo(piezoSample);
  checkFifoStatus();

  float dtSeconds = (sample.timestampMs - lastSampleMs) / 1000.0f;
  if (dtSeconds <= 0.0f || dtSeconds > 5.0f) {
    dtSeconds = SAMPLE_INTERVAL_MS / 1000.0f;
  }
  lastSampleMs = sample.timestampMs;

  filterData(sample, filtered);
  calculateTilt(sample, filtered, dtSeconds, orientation);
  calculateGravity(filtered, motion);
  calculateShock(filtered, motion);
  calculateRotation(sample, motion);
  calculateTrend(orientation, motion, dtSeconds, trend);
  
  filterPiezo(piezoSample, piezoFiltered);
  calculateVibration(piezoSample, piezoFiltered, dtSeconds, vibration);

  // === TAMBAHKAN INI: Cek auto-calibration ===
  checkAutoCalibration(filtered, orientation, motion, sample.timestampMs);

  float fusionConfidence = calculateFusionConfidence(orientation, motion, trend, vibration);
  float hazardIndex = calculateHazard(orientation, motion, trend, vibration, fusionConfidence);
  updateState(hazardIndex, sample.timestampMs);
  
  uint16_t eventMask = detectEvents(sample, filtered, orientation, motion, trend,
                                    vibration, fusionConfidence);

  DataRecord record;
  makeDataRecord(sample, filtered, orientation, motion, trend, vibration,
                 fusionConfidence, hazardIndex, eventMask, record);
  logRecord(record);
  writeStorageRecord(record);
  publishNetworkTelemetry(record);
  readFutureSensors();

  // === TAMBAHKAN INI: Kontrol Buzzer ===
  controlBuzzer();

  previousOrientation = orientation;
  previousGravityDiffG = motion.gravityDiffG;
  previousLongTrend = trend.longGravityTrend;
  previousPiezoAmplitudeAdc = vibration.amplitudeAdc;
  previousPiezoLongTrend = vibration.longTrend;
  previousEventMask = eventMask;
}

