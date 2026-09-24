#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// Geometry types must be declared before Arduino generates function prototypes.
// Keeping them at the top avoids the Arduino preprocessor error
// "LocalPoint/GeoPoint does not name a type".
struct GeoPoint {
  double lat;
  double lon;
};

struct LocalPoint {
  float x;  // east, metres
  float y;  // north, metres
};

// ============================================================
// ОСНОВНАЯ ESP32
//
// MPU-6050:
//   SDA -> GPIO21
//   SCL -> GPIO22
//
// UART к ESP32-S3-CAM:
//   GPIO14 TX -> S3 GPIO1 RX
//   GPIO13 RX <- S3 GPIO2 TX
//   GND        -> GND
//
// GPS GY-NEO6MV2:
//   GPS TX -> ESP32 GPIO16 RX
//   GPS RX -> ESP32 GPIO17 TX (необязательно для чтения)
//   GND    -> GND
// ============================================================

// ============================================================
// ESC / differential thrust
// Tested physical orientation:
//   left ESC signal  -> GPIO25
//   right ESC signal -> GPIO2
//   equal forward PWM drives straight.
// ============================================================

constexpr int ESC_LEFT_PIN = 25;
constexpr int ESC_RIGHT_PIN = 2;
constexpr int ESC_NEUTRAL_US = 1500;

// Conservative automatic range. Lower pulse = more forward thrust
// on this specific catamaran/ESC setup.
// ESC calibration confirmed on this boat:
//   1500 us = neutral/stop
//   lower pulse = more forward thrust
//   1000 us = maximum forward command (NOT used in first water tests)
constexpr int ESC_MAX_FORWARD_US = 1000;

// Conservative first-water-test values.
// These deliberately stay far away from 1000 us.
constexpr int AUTO_FAST_US = 1300;
constexpr int AUTO_CRUISE_US = 1350;
constexpr int AUTO_SLOW_US = 1420;
constexpr int AUTO_FORWARD_LIMIT_US = 1180;
constexpr int AUTO_NEUTRAL_LIMIT_US = 1470;

constexpr uint32_t ESC_ARM_DELAY_MS = 12000;
constexpr uint32_t MOTOR_TEST_DURATION_MS = 1200;
constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 700;

Servo escLeft;
Servo escRight;

bool motorsArmed = false;
int currentLeftUs = ESC_NEUTRAL_US;
int currentRightUs = ESC_NEUTRAL_US;
uint32_t bootStartedMs = 0;
uint32_t motorTestStopAtMs = 0;
bool manualDriveActive = false;
uint32_t lastManualCommandMs = 0;

// Positive trim makes the right motor weaker in forward motion by moving
// its pulse closer to the 1500 us neutral point.
// Field observation: the right motor is stronger, so start with +20 us.
constexpr int RIGHT_FORWARD_TRIM_DEFAULT_US = 20;
constexpr int RIGHT_FORWARD_TRIM_MIN_US = -80;
constexpr int RIGHT_FORWARD_TRIM_MAX_US = 80;
int rightForwardTrimUs = RIGHT_FORWARD_TRIM_DEFAULT_US;

// Human-readable event visible on the web page.
String lastSystemEvent = "BOOT";

// Main-controller Wi-Fi is only a setup/safety interface.
// Autonomous navigation runs locally and does not require the phone to stay connected.
const char* CONTROL_AP_NAME = "CATAMARAN-CONTROL";
const char* CONTROL_AP_PASS = "12345678";
WebServer controlServer(80);
Preferences preferences;

enum class SurveyPointSlot : uint8_t { NONE, A, B, C, D };


// Explicit prototypes for functions that use custom geometry types.
// Arduino otherwise may auto-generate these prototypes above the type definitions.
LocalPoint geoToLocal(const GeoPoint& origin, const GeoPoint& point);
GeoPoint localToGeo(const GeoPoint& origin, const LocalPoint& point);
float localLength(const LocalPoint& point);
float distanceToGeo(const GeoPoint& point);
void setActiveTarget(const GeoPoint& point);
float cross2D(const LocalPoint& a, const LocalPoint& b, const LocalPoint& c);
float localDistance(const LocalPoint& a, const LocalPoint& b);
LocalPoint interpolateLocal(const LocalPoint& a, const LocalPoint& b, float t);
float polygonArea4(const LocalPoint& a, const LocalPoint& b, const LocalPoint& c, const LocalPoint& d);
bool orderedConvexQuadrilateral(
  const LocalPoint& a,
  const LocalPoint& b,
  const LocalPoint& c,
  const LocalPoint& d
);
bool chooseSurveyLockTarget(GeoPoint& target, bool& approachStart);
SurveyPointSlot surveyPointAveragingSlot = SurveyPointSlot::NONE;
uint32_t surveyPointAverageStartedMs = 0;
uint32_t surveyPointAverageLastSampleMs = 0;
double surveyPointAverageLatSum = 0.0;
double surveyPointAverageLonSum = 0.0;
uint32_t surveyPointAverageSamples = 0;
constexpr uint32_t SURVEY_POINT_AVERAGE_DURATION_MS = 6000;
constexpr uint32_t SURVEY_POINT_AVERAGE_SAMPLE_INTERVAL_MS = 200;

// ============================================================
// MPU-6050
// ============================================================

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

// ============================================================
// UART камеры
// Используем UART1, чтобы UART2 позже оставить GPS на GPIO16/17.
// ============================================================

constexpr uint32_t CAMERA_UART_BAUD = 115200;
constexpr int CAMERA_UART_RX_PIN = 13;
constexpr int CAMERA_UART_TX_PIN = 14;

HardwareSerial CameraSerial(1);
String cameraInputLine;

bool cameraOnline = false;
bool cameraRemoteReady = false;
bool cameraSdReady = false;
uint32_t lastCameraRxMs = 0;
uint32_t lastPingMs = 0;
uint32_t nextCaptureRequestId = 1;

bool missionActive = false;
uint32_t currentMissionNumber = 0;
String currentMissionPath = "";

bool captureRequestPending = false;
uint32_t pendingCaptureRequestId = 0;
int8_t lastCaptureResult = 0;  // 0 waiting/none, 1 OK, -1 failed
uint32_t captureRequestStartedMs = 0;
bool pendingCaptureIsSurvey = false;
uint32_t surveyPhotosRequested = 0;
uint32_t surveyPhotosSaved = 0;
uint32_t surveyPhotosFailed = 0;

constexpr uint32_t CAMERA_PING_INTERVAL_MS = 3000;
constexpr uint32_t CAMERA_OFFLINE_TIMEOUT_MS = 8000;


// ============================================================
// UART GPS
// UART2 остаётся отдельным от камеры и USB Serial.
// ============================================================

constexpr uint32_t GPS_UART_BAUD = 9600;
constexpr int GPS_UART_RX_PIN = 16;
constexpr int GPS_UART_TX_PIN = 17;

HardwareSerial GpsSerial(2);
TinyGPSPlus gps;

uint32_t lastGpsByteMs = 0;
uint32_t lastGpsFixMs = 0;

constexpr uint32_t GPS_NO_DATA_TIMEOUT_MS = 3000;
constexpr uint32_t GPS_FIX_STALE_TIMEOUT_MS = 5000;

constexpr uint8_t AUTO_MIN_SATELLITES = 5;
constexpr float AUTO_MAX_HDOP = 3.5f;
constexpr uint32_t AUTO_MAX_COURSE_AGE_MS = 2500;

// ============================================================
// Частота работы MPU
// ============================================================

constexpr uint32_t UPDATE_INTERVAL_US = 10000;  // 100 Гц
constexpr uint32_t PRINT_INTERVAL_MS = 250;     // 4 раза/с

// ============================================================
// Калибровка
// ============================================================

constexpr uint32_t CALIBRATION_SAMPLES = 800;
constexpr float CALIBRATION_MAX_GYRO = 5.0f;
constexpr float CALIBRATION_MIN_ACCEL = 0.85f;
constexpr float CALIBRATION_MAX_ACCEL = 1.15f;

// ============================================================
// Условия стабильности
// ============================================================

// These values are diagnostics for water conditions, not a hard photo lock.
constexpr float MAX_STABLE_GYRO_X = 6.0f;
constexpr float MAX_STABLE_GYRO_Y = 6.0f;
constexpr float MAX_STABLE_GYRO_Z = 9.0f;

constexpr float MIN_STABLE_ACCEL = 0.75f;
constexpr float MAX_STABLE_ACCEL = 1.25f;

constexpr uint32_t REQUIRED_STABLE_TIME_MS = 180;

constexpr float MAX_CAPTURE_ROLL = 18.0f;
constexpr float MAX_CAPTURE_PITCH = 18.0f;

constexpr float FILTER_ALPHA = 0.98f;

// ============================================================
// Состояние MPU
// ============================================================

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float rollZeroDegrees = 0.0f;
float pitchZeroDegrees = 0.0f;

float filteredRollDegrees = 0.0f;
float filteredPitchDegrees = 0.0f;

float relativeRollDegrees = 0.0f;
float relativePitchDegrees = 0.0f;

float currentGyroX = 0.0f;
float currentGyroY = 0.0f;
float currentGyroZ = 0.0f;
float currentTotalAcceleration = 0.0f;
float currentTemperature = 0.0f;

bool stableInstant = false;
bool stableConfirmed = false;
bool levelEnough = false;
bool captureReady = false;

uint32_t stableStartedAt = 0;
uint32_t previousUpdateUs = 0;
uint32_t previousPrintMs = 0;
uint32_t lastMpuGoodMs = 0;

// ============================================================
// Heading estimator without a compass
// ============================================================
// Absolute heading is initialized by physically pointing the bow at the
// current target and pressing LOCK BOW TO TARGET. After that, MPU gyro Z
// tracks relative turns. GPS course is used only as a very slow correction
// while the boat is already moving straight.
bool headingInitialized = false;
float headingEstimateDeg = 0.0f;
uint32_t headingLockedAtMs = 0;
uint32_t lastGpsHeadingCorrectionMs = 0;
float gpsHeadingCorrectionTotalDeg = 0.0f;

// Compass heading grows clockwise. With the confirmed mounting (Z+ up),
// MPU-6050 gyro Z normally has the opposite sign, so -1 is the safe default.
// A continuous manual LEFT/RIGHT hold automatically verifies the sign.
int8_t gyroCompassSign = -1;
bool gyroSignVerified = false;
bool gyroTurnLearning = false;
int8_t gyroTurnExpectedDirection = 0;  // +1 right, -1 left
uint32_t gyroTurnLearningStartedMs = 0;
double gyroTurnSampleSum = 0.0;
uint32_t gyroTurnSampleCount = 0;

bool bowLockPending = false;
uint32_t bowLockStartedMs = 0;
uint32_t bowLockLastSampleMs = 0;
double bowLockGyroSum = 0.0;
double bowLockAbsGyroSum = 0.0;
double bowLockBearingSinSum = 0.0;
double bowLockBearingCosSum = 0.0;
uint32_t bowLockSampleCount = 0;

// ============================================================
// Регистры MPU-6050
// ============================================================

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(
    MPU_ADDRESS,
    static_cast<uint8_t>(1),
    static_cast<uint8_t>(true)
  );

  if (received != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool readRegisters(
  uint8_t startRegister,
  uint8_t* buffer,
  size_t length
) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(startRegister);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(
    MPU_ADDRESS,
    static_cast<uint8_t>(length),
    static_cast<uint8_t>(true)
  );

  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte) {
  return static_cast<int16_t>(
    (static_cast<uint16_t>(highByte) << 8) |
    static_cast<uint16_t>(lowByte)
  );
}

bool readMPU(
  float& ax,
  float& ay,
  float& az,
  float& gx,
  float& gy,
  float& gz,
  float& temperature
) {
  uint8_t data[14];

  if (!readRegisters(0x3B, data, sizeof(data))) {
    return false;
  }

  int16_t rawAx = combineBytes(data[0], data[1]);
  int16_t rawAy = combineBytes(data[2], data[3]);
  int16_t rawAz = combineBytes(data[4], data[5]);

  int16_t rawTemperature =
    combineBytes(data[6], data[7]);

  int16_t rawGx = combineBytes(data[8], data[9]);
  int16_t rawGy = combineBytes(data[10], data[11]);
  int16_t rawGz = combineBytes(data[12], data[13]);

  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;

  gx = rawGx / 131.0f;
  gy = rawGy / 131.0f;
  gz = rawGz / 131.0f;

  temperature =
    rawTemperature / 340.0f + 36.53f;

  return true;
}

bool initializeMPU() {
  Wire.beginTransmission(MPU_ADDRESS);

  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU-6050 not found");
    return false;
  }

  if (!writeRegister(0x6B, 0x80)) {
    Serial.println("ERROR: MPU reset failed");
    return false;
  }

  delay(150);

  if (!writeRegister(0x6B, 0x01)) {
    Serial.println("ERROR: MPU wake failed");
    return false;
  }

  if (!writeRegister(0x1A, 0x03)) {
    return false;
  }

  if (!writeRegister(0x1B, 0x00)) {
    return false;
  }

  if (!writeRegister(0x1C, 0x00)) {
    return false;
  }

  if (!writeRegister(0x19, 0x09)) {
    return false;
  }

  delay(200);

  uint8_t whoAmI = 0;

  if (!readRegister(0x75, whoAmI)) {
    Serial.println("ERROR: WHO_AM_I read failed");
    return false;
  }

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  return whoAmI == 0x68;
}

bool calibrateRobotPosition() {
  double gyroSumX = 0.0;
  double gyroSumY = 0.0;
  double gyroSumZ = 0.0;

  double rollSum = 0.0;
  double pitchSum = 0.0;

  uint32_t acceptedSamples = 0;
  uint32_t calibrationStartedAt = millis();

  Serial.println();
  Serial.println("======================================");
  Serial.println("MPU CALIBRATION");
  Serial.println("DO NOT MOVE THE ROBOT");
  Serial.println("Current position becomes level zero");
  Serial.println("======================================");

  delay(1000);

  while (acceptedSamples < CALIBRATION_SAMPLES) {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float temperature;

    if (!readMPU(
          ax,
          ay,
          az,
          gx,
          gy,
          gz,
          temperature
        )) {
      Serial.println("ERROR: MPU read failed during calibration");
      return false;
    }

    float totalAcceleration = sqrtf(
      ax * ax +
      ay * ay +
      az * az
    );

    bool accelerationValid =
      totalAcceleration >= CALIBRATION_MIN_ACCEL &&
      totalAcceleration <= CALIBRATION_MAX_ACCEL;

    bool gyroValid =
      fabsf(gx) <= CALIBRATION_MAX_GYRO &&
      fabsf(gy) <= CALIBRATION_MAX_GYRO &&
      fabsf(gz) <= CALIBRATION_MAX_GYRO;

    if (!accelerationValid || !gyroValid) {
      Serial.println("Movement detected. Calibration restarted.");

      gyroSumX = 0.0;
      gyroSumY = 0.0;
      gyroSumZ = 0.0;
      rollSum = 0.0;
      pitchSum = 0.0;
      acceptedSamples = 0;

      delay(500);
      continue;
    }

    float measuredRoll =
      atan2f(ay, az) * 180.0f / PI;

    float measuredPitch =
      atan2f(
        -ax,
        sqrtf(ay * ay + az * az)
      ) * 180.0f / PI;

    gyroSumX += gx;
    gyroSumY += gy;
    gyroSumZ += gz;

    rollSum += measuredRoll;
    pitchSum += measuredPitch;

    acceptedSamples++;

    if (acceptedSamples % 100 == 0) {
      Serial.print("Calibration: ");
      Serial.print(acceptedSamples);
      Serial.print(" / ");
      Serial.println(CALIBRATION_SAMPLES);
    }

    delay(5);

    if (millis() - calibrationStartedAt > 60000) {
      Serial.println("ERROR: calibration timeout");
      return false;
    }
  }

  gyroOffsetX =
    static_cast<float>(gyroSumX / CALIBRATION_SAMPLES);

  gyroOffsetY =
    static_cast<float>(gyroSumY / CALIBRATION_SAMPLES);

  gyroOffsetZ =
    static_cast<float>(gyroSumZ / CALIBRATION_SAMPLES);

  rollZeroDegrees =
    static_cast<float>(rollSum / CALIBRATION_SAMPLES);

  pitchZeroDegrees =
    static_cast<float>(pitchSum / CALIBRATION_SAMPLES);

  filteredRollDegrees = rollZeroDegrees;
  filteredPitchDegrees = pitchZeroDegrees;

  relativeRollDegrees = 0.0f;
  relativePitchDegrees = 0.0f;

  stableStartedAt = 0;
  stableInstant = false;
  stableConfirmed = false;
  levelEnough = true;
  captureReady = false;

  // A new gyro calibration invalidates the old absolute heading reference.
  headingInitialized = false;
  bowLockPending = false;
  headingEstimateDeg = 0.0f;
  gpsHeadingCorrectionTotalDeg = 0.0f;

  previousUpdateUs = micros();
  previousPrintMs = millis();
  lastMpuGoodMs = millis();

  Serial.println();
  Serial.println("CALIBRATION COMPLETED");

  Serial.print("Gyro offsets: ");
  Serial.print(gyroOffsetX, 4);
  Serial.print(", ");
  Serial.print(gyroOffsetY, 4);
  Serial.print(", ");
  Serial.println(gyroOffsetZ, 4);

  Serial.print("Level zero: roll=");
  Serial.print(rollZeroDegrees, 2);
  Serial.print(" pitch=");
  Serial.println(pitchZeroDegrees, 2);

  Serial.println();
  return true;
}

void updateMPU() {
  uint32_t nowUs = micros();

  uint32_t elapsedUs =
    static_cast<uint32_t>(
      nowUs - previousUpdateUs
    );

  if (elapsedUs < UPDATE_INTERVAL_US) {
    return;
  }

  float deltaTime =
    elapsedUs / 1000000.0f;

  previousUpdateUs = nowUs;

  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float temperature;

  if (!readMPU(
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        temperature
      )) {
    Serial.println("ERROR: MPU read failed");
    return;
  }

  gx -= gyroOffsetX;
  gy -= gyroOffsetY;
  gz -= gyroOffsetZ;

  currentGyroX = gx;
  currentGyroY = gy;
  currentGyroZ = gz;
  currentTemperature = temperature;
  lastMpuGoodMs = millis();

  // Integrate only relative yaw. The absolute reference is established by
  // LOCK BOW TO TARGET, never by the noisy GPS course at startup.
  if (headingInitialized && !bowLockPending && deltaTime > 0.0f && deltaTime <= 0.10f) {
    float compassYawRateDegPerSec =
      static_cast<float>(gyroCompassSign) * currentGyroZ;

    headingEstimateDeg += compassYawRateDegPerSec * deltaTime;
    while (headingEstimateDeg >= 360.0f) headingEstimateDeg -= 360.0f;
    while (headingEstimateDeg < 0.0f) headingEstimateDeg += 360.0f;
  }

  // Learn the gyro sign from a deliberate manual LEFT or RIGHT turn.
  // This removes any doubt about board orientation or a flipped MPU module.
  if (gyroTurnLearning) {
    gyroTurnSampleSum += currentGyroZ;
    gyroTurnSampleCount++;

    if (millis() - gyroTurnLearningStartedMs >= 800) {
      float averageGyroZ = gyroTurnSampleCount > 0
        ? static_cast<float>(gyroTurnSampleSum / gyroTurnSampleCount)
        : 0.0f;

      if (fabsf(averageGyroZ) >= 2.0f) {
        int8_t rawTurnSign = averageGyroZ >= 0.0f ? 1 : -1;
        gyroCompassSign = gyroTurnExpectedDirection * rawTurnSign;
        gyroSignVerified = true;
        preferences.putInt("gyroSign", gyroCompassSign);
        preferences.putBool("gyroSignOK", true);
        lastSystemEvent = "GYRO_SIGN_VERIFIED_" + String(static_cast<int>(gyroCompassSign));

        Serial.print("Gyro compass sign verified: ");
        Serial.print(static_cast<int>(gyroCompassSign));
        Serial.print(" | average gyro Z: ");
        Serial.println(averageGyroZ, 2);
      } else {
        lastSystemEvent = "GYRO_SIGN_TEST_TOO_WEAK";
        Serial.println("Gyro sign test failed: hold LEFT or RIGHT longer");
      }

      gyroTurnLearning = false;
      gyroTurnExpectedDirection = 0;
      gyroTurnSampleSum = 0.0;
      gyroTurnSampleCount = 0;
    }
  }

  currentTotalAcceleration = sqrtf(
    ax * ax +
    ay * ay +
    az * az
  );

  float accelerometerRoll =
    atan2f(ay, az) * 180.0f / PI;

  float accelerometerPitch =
    atan2f(
      -ax,
      sqrtf(ay * ay + az * az)
    ) * 180.0f / PI;

  filteredRollDegrees =
    FILTER_ALPHA *
      (
        filteredRollDegrees +
        gx * deltaTime
      ) +
    (1.0f - FILTER_ALPHA) *
      accelerometerRoll;

  filteredPitchDegrees =
    FILTER_ALPHA *
      (
        filteredPitchDegrees +
        gy * deltaTime
      ) +
    (1.0f - FILTER_ALPHA) *
      accelerometerPitch;

  relativeRollDegrees =
    filteredRollDegrees -
    rollZeroDegrees;

  relativePitchDegrees =
    filteredPitchDegrees -
    pitchZeroDegrees;

  stableInstant =
    fabsf(gx) < MAX_STABLE_GYRO_X &&
    fabsf(gy) < MAX_STABLE_GYRO_Y &&
    fabsf(gz) < MAX_STABLE_GYRO_Z &&
    currentTotalAcceleration > MIN_STABLE_ACCEL &&
    currentTotalAcceleration < MAX_STABLE_ACCEL;

  uint32_t nowMs = millis();

  if (stableInstant) {
    if (stableStartedAt == 0) {
      stableStartedAt = nowMs;
    }

    stableConfirmed =
      nowMs - stableStartedAt >=
      REQUIRED_STABLE_TIME_MS;
  } else {
    stableStartedAt = 0;
    stableConfirmed = false;
  }

  levelEnough =
    fabsf(relativeRollDegrees) < MAX_CAPTURE_ROLL &&
    fabsf(relativePitchDegrees) < MAX_CAPTURE_PITCH;

  captureReady =
    stableConfirmed &&
    levelEnough;
}


// Re-zero only roll/pitch after the boat is floating.
// Gyro offsets remain from the proper stationary boot calibration.
void setCurrentWaterLevelZero() {
  rollZeroDegrees = filteredRollDegrees;
  pitchZeroDegrees = filteredPitchDegrees;
  relativeRollDegrees = 0.0f;
  relativePitchDegrees = 0.0f;
  stableStartedAt = 0;
  stableConfirmed = false;
  levelEnough = true;
  captureReady = false;
  lastSystemEvent = "WATER_LEVEL_ZERO_SET";

  Serial.print("Water level zero set: roll=");
  Serial.print(rollZeroDegrees, 2);
  Serial.print(" pitch=");
  Serial.println(pitchZeroDegrees, 2);
}


// ============================================================
// GPS
// ============================================================

void updateGPS() {
  while (GpsSerial.available() > 0) {
    char incoming = static_cast<char>(GpsSerial.read());
    lastGpsByteMs = millis();
    gps.encode(incoming);
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    lastGpsFixMs = millis();
  }
}

bool gpsHasData() {
  return
    gps.charsProcessed() > 10 &&
    millis() - lastGpsByteMs <= GPS_NO_DATA_TIMEOUT_MS;
}

bool gpsHasFreshFix() {
  return
    gps.location.isValid() &&
    millis() - lastGpsFixMs <= GPS_FIX_STALE_TIMEOUT_MS;
}

void printGpsStatus() {
  Serial.println();
  Serial.println("========== GPS STATUS ==========");

  Serial.print("UART RX: GPIO");
  Serial.println(GPS_UART_RX_PIN);

  Serial.print("UART TX: GPIO");
  Serial.println(GPS_UART_TX_PIN);

  Serial.print("Baud: ");
  Serial.println(GPS_UART_BAUD);

  Serial.print("Chars processed: ");
  Serial.println(gps.charsProcessed());

  Serial.print("GPS data: ");
  Serial.println(gpsHasData() ? "YES" : "NO");

  Serial.print("Fix: ");
  Serial.println(gpsHasFreshFix() ? "YES" : "NO");

  Serial.print("Satellites: ");
  if (gps.satellites.isValid()) {
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.print("HDOP: ");
  if (gps.hdop.isValid()) {
    Serial.println(gps.hdop.hdop(), 2);
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.print("Latitude: ");
  if (gps.location.isValid()) {
    Serial.println(gps.location.lat(), 7);
  } else {
    Serial.println("NO FIX");
  }

  Serial.print("Longitude: ");
  if (gps.location.isValid()) {
    Serial.println(gps.location.lng(), 7);
  } else {
    Serial.println("NO FIX");
  }

  Serial.print("Altitude: ");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 1);
    Serial.println(" m");
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.print("Speed: ");
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph(), 2);
    Serial.println(" km/h");
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.print("Course: ");
  if (gps.course.isValid()) {
    Serial.print(gps.course.deg(), 1);
    Serial.println(" deg");
  } else {
    Serial.println("UNKNOWN");
  }

  Serial.println("================================");
  Serial.println();
}

String gpsUtcString() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return "NO_TIME";
  }

  char buffer[32];
  snprintf(
    buffer,
    sizeof(buffer),
    "%04d-%02d-%02dT%02d:%02d:%02dZ",
    gps.date.year(),
    gps.date.month(),
    gps.date.day(),
    gps.time.hour(),
    gps.time.minute(),
    gps.time.second()
  );

  return String(buffer);
}

String gpsValueOrNA(bool valid, double value, uint8_t decimals) {
  if (!valid) {
    return "NA";
  }
  return String(value, static_cast<unsigned int>(decimals));
}

// ============================================================
// Camera UART + mission control
// ============================================================

void sendCameraCommand(const String& command) {
  CameraSerial.println(command);

  Serial.print("CAMERA TX: ");
  Serial.println(command);
}

void sendPing() {
  sendCameraCommand("PING");
  lastPingMs = millis();
}

void requestCameraStatus() {
  sendCameraCommand("STATUS");
}

void requestMissionStatus() {
  sendCameraCommand("MISSION_STATUS");
}

void requestMissionStart() {
  if (!cameraOnline) {
    Serial.println("MISSION START BLOCKED: camera is offline");
    return;
  }

  if (missionActive) {
    Serial.println("MISSION START BLOCKED: mission already active");
    return;
  }

  lastSystemEvent = "MISSION_START_SENT";
  sendCameraCommand("MISSION_START");
}

void requestMissionEnd() {
  if (!cameraOnline) {
    Serial.println("MISSION END BLOCKED: camera is offline");
    return;
  }

  if (!missionActive) {
    Serial.println("MISSION END BLOCKED: no active mission");
    return;
  }

  sendCameraCommand("MISSION_END");
}

bool requestCapture(bool forceCapture) {
  if (captureRequestPending) {
    Serial.println("CAPTURE BLOCKED: previous request is still pending");
    return false;
  }

  if (!cameraOnline) {
    Serial.println("CAPTURE BLOCKED: camera is offline");
    return false;
  }

  if (!missionActive) {
    Serial.println("CAPTURE BLOCKED: start a mission with M first");
    return false;
  }

  if (!forceCapture && !captureReady) {
    Serial.println("CAPTURE BLOCKED: robot is not stable/level");
    return false;
  }

  if (!forceCapture && !gpsHasFreshFix()) {
    Serial.println("CAPTURE BLOCKED: GPS fix is not fresh");
    return false;
  }

  uint32_t requestId = nextCaptureRequestId++;

  String command;
  command.reserve(460);

  command = "CAPTURE,";
  command += String(requestId);
  command += ",";
  command += gpsUtcString();
  command += ",";
  command += gpsValueOrNA(
    gps.location.isValid(),
    gps.location.isValid() ? gps.location.lat() : 0.0,
    7
  );
  command += ",";
  command += gpsValueOrNA(
    gps.location.isValid(),
    gps.location.isValid() ? gps.location.lng() : 0.0,
    7
  );
  command += ",";
  command += gpsValueOrNA(
    gps.altitude.isValid(),
    gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
    2
  );
  command += ",";
  command += gps.satellites.isValid()
    ? String(gps.satellites.value())
    : "NA";
  command += ",";
  command += gpsValueOrNA(
    gps.hdop.isValid(),
    gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
    2
  );
  command += ",";
  command += gpsValueOrNA(
    gps.speed.isValid(),
    gps.speed.isValid() ? gps.speed.kmph() : 0.0,
    3
  );
  command += ",";
  command += gpsValueOrNA(
    gps.course.isValid(),
    gps.course.isValid() ? gps.course.deg() : 0.0,
    2
  );
  command += ",";
  command += String(relativeRollDegrees, 3);
  command += ",";
  command += String(relativePitchDegrees, 3);
  command += ",";
  command += String(currentGyroX, 3);
  command += ",";
  command += String(currentGyroY, 3);
  command += ",";
  command += String(currentGyroZ, 3);
  command += ",";
  command += String(currentTotalAcceleration, 4);
  command += ",";
  command += stableConfirmed ? "1" : "0";
  command += ",";
  command += levelEnough ? "1" : "0";

  sendCameraCommand(command);

  captureRequestPending = true;
  pendingCaptureRequestId = requestId;
  lastCaptureResult = 0;
  captureRequestStartedMs = millis();

  Serial.print("Capture request ID: ");
  Serial.println(requestId);

  if (forceCapture) {
    Serial.println("Forced mission capture requested");
  }

  return true;
}

String csvField(const String& text, int fieldIndex) {
  int start = 0;
  int current = 0;

  while (true) {
    int comma = text.indexOf(',', start);

    if (current == fieldIndex) {
      if (comma < 0) {
        return text.substring(start);
      }
      return text.substring(start, comma);
    }

    if (comma < 0) {
      return "";
    }

    start = comma + 1;
    current++;
  }
}

void applyMissionStatusResponse(const String& response) {
  int activePos = response.indexOf("ACTIVE=");
  int numberPos = response.indexOf("NUMBER=");
  int pathPos = response.indexOf("PATH=");

  if (activePos >= 0) {
    int end = response.indexOf(',', activePos);
    String value = response.substring(
      activePos + 7,
      end < 0 ? response.length() : end
    );
    missionActive = value == "1";
  }

  if (numberPos >= 0) {
    int end = response.indexOf(',', numberPos);
    String value = response.substring(
      numberPos + 7,
      end < 0 ? response.length() : end
    );
    currentMissionNumber = value.toInt();
  }

  if (pathPos >= 0) {
    int end = response.indexOf(',', pathPos);
    currentMissionPath = response.substring(
      pathPos + 5,
      end < 0 ? response.length() : end
    );

    if (currentMissionPath == "-") {
      currentMissionPath = "";
    }
  }
}

void processCameraResponse(String response) {
  response.trim();

  if (response.length() == 0) {
    return;
  }

  lastCameraRxMs = millis();
  cameraOnline = true;

  Serial.print("CAMERA RX: ");
  Serial.println(response);

  if (response == "PONG") {
    Serial.println("CAMERA LINK: ONLINE");
    return;
  }

  if (response == "CAMERA_BOOT") {
    lastSystemEvent = "CAMERA_BOOT";
    requestCameraStatus();
    requestMissionStatus();
    return;
  }

  if (response.startsWith("MISSION_OK,")) {
    currentMissionNumber = csvField(response, 1).toInt();
    currentMissionPath = csvField(response, 2);
    missionActive = true;

    lastSystemEvent = "MISSION_STARTED_M" + String(currentMissionNumber);
    Serial.println("MISSION STARTED");
    Serial.print("Mission number: ");
    Serial.println(currentMissionNumber);
    Serial.print("Mission path: ");
    Serial.println(currentMissionPath);
    return;
  }

  if (response.startsWith("MISSION_RESUMED,")) {
    currentMissionNumber = csvField(response, 1).toInt();
    currentMissionPath = csvField(response, 2);
    missionActive = true;

    Serial.println("MISSION RESUMED AFTER REBOOT");
    return;
  }

  if (response.startsWith("MISSION_STATUS,")) {
    applyMissionStatusResponse(response);
    Serial.print("MISSION: ");
    Serial.println(missionActive ? "ACTIVE" : "INACTIVE");
    return;
  }

  if (response.startsWith("MISSION_ENDED,")) {
    Serial.println("MISSION ENDED");
    missionActive = false;
    currentMissionNumber = 0;
    currentMissionPath = "";
    return;
  }

  if (response.startsWith("MISSION_ERROR,")) {
    lastSystemEvent = response;
    Serial.print("MISSION COMMAND FAILED: ");
    Serial.println(csvField(response, 1));
    return;
  }

  if (response.startsWith("STATUS,")) {
    int cameraPos = response.indexOf("CAMERA=");
    int sdPos = response.indexOf("SD=");
    int missionPos = response.indexOf("MISSION_ACTIVE=");

    if (cameraPos >= 0) {
      int end = response.indexOf(',', cameraPos);
      String value = response.substring(
        cameraPos + 7,
        end < 0 ? response.length() : end
      );
      cameraRemoteReady = value == "1";
    }

    if (sdPos >= 0) {
      int end = response.indexOf(',', sdPos);
      String value = response.substring(
        sdPos + 3,
        end < 0 ? response.length() : end
      );
      cameraSdReady = value == "1";
    }
    if (missionPos >= 0) {
      int end = response.indexOf(',', missionPos);
      String active = response.substring(
        missionPos + 15,
        end < 0 ? response.length() : end
      );
      missionActive = active == "1";
    }
    return;
  }

  if (response.startsWith("CAPTURE_OK,")) {
    uint32_t responseId = csvField(response, 1).toInt();
    bool matchedPending = captureRequestPending && responseId == pendingCaptureRequestId;
    bool wasSurveyCapture = matchedPending && pendingCaptureIsSurvey;
    if (matchedPending) {
      captureRequestPending = false;
      lastCaptureResult = 1;
      pendingCaptureIsSurvey = false;
      if (wasSurveyCapture) surveyPhotosSaved++;
    }

    lastSystemEvent = "CAPTURE_OK_" + csvField(response, 2);
    Serial.println("PHOTO + TELEMETRY + GEO SAVED SUCCESSFULLY");
    Serial.print("Saved path: ");
    Serial.println(csvField(response, 2));
    return;
  }

  if (response.startsWith("CAPTURE_PARTIAL,")) {
    uint32_t responseId = csvField(response, 1).toInt();
    bool matchedPending = captureRequestPending && responseId == pendingCaptureRequestId;
    bool wasSurveyCapture = matchedPending && pendingCaptureIsSurvey;
    if (matchedPending) {
      captureRequestPending = false;
      lastCaptureResult = -1;
      pendingCaptureIsSurvey = false;
      if (wasSurveyCapture) surveyPhotosFailed++;
    }

    lastSystemEvent = "CAPTURE_PARTIAL";
    Serial.println("PHOTO SAVED, BUT TELEMETRY/GEO WRITE FAILED");
    return;
  }

  if (response.startsWith("CAPTURE_ERROR,")) {
    uint32_t responseId = csvField(response, 1).toInt();
    bool matchedPending = captureRequestPending && responseId == pendingCaptureRequestId;
    bool wasSurveyCapture = matchedPending && pendingCaptureIsSurvey;
    if (matchedPending) {
      captureRequestPending = false;
      lastCaptureResult = -1;
      pendingCaptureIsSurvey = false;
      if (wasSurveyCapture) surveyPhotosFailed++;
    }

    lastSystemEvent = response;
    Serial.println("PHOTO SAVE FAILED");
    return;
  }
}

void updateCameraUart() {
  while (CameraSerial.available() > 0) {
    char incoming =
      static_cast<char>(CameraSerial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      processCameraResponse(cameraInputLine);
      cameraInputLine = "";
      continue;
    }

    if (cameraInputLine.length() < 700) {
      cameraInputLine += incoming;
    } else {
      cameraInputLine = "";
      Serial.println("CAMERA RX ERROR: line too long");
    }
  }

  uint32_t nowMs = millis();

  if (
    cameraOnline &&
    nowMs - lastCameraRxMs >
      CAMERA_OFFLINE_TIMEOUT_MS
  ) {
    cameraOnline = false;
    Serial.println("CAMERA LINK: OFFLINE");
  }

  if (
    nowMs - lastPingMs >=
      CAMERA_PING_INTERVAL_MS
  ) {
    sendPing();
  }
}

// ============================================================
// V5.1.1 match survey controller (Arduino compile fix)
// Four ordered boundary points define the search quadrilateral:
//   A = start of first strip
//   B = end of first strip
//   C = opposite corner next to B
//   D = opposite corner next to A
// Points must be entered around the boundary as A -> B -> C -> D.
// Survey strips are interpolated between side A-D and side B-C. This keeps the
// planner simple and reliable while supporting rectangles and mild trapezoids.
// ============================================================

enum class SurveyState : uint8_t {
  IDLE,
  TURN_TO_START,
  APPROACH_START,
  TURN_TO_LEG,
  TRACK_LEG,
  WAYPOINT_HOLD,
  FINISH_WAIT,
  COMPLETE,
  FAILSAFE
};

SurveyState surveyState = SurveyState::IDLE;
String autoMessage = "IDLE";

GeoPoint surveyA;
GeoPoint surveyB;
GeoPoint surveyC;
GeoPoint surveyD;
bool surveyASet = false;
bool surveyBSet = false;
bool surveyCSet = false;
bool surveyDSet = false;

constexpr uint8_t MAX_SURVEY_LANES = 32;
constexpr uint8_t MAX_ROUTE_POINTS = MAX_SURVEY_LANES * 2;
GeoPoint surveyRoute[MAX_ROUTE_POINTS];
uint8_t surveyRoutePointCount = 0;
uint8_t surveyLaneCount = 0;
float surveyRequestedSpacingM = 1.5f;
float surveyActualSpacingM = 0.0f;
float surveyLengthM = 0.0f;
float surveyWidthM = 0.0f;
float surveyAreaM2 = 0.0f;
float surveyTotalRouteM = 0.0f;
bool surveyRouteBuilt = false;

int16_t activeLegIndex = -1;  // route point i -> i+1; -1 means approach A
bool activeLegIsSurvey = false;
float activeLegLengthM = 0.0f;
float activeLegAlongM = 0.0f;
float activeLegCrossTrackM = 0.0f;
float activeLegProgressPercent = 0.0f;
uint32_t activeLegStartedMs = 0;
uint32_t turnAlignedStartedMs = 0;
uint32_t waypointHoldStartedMs = 0;
bool finalPhotoRequestedForLeg = false;

bool targetSet = false;
double targetLatitude = 0.0;
double targetLongitude = 0.0;
double currentTargetDistanceM = -1.0;
double currentTargetBearingDeg = -1.0;
double currentHeadingErrorDeg = 0.0;
float desiredTrackBearingDeg = 0.0f;

float steeringCorrectionUs = 0.0f;
uint32_t lastSteeringUpdateMs = 0;
uint32_t previousAutoUpdateMs = 0;
uint32_t autoRunStartedMs = 0;
double autoStartLatitude = 0.0;
double autoStartLongitude = 0.0;

uint32_t surveyPhotoIntervalMs = 1100;
uint32_t lastSurveyPhotoRequestMs = 0;
uint32_t surveyLegStartedMs = 0;

constexpr float EARTH_RADIUS_M = 6371000.0f;
constexpr uint32_t AUTO_UPDATE_INTERVAL_MS = 100;
constexpr uint32_t MPU_STALE_TIMEOUT_MS = 500;
constexpr uint32_t AUTO_MAX_RUN_TIME_MS = 30UL * 60UL * 1000UL;
constexpr float AUTO_MAX_DISTANCE_FROM_A_M = 120.0f;
constexpr float AUTO_MAX_ROLL_DEG = 30.0f;
constexpr float AUTO_MAX_PITCH_DEG = 30.0f;
constexpr uint32_t AUTO_CAPTURE_TIMEOUT_MS = 12000;

constexpr float SURVEY_MIN_LENGTH_M = 5.0f;
constexpr float SURVEY_MAX_LENGTH_M = 90.0f;
constexpr float SURVEY_MIN_WIDTH_M = 2.5f;
constexpr float SURVEY_MAX_WIDTH_M = 60.0f;
constexpr float SURVEY_MIN_SPACING_M = 1.0f;
constexpr float SURVEY_MAX_SPACING_M = 8.0f;
constexpr uint32_t SURVEY_MIN_PHOTO_INTERVAL_MS = 900;
constexpr uint32_t SURVEY_MAX_PHOTO_INTERVAL_MS = 3000;
constexpr float SURVEY_START_NEAR_M = 5.0f;
constexpr float SURVEY_START_REACHED_M = 2.5f;
constexpr float SURVEY_ENDPOINT_RADIUS_M = 2.4f;
constexpr float SURVEY_CONNECTOR_RADIUS_M = 1.8f;
constexpr float SURVEY_LOOKAHEAD_M = 3.0f;
constexpr float CONNECTOR_LOOKAHEAD_M = 1.6f;
constexpr float SURVEY_HEADING_DEADBAND_DEG = 5.0f;
constexpr float SURVEY_HEADING_KP_US_PER_DEG = 0.50f;
constexpr float SURVEY_YAW_DAMPING_US_PER_DPS = 0.80f;
constexpr float SURVEY_MAX_STEERING_US = 30.0f;
constexpr float SURVEY_STEERING_SLEW_US = 4.0f;
constexpr int SURVEY_FORWARD_LIMIT_US = 1210;
constexpr int SURVEY_WEAKEST_FORWARD_US = 1460;
constexpr int SURVEY_BASE_US = 1360;
constexpr int CONNECTOR_BASE_US = 1410;
constexpr uint32_t TURN_ALIGNED_HOLD_MS = 550;
constexpr uint32_t TURN_TIMEOUT_MS = 25000;
constexpr float TURN_ACCEPT_DEG = 10.0f;
constexpr int TURN_FAST_FINAL_US = 1370;
constexpr int TURN_SLOW_FINAL_US = 1420;
constexpr uint32_t WAYPOINT_HOLD_MS = 900;
constexpr uint32_t BOW_LOCK_MAX_START_AGE_MS = 5UL * 60UL * 1000UL;

// Slow GPS correction for gyro drift; never used for direct steering.
constexpr float HEADING_GPS_MIN_SPEED_KMPH = 1.00f;
constexpr float HEADING_GPS_MAX_YAW_RATE_DPS = 5.0f;
constexpr float HEADING_GPS_MAX_COURSE_DELTA_DEG = 25.0f;
constexpr float HEADING_GPS_CORRECTION_GAIN = 0.030f;
constexpr float HEADING_GPS_MAX_STEP_DEG = 0.6f;
constexpr uint32_t HEADING_GPS_CORRECTION_INTERVAL_MS = 900;

constexpr uint32_t BOW_LOCK_DURATION_MS = 1800;
constexpr uint32_t BOW_LOCK_SAMPLE_INTERVAL_MS = 20;
constexpr float BOW_LOCK_MAX_AVERAGE_GYRO_DPS = 2.5f;
constexpr float BOW_LOCK_MAX_AVERAGE_ABS_GYRO_DPS = 5.0f;

String surveyStateName() {
  switch (surveyState) {
    case SurveyState::IDLE: return "IDLE";
    case SurveyState::TURN_TO_START: return "TURN_TO_START";
    case SurveyState::APPROACH_START: return "APPROACH_START";
    case SurveyState::TURN_TO_LEG: return "TURN_TO_LEG";
    case SurveyState::TRACK_LEG: return "TRACK_LEG";
    case SurveyState::WAYPOINT_HOLD: return "WAYPOINT_HOLD";
    case SurveyState::FINISH_WAIT: return "FINISH_WAIT";
    case SurveyState::COMPLETE: return "COMPLETE";
    case SurveyState::FAILSAFE: return "FAILSAFE";
  }
  return "UNKNOWN";
}

bool surveyRunning() {
  return surveyState != SurveyState::IDLE &&
         surveyState != SurveyState::COMPLETE &&
         surveyState != SurveyState::FAILSAFE;
}

float normalizeHeadingError(float errorDegrees) {
  while (errorDegrees > 180.0f) errorDegrees -= 360.0f;
  while (errorDegrees < -180.0f) errorDegrees += 360.0f;
  return errorDegrees;
}

float normalizeHeading360(float headingDegrees) {
  while (headingDegrees >= 360.0f) headingDegrees -= 360.0f;
  while (headingDegrees < 0.0f) headingDegrees += 360.0f;
  return headingDegrees;
}

LocalPoint geoToLocal(const GeoPoint& origin, const GeoPoint& point) {
  float lat0Rad = static_cast<float>(origin.lat * PI / 180.0);
  float dLatRad = static_cast<float>((point.lat - origin.lat) * PI / 180.0);
  float dLonRad = static_cast<float>((point.lon - origin.lon) * PI / 180.0);
  LocalPoint result;
  result.x = dLonRad * cosf(lat0Rad) * EARTH_RADIUS_M;
  result.y = dLatRad * EARTH_RADIUS_M;
  return result;
}

GeoPoint localToGeo(const GeoPoint& origin, const LocalPoint& point) {
  float lat0Rad = static_cast<float>(origin.lat * PI / 180.0);
  GeoPoint result;
  result.lat = origin.lat + static_cast<double>(point.y / EARTH_RADIUS_M) * 180.0 / PI;
  float cosLat = max(0.2f, fabsf(cosf(lat0Rad)));
  result.lon = origin.lon + static_cast<double>(point.x / (EARTH_RADIUS_M * cosLat)) * 180.0 / PI;
  return result;
}

float localLength(const LocalPoint& point) {
  return sqrtf(point.x * point.x + point.y * point.y);
}

float bearingFromLocalVector(float dxEast, float dyNorth) {
  return normalizeHeading360(atan2f(dxEast, dyNorth) * 180.0f / PI);
}

float distanceToGeo(const GeoPoint& point) {
  if (!gps.location.isValid()) return -1.0f;
  return static_cast<float>(TinyGPSPlus::distanceBetween(
    gps.location.lat(), gps.location.lng(), point.lat, point.lon
  ));
}

void setActiveTarget(const GeoPoint& point) {
  targetLatitude = point.lat;
  targetLongitude = point.lon;
  targetSet = true;
}

void updateTargetGeometry() {
  if (!targetSet || !gps.location.isValid()) {
    currentTargetDistanceM = -1.0;
    currentTargetBearingDeg = -1.0;
    return;
  }
  currentTargetDistanceM = TinyGPSPlus::distanceBetween(
    gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude
  );
  currentTargetBearingDeg = TinyGPSPlus::courseTo(
    gps.location.lat(), gps.location.lng(), targetLatitude, targetLongitude
  );
}

void saveMotorTrimToNvs() {
  preferences.putInt("rightTrim", rightForwardTrimUs);
}

void loadMotorTrimFromNvs() {
  rightForwardTrimUs = constrain(
    preferences.getInt("rightTrim", RIGHT_FORWARD_TRIM_DEFAULT_US),
    RIGHT_FORWARD_TRIM_MIN_US,
    RIGHT_FORWARD_TRIM_MAX_US
  );
}

void changeRightMotorTrim(int deltaUs) {
  rightForwardTrimUs = constrain(
    rightForwardTrimUs + deltaUs,
    RIGHT_FORWARD_TRIM_MIN_US,
    RIGHT_FORWARD_TRIM_MAX_US
  );
  saveMotorTrimToNvs();
  lastSystemEvent = "RIGHT_TRIM_" + String(rightForwardTrimUs) + "US";
}

void loadGyroSignFromNvs() {
  int storedSign = preferences.getInt("gyroSign", -1);
  gyroCompassSign = storedSign >= 0 ? 1 : -1;
  gyroSignVerified = preferences.getBool("gyroSignOK", false);
}

void requestGyroSignRelearn() {
  gyroSignVerified = false;
  gyroTurnLearning = false;
  preferences.putBool("gyroSignOK", false);
  lastSystemEvent = "GYRO_SIGN_RELEARN_ARMED";
}

void beginGyroTurnLearning(int8_t expectedDirection) {
  if (gyroSignVerified || gyroTurnLearning) return;
  gyroTurnLearning = true;
  gyroTurnExpectedDirection = expectedDirection;
  gyroTurnLearningStartedMs = millis();
  gyroTurnSampleSum = 0.0;
  gyroTurnSampleCount = 0;
  lastSystemEvent = expectedDirection > 0
    ? "LEARNING_GYRO_SIGN_RIGHT"
    : "LEARNING_GYRO_SIGN_LEFT";
}

void stopMotors();

void saveSurveyConfigToNvs() {
  preferences.putBool("surveyASet", surveyASet);
  preferences.putBool("surveyBSet", surveyBSet);
  preferences.putBool("surveyCSet", surveyCSet);
  preferences.putBool("surveyDSet", surveyDSet);
  if (surveyASet) { preferences.putDouble("surveyALat", surveyA.lat); preferences.putDouble("surveyALon", surveyA.lon); }
  if (surveyBSet) { preferences.putDouble("surveyBLat", surveyB.lat); preferences.putDouble("surveyBLon", surveyB.lon); }
  if (surveyCSet) { preferences.putDouble("surveyCLat", surveyC.lat); preferences.putDouble("surveyCLon", surveyC.lon); }
  if (surveyDSet) { preferences.putDouble("surveyDLat", surveyD.lat); preferences.putDouble("surveyDLon", surveyD.lon); }
  preferences.putFloat("surveySpace", surveyRequestedSpacingM);
  preferences.putUInt("surveyPhoto", surveyPhotoIntervalMs);
}

void loadSurveyConfigFromNvs() {
  surveyASet = preferences.getBool("surveyASet", false);
  surveyBSet = preferences.getBool("surveyBSet", false);
  surveyCSet = preferences.getBool("surveyCSet", false);
  surveyDSet = preferences.getBool("surveyDSet", false);
  if (surveyASet) { surveyA.lat = preferences.getDouble("surveyALat", 0.0); surveyA.lon = preferences.getDouble("surveyALon", 0.0); }
  if (surveyBSet) { surveyB.lat = preferences.getDouble("surveyBLat", 0.0); surveyB.lon = preferences.getDouble("surveyBLon", 0.0); }
  if (surveyCSet) { surveyC.lat = preferences.getDouble("surveyCLat", 0.0); surveyC.lon = preferences.getDouble("surveyCLon", 0.0); }
  if (surveyDSet) { surveyD.lat = preferences.getDouble("surveyDLat", 0.0); surveyD.lon = preferences.getDouble("surveyDLon", 0.0); }
  surveyRequestedSpacingM = constrain(preferences.getFloat("surveySpace", 1.5f), SURVEY_MIN_SPACING_M, SURVEY_MAX_SPACING_M);
  surveyPhotoIntervalMs = constrain(preferences.getUInt("surveyPhoto", 1100), SURVEY_MIN_PHOTO_INTERVAL_MS, SURVEY_MAX_PHOTO_INTERVAL_MS);
}

void clearSurveyArea() {
  surveyASet = surveyBSet = surveyCSet = surveyDSet = false;
  surveyRouteBuilt = false;
  surveyRoutePointCount = 0;
  surveyLaneCount = 0;
  surveyAreaM2 = 0.0f;
  surveyTotalRouteM = 0.0f;
  surveyState = SurveyState::IDLE;
  headingInitialized = false;
  bowLockPending = false;
  stopMotors();
  saveSurveyConfigToNvs();
  lastSystemEvent = "SURVEY_AREA_CLEARED";
}

void writeMotors(int leftUs, int rightUs) {
  // Apply trim only to forward motion, then clamp again. This prevents a large
  // positive trim from crossing 1500 us and accidentally stopping/reversing a motor.
  if (rightUs < ESC_NEUTRAL_US) {
    rightUs += rightForwardTrimUs;
    rightUs = constrain(rightUs, 1000, AUTO_NEUTRAL_LIMIT_US);
  }
  if (leftUs < ESC_NEUTRAL_US) {
    leftUs = constrain(leftUs, 1000, AUTO_NEUTRAL_LIMIT_US);
  }
  leftUs = constrain(leftUs, 1000, 2000);
  rightUs = constrain(rightUs, 1000, 2000);
  if (!motorsArmed) {
    leftUs = ESC_NEUTRAL_US;
    rightUs = ESC_NEUTRAL_US;
  }
  currentLeftUs = leftUs;
  currentRightUs = rightUs;
  escLeft.writeMicroseconds(currentLeftUs);
  escRight.writeMicroseconds(currentRightUs);
}

int rightRawForDesiredFinal(int desiredFinalUs) {
  if (desiredFinalUs >= ESC_NEUTRAL_US) return desiredFinalUs;
  return constrain(desiredFinalUs - rightForwardTrimUs, 1000, 1499);
}

void stopMotors() {
  currentLeftUs = ESC_NEUTRAL_US;
  currentRightUs = ESC_NEUTRAL_US;
  escLeft.writeMicroseconds(ESC_NEUTRAL_US);
  escRight.writeMicroseconds(ESC_NEUTRAL_US);
}

bool armMotors() {
  if (millis() - bootStartedMs < ESC_ARM_DELAY_MS) {
    lastSystemEvent = "ARM_BLOCKED_WAIT_12S";
    return false;
  }
  stopMotors();
  motorsArmed = true;
  lastSystemEvent = "MOTORS_ARMED";
  return true;
}

void stopAutonomy(const String& reason, bool failsafe) {
  manualDriveActive = false;
  stopMotors();
  surveyState = failsafe ? SurveyState::FAILSAFE : SurveyState::IDLE;
  autoMessage = reason;
  autoRunStartedMs = 0;
  activeLegIndex = -1;
  lastSystemEvent = failsafe ? ("FAILSAFE_" + reason) : reason;
}

void disarmMotors() {
  stopAutonomy("DISARMED", false);
  motorsArmed = false;
}

void startTimedMotorTest(int leftUs, int rightUs) {
  if (!motorsArmed) {
    lastSystemEvent = "MOTOR_TEST_BLOCKED_ARM";
    return;
  }
  stopAutonomy("MANUAL_TEST", false);
  writeMotors(leftUs, rightUs);
  motorTestStopAtMs = millis() + MOTOR_TEST_DURATION_MS;
}

void updateMotorTestTimeout() {
  if (motorTestStopAtMs != 0 && millis() >= motorTestStopAtMs) {
    motorTestStopAtMs = 0;
    stopMotors();
  }
}

bool gpsGoodForAuto() {
  if (!gpsHasFreshFix()) return false;
  if (!gps.satellites.isValid() || gps.satellites.value() < AUTO_MIN_SATELLITES) return false;
  if (gps.hdop.isValid() && gps.hdop.hdop() > AUTO_MAX_HDOP) return false;
  return true;
}

void startSurveyPointAveraging(SurveyPointSlot slot) {
  if (!gpsGoodForAuto()) {
    lastSystemEvent = "POINT_AVERAGE_BLOCKED_GPS";
    return;
  }
  stopAutonomy("POINT_AVERAGING", false);
  surveyPointAveragingSlot = slot;
  surveyPointAverageStartedMs = millis();
  surveyPointAverageLastSampleMs = 0;
  surveyPointAverageLatSum = 0.0;
  surveyPointAverageLonSum = 0.0;
  surveyPointAverageSamples = 0;
  lastSystemEvent = slot == SurveyPointSlot::A ? "AVERAGING_A" :
                    slot == SurveyPointSlot::B ? "AVERAGING_B" :
                    slot == SurveyPointSlot::C ? "AVERAGING_C" : "AVERAGING_D";
}

void updateSurveyPointAveraging() {
  if (surveyPointAveragingSlot == SurveyPointSlot::NONE) return;
  uint32_t nowMs = millis();
  if (gpsGoodForAuto() && (surveyPointAverageLastSampleMs == 0 || nowMs - surveyPointAverageLastSampleMs >= SURVEY_POINT_AVERAGE_SAMPLE_INTERVAL_MS)) {
    surveyPointAverageLastSampleMs = nowMs;
    surveyPointAverageLatSum += gps.location.lat();
    surveyPointAverageLonSum += gps.location.lng();
    surveyPointAverageSamples++;
  }
  if (nowMs - surveyPointAverageStartedMs < SURVEY_POINT_AVERAGE_DURATION_MS) return;
  SurveyPointSlot completedSlot = surveyPointAveragingSlot;
  surveyPointAveragingSlot = SurveyPointSlot::NONE;
  if (surveyPointAverageSamples < 12) {
    lastSystemEvent = "POINT_AVERAGE_FAILED";
    return;
  }
  GeoPoint result;
  result.lat = surveyPointAverageLatSum / static_cast<double>(surveyPointAverageSamples);
  result.lon = surveyPointAverageLonSum / static_cast<double>(surveyPointAverageSamples);
  if (completedSlot == SurveyPointSlot::A) { surveyA = result; surveyASet = true; lastSystemEvent = "SURVEY_A_SAVED"; }
  if (completedSlot == SurveyPointSlot::B) { surveyB = result; surveyBSet = true; lastSystemEvent = "SURVEY_B_SAVED"; }
  if (completedSlot == SurveyPointSlot::C) { surveyC = result; surveyCSet = true; lastSystemEvent = "SURVEY_C_SAVED"; }
  if (completedSlot == SurveyPointSlot::D) { surveyD = result; surveyDSet = true; lastSystemEvent = "SURVEY_D_SAVED"; }
  surveyRouteBuilt = false;
  headingInitialized = false;
  saveSurveyConfigToNvs();
}

float cross2D(const LocalPoint& a, const LocalPoint& b, const LocalPoint& c) {
  return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

float localDistance(const LocalPoint& a, const LocalPoint& b) {
  float dx = b.x - a.x;
  float dy = b.y - a.y;
  return sqrtf(dx * dx + dy * dy);
}

LocalPoint interpolateLocal(const LocalPoint& a, const LocalPoint& b, float t) {
  LocalPoint result;
  result.x = a.x + (b.x - a.x) * t;
  result.y = a.y + (b.y - a.y) * t;
  return result;
}

float polygonArea4(const LocalPoint& a, const LocalPoint& b, const LocalPoint& c, const LocalPoint& d) {
  float sum = a.x * b.y - a.y * b.x;
  sum += b.x * c.y - b.y * c.x;
  sum += c.x * d.y - c.y * d.x;
  sum += d.x * a.y - d.y * a.x;
  return fabsf(sum) * 0.5f;
}

bool orderedConvexQuadrilateral(
  const LocalPoint& a,
  const LocalPoint& b,
  const LocalPoint& c,
  const LocalPoint& d
) {
  float z1 = cross2D(a, b, c);
  float z2 = cross2D(b, c, d);
  float z3 = cross2D(c, d, a);
  float z4 = cross2D(d, a, b);
  constexpr float MIN_CROSS = 0.35f;
  if (fabsf(z1) < MIN_CROSS || fabsf(z2) < MIN_CROSS ||
      fabsf(z3) < MIN_CROSS || fabsf(z4) < MIN_CROSS) {
    return false;
  }
  bool positive = z1 > 0.0f;
  return (z2 > 0.0f) == positive &&
         (z3 > 0.0f) == positive &&
         (z4 > 0.0f) == positive;
}

bool validGeoCoordinate(double latitude, double longitude) {
  return isfinite(latitude) && isfinite(longitude) &&
         latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0 &&
         (fabs(latitude) > 0.000001 || fabs(longitude) > 0.000001);
}

bool setSurveyCorner(SurveyPointSlot slot, double latitude, double longitude) {
  if (slot == SurveyPointSlot::NONE || !validGeoCoordinate(latitude, longitude)) {
    lastSystemEvent = "CORNER_SET_INVALID";
    return false;
  }
  GeoPoint point = {latitude, longitude};
  if (slot == SurveyPointSlot::A) { surveyA = point; surveyASet = true; lastSystemEvent = "SURVEY_A_SET_MANUAL"; }
  if (slot == SurveyPointSlot::B) { surveyB = point; surveyBSet = true; lastSystemEvent = "SURVEY_B_SET_MANUAL"; }
  if (slot == SurveyPointSlot::C) { surveyC = point; surveyCSet = true; lastSystemEvent = "SURVEY_C_SET_MANUAL"; }
  if (slot == SurveyPointSlot::D) { surveyD = point; surveyDSet = true; lastSystemEvent = "SURVEY_D_SET_MANUAL"; }
  surveyRouteBuilt = false;
  headingInitialized = false;
  bowLockPending = false;
  saveSurveyConfigToNvs();
  return true;
}

bool buildSurveyRoute() {
  if (!surveyASet || !surveyBSet || !surveyCSet || !surveyDSet) {
    lastSystemEvent = "BUILD_BLOCKED_SAVE_A_B_C_D";
    return false;
  }

  const LocalPoint a = {0.0f, 0.0f};
  const LocalPoint b = geoToLocal(surveyA, surveyB);
  const LocalPoint c = geoToLocal(surveyA, surveyC);
  const LocalPoint d = geoToLocal(surveyA, surveyD);

  if (!orderedConvexQuadrilateral(a, b, c, d)) {
    lastSystemEvent = "BUILD_BAD_POINT_ORDER_USE_A_B_C_D_AROUND_EDGE";
    return false;
  }

  const float firstStripLength = localDistance(a, b);
  const float lastStripLength = localDistance(d, c);
  const float leftSideWidth = localDistance(a, d);
  const float rightSideWidth = localDistance(b, c);
  const float maximumWidth = max(leftSideWidth, rightSideWidth);

  if (firstStripLength < SURVEY_MIN_LENGTH_M || firstStripLength > SURVEY_MAX_LENGTH_M ||
      lastStripLength < SURVEY_MIN_LENGTH_M || lastStripLength > SURVEY_MAX_LENGTH_M) {
    lastSystemEvent = "BUILD_BAD_STRIP_LENGTH";
    return false;
  }
  if (leftSideWidth < SURVEY_MIN_WIDTH_M || leftSideWidth > SURVEY_MAX_WIDTH_M ||
      rightSideWidth < SURVEY_MIN_WIDTH_M || rightSideWidth > SURVEY_MAX_WIDTH_M) {
    lastSystemEvent = "BUILD_BAD_SIDE_WIDTH";
    return false;
  }

  // Use the wider side to choose lane count. Therefore spacing on neither side
  // exceeds the requested value, which is important for complete photo coverage.
  uint8_t laneCount = static_cast<uint8_t>(ceilf(maximumWidth / surveyRequestedSpacingM)) + 1;
  if (laneCount < 2) laneCount = 2;
  if (laneCount > MAX_SURVEY_LANES) {
    lastSystemEvent = "BUILD_TOO_MANY_LANES_INCREASE_SPACING";
    return false;
  }

  uint8_t pointIndex = 0;
  float totalDistance = 0.0f;
  LocalPoint previousEnd = a;
  bool previousValid = false;

  for (uint8_t lane = 0; lane < laneCount; lane++) {
    float t = laneCount <= 1 ? 0.0f : static_cast<float>(lane) / static_cast<float>(laneCount - 1);
    LocalPoint left = interpolateLocal(a, d, t);
    LocalPoint right = interpolateLocal(b, c, t);
    float stripLength = localDistance(left, right);
    if (stripLength < SURVEY_MIN_LENGTH_M) {
      lastSystemEvent = "BUILD_STRIP_TOO_SHORT";
      return false;
    }

    LocalPoint start = (lane & 1U) == 0U ? left : right;
    LocalPoint finish = (lane & 1U) == 0U ? right : left;

    if (previousValid) totalDistance += localDistance(previousEnd, start);
    totalDistance += stripLength;
    previousEnd = finish;
    previousValid = true;

    surveyRoute[pointIndex++] = localToGeo(surveyA, start);
    surveyRoute[pointIndex++] = localToGeo(surveyA, finish);
  }

  surveyRoutePointCount = pointIndex;
  surveyLaneCount = laneCount;
  surveyLengthM = 0.5f * (firstStripLength + lastStripLength);
  surveyWidthM = 0.5f * (leftSideWidth + rightSideWidth);
  surveyAreaM2 = polygonArea4(a, b, c, d);
  surveyActualSpacingM = maximumWidth / static_cast<float>(laneCount - 1);
  surveyTotalRouteM = totalDistance;
  surveyRouteBuilt = true;
  headingInitialized = false;
  bowLockPending = false;
  surveyState = SurveyState::IDLE;
  saveSurveyConfigToNvs();
  lastSystemEvent = "SURVEY_ROUTE_BUILT_" + String(laneCount) + "_LANES";
  return true;
}

void configureSurvey(float spacingM, uint32_t photoIntervalMs) {
  surveyRequestedSpacingM = constrain(spacingM, SURVEY_MIN_SPACING_M, SURVEY_MAX_SPACING_M);
  surveyPhotoIntervalMs = constrain(photoIntervalMs, SURVEY_MIN_PHOTO_INTERVAL_MS, SURVEY_MAX_PHOTO_INTERVAL_MS);
  surveyRouteBuilt = false;
  saveSurveyConfigToNvs();
  lastSystemEvent = "SURVEY_CONFIG_SAVED";
}

void resetNavigationController() {
  steeringCorrectionUs = 0.0f;
  lastSteeringUpdateMs = millis();
  lastGpsHeadingCorrectionMs = millis();
  gpsHeadingCorrectionTotalDeg = 0.0f;
}

bool chooseSurveyLockTarget(GeoPoint& target, bool& approachStart) {
  if (!surveyRouteBuilt || surveyRoutePointCount < 2 || !gps.location.isValid()) return false;
  float startDistance = distanceToGeo(surveyRoute[0]);
  if (startDistance < 0.0f || startDistance > SURVEY_START_NEAR_M) return false;
  approachStart = false;
  target = surveyRoute[1];
  return true;
}

bool startBowLockCalibration() {
  GeoPoint lockTarget;
  bool approachStart = false;
  if (!chooseSurveyLockTarget(lockTarget, approachStart)) {
    lastSystemEvent = "BOW_LOCK_BLOCKED_BUILD_ROUTE_OR_MOVE_NEAR_A";
    return false;
  }
  if (!gpsGoodForAuto()) { lastSystemEvent = "BOW_LOCK_BLOCKED_GPS"; return false; }
  if (!gyroSignVerified) { lastSystemEvent = "BOW_LOCK_BLOCKED_GYRO_SIGN"; return false; }
  if (millis() - lastMpuGoodMs > MPU_STALE_TIMEOUT_MS) { lastSystemEvent = "BOW_LOCK_BLOCKED_MPU"; return false; }
  stopAutonomy("BOW_LOCK_PREPARE", false);
  setActiveTarget(lockTarget);
  updateTargetGeometry();
  bowLockPending = true;
  headingInitialized = false;
  bowLockStartedMs = millis();
  bowLockLastSampleMs = 0;
  bowLockGyroSum = 0.0;
  bowLockAbsGyroSum = 0.0;
  bowLockBearingSinSum = 0.0;
  bowLockBearingCosSum = 0.0;
  bowLockSampleCount = 0;
  lastSystemEvent = approachStart ? "LOCK_BOW_TO_A_HOLD_2S" : "LOCK_BOW_TO_FIRST_STRIP_HOLD_2S";
  return true;
}

void clearBowLock() {
  bowLockPending = false;
  headingInitialized = false;
  headingEstimateDeg = 0.0f;
  currentHeadingErrorDeg = 0.0;
  gpsHeadingCorrectionTotalDeg = 0.0f;
  lastSystemEvent = "BOW_LOCK_CLEARED";
}

void updateBowLockCalibration() {
  if (!bowLockPending) return;
  uint32_t nowMs = millis();
  if (!gpsGoodForAuto() || nowMs - lastMpuGoodMs > MPU_STALE_TIMEOUT_MS) {
    bowLockPending = false;
    lastSystemEvent = "BOW_LOCK_FAILED_SENSOR";
    return;
  }
  if (bowLockLastSampleMs == 0 || nowMs - bowLockLastSampleMs >= BOW_LOCK_SAMPLE_INTERVAL_MS) {
    bowLockLastSampleMs = nowMs;
    updateTargetGeometry();
    if (currentTargetBearingDeg >= 0.0) {
      float bearingRad = static_cast<float>(currentTargetBearingDeg) * PI / 180.0f;
      bowLockBearingSinSum += sinf(bearingRad);
      bowLockBearingCosSum += cosf(bearingRad);
      bowLockGyroSum += currentGyroZ;
      bowLockAbsGyroSum += fabsf(currentGyroZ);
      bowLockSampleCount++;
    }
  }
  if (nowMs - bowLockStartedMs < BOW_LOCK_DURATION_MS) return;
  if (bowLockSampleCount < 20) { bowLockPending = false; lastSystemEvent = "BOW_LOCK_FAILED_SAMPLES"; return; }
  float averageGyroZ = static_cast<float>(bowLockGyroSum / static_cast<double>(bowLockSampleCount));
  float averageAbsGyroZ = static_cast<float>(bowLockAbsGyroSum / static_cast<double>(bowLockSampleCount));
  if (fabsf(averageGyroZ) > BOW_LOCK_MAX_AVERAGE_GYRO_DPS || averageAbsGyroZ > BOW_LOCK_MAX_AVERAGE_ABS_GYRO_DPS) {
    bowLockPending = false;
    lastSystemEvent = "BOW_LOCK_FAILED_MOVING";
    return;
  }
  gyroOffsetZ += averageGyroZ;
  currentGyroZ -= averageGyroZ;
  float averageBearingDeg = atan2f(static_cast<float>(bowLockBearingSinSum), static_cast<float>(bowLockBearingCosSum)) * 180.0f / PI;
  headingEstimateDeg = normalizeHeading360(averageBearingDeg);
  headingInitialized = true;
  headingLockedAtMs = nowMs;
  lastGpsHeadingCorrectionMs = nowMs;
  gpsHeadingCorrectionTotalDeg = 0.0f;
  currentHeadingErrorDeg = 0.0;
  bowLockPending = false;
  lastSystemEvent = "BOW_LOCKED_SURVEY_READY";
}

void updateHeadingGpsCorrection() {
  if (!headingInitialized || surveyState != SurveyState::TRACK_LEG) return;
  uint32_t nowMs = millis();
  if (nowMs - lastGpsHeadingCorrectionMs < HEADING_GPS_CORRECTION_INTERVAL_MS) return;
  lastGpsHeadingCorrectionMs = nowMs;
  if (!gps.course.isValid() || gps.course.age() > AUTO_MAX_COURSE_AGE_MS) return;
  if (!gps.speed.isValid() || gps.speed.kmph() < HEADING_GPS_MIN_SPEED_KMPH) return;
  float compassYawRate = static_cast<float>(gyroCompassSign) * currentGyroZ;
  if (fabsf(compassYawRate) > HEADING_GPS_MAX_YAW_RATE_DPS) return;
  int rightPreTrim = currentRightUs < ESC_NEUTRAL_US ? currentRightUs - rightForwardTrimUs : currentRightUs;
  if (abs(currentLeftUs - rightPreTrim) > 16) return;
  float courseDelta = normalizeHeadingError(static_cast<float>(gps.course.deg()) - headingEstimateDeg);
  if (fabsf(courseDelta) > HEADING_GPS_MAX_COURSE_DELTA_DEG) return;
  float correctionStep = constrain(courseDelta * HEADING_GPS_CORRECTION_GAIN, -HEADING_GPS_MAX_STEP_DEG, HEADING_GPS_MAX_STEP_DEG);
  headingEstimateDeg = normalizeHeading360(headingEstimateDeg + correctionStep);
  gpsHeadingCorrectionTotalDeg += correctionStep;
}

bool requestSurveyCapture() {
  pendingCaptureIsSurvey = true;
  bool ok = requestCapture(true);
  if (!ok) {
    pendingCaptureIsSurvey = false;
    return false;
  }
  surveyPhotosRequested++;
  lastSurveyPhotoRequestMs = millis();
  return true;
}

bool surveyPhotoQualityOkay() {
  return levelEnough &&
         fabsf(relativeRollDegrees) <= 16.0f &&
         fabsf(relativePitchDegrees) <= 16.0f &&
         fabsf(currentGyroX) <= 10.0f &&
         fabsf(currentGyroY) <= 10.0f &&
         fabsf(currentGyroZ) <= 14.0f;
}

void updateSurveyCapture() {
  if (surveyState != SurveyState::TRACK_LEG || !activeLegIsSurvey) return;
  if (captureRequestPending || !missionActive || !cameraOnline) return;
  uint32_t nowMs = millis();
  bool firstPhoto = lastSurveyPhotoRequestMs == 0 || lastSurveyPhotoRequestMs < surveyLegStartedMs;
  uint32_t elapsed = firstPhoto ? surveyPhotoIntervalMs : nowMs - lastSurveyPhotoRequestMs;
  if (!firstPhoto && elapsed < surveyPhotoIntervalMs) return;
  bool quality = surveyPhotoQualityOkay();
  bool coverageDeadline = firstPhoto || elapsed >= surveyPhotoIntervalMs * 2UL;
  if (quality || coverageDeadline) requestSurveyCapture();
}

void beginRouteLeg(uint8_t legIndex) {
  if (!surveyRouteBuilt || legIndex + 1 >= surveyRoutePointCount) {
    surveyState = SurveyState::FINISH_WAIT;
    autoMessage = "ROUTE_DONE";
    return;
  }
  activeLegIndex = legIndex;
  activeLegIsSurvey = (legIndex % 2) == 0;
  setActiveTarget(surveyRoute[legIndex + 1]);
  GeoPoint legStart = surveyRoute[legIndex];
  GeoPoint legEnd = surveyRoute[legIndex + 1];
  LocalPoint endLocal = geoToLocal(legStart, legEnd);
  activeLegLengthM = localLength(endLocal);
  activeLegAlongM = 0.0f;
  activeLegCrossTrackM = 0.0f;
  activeLegProgressPercent = 0.0f;
  activeLegStartedMs = millis();
  turnAlignedStartedMs = 0;
  finalPhotoRequestedForLeg = false;
  steeringCorrectionUs = 0.0f;
  surveyState = SurveyState::TURN_TO_LEG;
  autoMessage = activeLegIsSurvey ? "TURN_TO_SURVEY_STRIP" : "TURN_TO_CONNECTOR";
  lastSystemEvent = "BEGIN_LEG_" + String(legIndex + 1) + "_OF_" + String(surveyRoutePointCount - 1);
}

void beginApproachStart() {
  activeLegIndex = -1;
  activeLegIsSurvey = false;
  setActiveTarget(surveyRoute[0]);
  activeLegLengthM = max(0.1f, distanceToGeo(surveyRoute[0]));
  activeLegAlongM = 0.0f;
  activeLegCrossTrackM = 0.0f;
  activeLegProgressPercent = 0.0f;
  activeLegStartedMs = millis();
  turnAlignedStartedMs = 0;
  surveyState = SurveyState::TURN_TO_START;
  autoMessage = "TURN_TO_A";
}

String surveyStartBlockReason() {
  if (surveyRunning()) return "SURVEY_ALREADY_RUNNING";
  if (!surveyRouteBuilt || surveyRoutePointCount < 2) return "BUILD_ROUTE";
  if (!motorsArmed) return "ARM_MOTORS";
  if (!gpsGoodForAuto()) return "GPS_QUALITY";
  if (!gyroSignVerified) return "VERIFY_GYRO_SIGN";
  if (!headingInitialized || bowLockPending) return "LOCK_BOW";
  if (millis() - headingLockedAtMs > BOW_LOCK_MAX_START_AGE_MS) return "LOCK_BOW_AGAIN";
  if (millis() - lastMpuGoodMs > MPU_STALE_TIMEOUT_MS) return "MPU_DATA";
  if (!cameraOnline || !cameraRemoteReady || !cameraSdReady) return "CAMERA_OR_SD";
  if (!missionActive) return "START_PHOTO_MISSION";
  if (captureRequestPending) return "CAMERA_BUSY";
  float distanceFromA = distanceToGeo(surveyA);
  if (distanceFromA < 0.0f) return "GPS_POSITION";
  if (distanceFromA > SURVEY_START_NEAR_M) return "MOVE_NEAR_A";
  return "";
}

bool startSurvey() {
  String blocked = surveyStartBlockReason();
  if (blocked.length() > 0) {
    lastSystemEvent = "START_BLOCKED_" + blocked;
    return false;
  }
  autoStartLatitude = gps.location.lat();
  autoStartLongitude = gps.location.lng();
  autoRunStartedMs = millis();
  resetNavigationController();
  surveyPhotosRequested = 0;
  surveyPhotosSaved = 0;
  surveyPhotosFailed = 0;
  lastSurveyPhotoRequestMs = 0;
  pendingCaptureIsSurvey = false;
  lastCaptureResult = 0;
  beginRouteLeg(0);
  lastSystemEvent = "SURVEY_STARTED";
  return true;
}

void commandTurnToBearing(float desiredBearing) {
  desiredTrackBearingDeg = normalizeHeading360(desiredBearing);
  currentHeadingErrorDeg = normalizeHeadingError(desiredTrackBearingDeg - headingEstimateDeg);
  float absError = fabsf(static_cast<float>(currentHeadingErrorDeg));
  float yawRate = static_cast<float>(gyroCompassSign) * currentGyroZ;
  if (absError <= TURN_ACCEPT_DEG && fabsf(yawRate) <= 9.0f) {
    stopMotors();
    if (turnAlignedStartedMs == 0) turnAlignedStartedMs = millis();
    return;
  }
  turnAlignedStartedMs = 0;
  int turnFinal = absError > 35.0f ? TURN_FAST_FINAL_US : TURN_SLOW_FINAL_US;
  if (currentHeadingErrorDeg > 0.0) {
    writeMotors(turnFinal, ESC_NEUTRAL_US);  // right turn
  } else {
    writeMotors(ESC_NEUTRAL_US, rightRawForDesiredFinal(turnFinal));  // left turn
  }
}

bool turnAlignmentComplete() {
  return turnAlignedStartedMs != 0 && millis() - turnAlignedStartedMs >= TURN_ALIGNED_HOLD_MS;
}

float computeLegDesiredBearing() {
  if (activeLegIndex < 0) {
    GeoPoint current = {gps.location.lat(), gps.location.lng()};
    LocalPoint targetLocal = geoToLocal(current, surveyRoute[0]);
    activeLegCrossTrackM = 0.0f;
    activeLegAlongM = max(0.0f, activeLegLengthM - localLength(targetLocal));
    activeLegProgressPercent = constrain(100.0f * activeLegAlongM / max(0.1f, activeLegLengthM), 0.0f, 100.0f);
    return bearingFromLocalVector(targetLocal.x, targetLocal.y);
  }
  GeoPoint legStart = surveyRoute[activeLegIndex];
  GeoPoint legEnd = surveyRoute[activeLegIndex + 1];
  GeoPoint current = {gps.location.lat(), gps.location.lng()};
  LocalPoint end = geoToLocal(legStart, legEnd);
  LocalPoint position = geoToLocal(legStart, current);
  float length = max(0.1f, localLength(end));
  float ux = end.x / length;
  float uy = end.y / length;
  float nx = -uy;
  float ny = ux;
  activeLegAlongM = position.x * ux + position.y * uy;
  activeLegCrossTrackM = position.x * nx + position.y * ny;
  activeLegProgressPercent = constrain(100.0f * activeLegAlongM / length, 0.0f, 100.0f);
  float lookahead = activeLegIsSurvey ? SURVEY_LOOKAHEAD_M : CONNECTOR_LOOKAHEAD_M;
  float desiredAlong = constrain(activeLegAlongM + lookahead, 0.0f, length);
  LocalPoint desired = {ux * desiredAlong, uy * desiredAlong};
  float dx = desired.x - position.x;
  float dy = desired.y - position.y;
  if (sqrtf(dx * dx + dy * dy) < 0.5f) {
    dx = end.x - position.x;
    dy = end.y - position.y;
  }
  return bearingFromLocalVector(dx, dy);
}

bool activeLegReached() {
  if (activeLegIndex < 0) return currentTargetDistanceM >= 0.0 && currentTargetDistanceM <= SURVEY_START_REACHED_M;
  if (activeLegLengthM <= 4.5f) {
    return activeLegAlongM >= activeLegLengthM * 0.70f ||
           (currentTargetDistanceM >= 0.0 && currentTargetDistanceM <= SURVEY_CONNECTOR_RADIUS_M);
  }
  return (activeLegAlongM >= activeLegLengthM - 0.7f && currentTargetDistanceM <= SURVEY_ENDPOINT_RADIUS_M) ||
         activeLegAlongM >= activeLegLengthM + 0.4f;
}

void driveCurrentLeg() {
  desiredTrackBearingDeg = computeLegDesiredBearing();
  currentHeadingErrorDeg = normalizeHeadingError(desiredTrackBearingDeg - headingEstimateDeg);
  if (fabsf(static_cast<float>(currentHeadingErrorDeg)) < SURVEY_HEADING_DEADBAND_DEG) currentHeadingErrorDeg = 0.0f;
  float yawRate = static_cast<float>(gyroCompassSign) * currentGyroZ;
  float targetCorrection = SURVEY_HEADING_KP_US_PER_DEG * static_cast<float>(currentHeadingErrorDeg) - SURVEY_YAW_DAMPING_US_PER_DPS * yawRate;
  targetCorrection = constrain(targetCorrection, -SURVEY_MAX_STEERING_US, SURVEY_MAX_STEERING_US);
  float step = constrain(targetCorrection - steeringCorrectionUs, -SURVEY_STEERING_SLEW_US, SURVEY_STEERING_SLEW_US);
  steeringCorrectionUs += step;
  int baseUs = activeLegIsSurvey ? SURVEY_BASE_US : CONNECTOR_BASE_US;
  float absError = fabsf(static_cast<float>(currentHeadingErrorDeg));
  if (absError > 25.0f) baseUs = max(baseUs, 1405);
  if (absError > 50.0f) baseUs = max(baseUs, 1430);
  int leftUs = static_cast<int>(roundf(baseUs - steeringCorrectionUs));
  int rightUs = static_cast<int>(roundf(baseUs + steeringCorrectionUs));
  int rightRawWeakLimit = SURVEY_WEAKEST_FORWARD_US - max(0, rightForwardTrimUs);
  rightRawWeakLimit = constrain(rightRawWeakLimit, SURVEY_FORWARD_LIMIT_US, SURVEY_WEAKEST_FORWARD_US);
  leftUs = constrain(leftUs, SURVEY_FORWARD_LIMIT_US, SURVEY_WEAKEST_FORWARD_US);
  rightUs = constrain(rightUs, SURVEY_FORWARD_LIMIT_US, rightRawWeakLimit);
  writeMotors(leftUs, rightUs);
}

void advanceAfterWaypoint() {
  if (activeLegIndex < 0) {
    beginRouteLeg(0);
    return;
  }
  uint8_t nextLeg = static_cast<uint8_t>(activeLegIndex + 1);
  if (nextLeg + 1 >= surveyRoutePointCount) {
    surveyState = SurveyState::FINISH_WAIT;
    autoMessage = "WAIT_FINAL_CAMERA";
    stopMotors();
    return;
  }
  beginRouteLeg(nextLeg);
}

void updateAutopilot() {
  updateMotorTestTimeout();
  uint32_t nowMs = millis();
  if (nowMs - previousAutoUpdateMs < AUTO_UPDATE_INTERVAL_MS) return;
  previousAutoUpdateMs = nowMs;
  updateTargetGeometry();
  if (!surveyRunning()) return;
  if (!motorsArmed) { stopAutonomy("MOTORS_DISARMED", true); return; }
  if (!gpsGoodForAuto()) { stopAutonomy("GPS_LOST_OR_POOR", true); return; }
  if (!headingInitialized) { stopAutonomy("HEADING_REFERENCE_LOST", true); return; }
  if (nowMs - lastMpuGoodMs > MPU_STALE_TIMEOUT_MS) { stopAutonomy("MPU_DATA_LOST", true); return; }
  if (autoRunStartedMs != 0 && nowMs - autoRunStartedMs > AUTO_MAX_RUN_TIME_MS) { stopAutonomy("SURVEY_TIME_LIMIT", true); return; }
  if (distanceToGeo(surveyA) > AUTO_MAX_DISTANCE_FROM_A_M) { stopAutonomy("GEOFENCE_FROM_A", true); return; }
  if (fabsf(relativeRollDegrees) > AUTO_MAX_ROLL_DEG || fabsf(relativePitchDegrees) > AUTO_MAX_PITCH_DEG) { stopAutonomy("EXCESSIVE_TILT", true); return; }
  if (!cameraOnline || !cameraRemoteReady || !cameraSdReady) { stopAutonomy("CAMERA_OR_SD_LOST", true); return; }
  if (captureRequestPending && nowMs - captureRequestStartedMs > AUTO_CAPTURE_TIMEOUT_MS) {
    if (pendingCaptureIsSurvey) surveyPhotosFailed++;
    captureRequestPending = false;
    pendingCaptureIsSurvey = false;
    stopAutonomy("CAMERA_CAPTURE_TIMEOUT", true);
    return;
  }
  updateHeadingGpsCorrection();
  switch (surveyState) {
    case SurveyState::TURN_TO_START: {
      float bearing = static_cast<float>(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), surveyRoute[0].lat, surveyRoute[0].lon));
      commandTurnToBearing(bearing);
      if (turnAlignmentComplete()) {
        surveyState = SurveyState::APPROACH_START;
        activeLegStartedMs = nowMs;
        steeringCorrectionUs = 0.0f;
        autoMessage = "GO_TO_A";
      } else if (nowMs - activeLegStartedMs > TURN_TIMEOUT_MS) stopAutonomy("TURN_TO_A_TIMEOUT", true);
      break;
    }
    case SurveyState::APPROACH_START:
      if (activeLegReached()) {
        stopMotors();
        waypointHoldStartedMs = nowMs;
        surveyState = SurveyState::WAYPOINT_HOLD;
        autoMessage = "A_REACHED";
      } else driveCurrentLeg();
      break;
    case SurveyState::TURN_TO_LEG: {
      GeoPoint legStart = surveyRoute[activeLegIndex];
      GeoPoint legEnd = surveyRoute[activeLegIndex + 1];
      float bearing = static_cast<float>(TinyGPSPlus::courseTo(legStart.lat, legStart.lon, legEnd.lat, legEnd.lon));
      commandTurnToBearing(bearing);
      if (turnAlignmentComplete()) {
        surveyState = SurveyState::TRACK_LEG;
        activeLegStartedMs = nowMs;
        surveyLegStartedMs = nowMs;
        lastSurveyPhotoRequestMs = 0;
        steeringCorrectionUs = 0.0f;
        autoMessage = activeLegIsSurvey ? "CAPTURING_STRIP" : "MOVE_TO_NEXT_STRIP";
      } else if (nowMs - activeLegStartedMs > TURN_TIMEOUT_MS) stopAutonomy("TURN_LEG_TIMEOUT", true);
      break;
    }
    case SurveyState::TRACK_LEG:
      if (activeLegReached()) {
        stopMotors();
        waypointHoldStartedMs = nowMs;
        finalPhotoRequestedForLeg = false;
        surveyState = SurveyState::WAYPOINT_HOLD;
        autoMessage = "WAYPOINT_REACHED";
      } else {
        driveCurrentLeg();
        updateSurveyCapture();
      }
      break;
    case SurveyState::WAYPOINT_HOLD:
      stopMotors();
      if (activeLegIndex >= 0 && activeLegIsSurvey && !finalPhotoRequestedForLeg && !captureRequestPending && nowMs - lastSurveyPhotoRequestMs >= 650) {
        finalPhotoRequestedForLeg = true;
        requestSurveyCapture();
      }
      if (nowMs - waypointHoldStartedMs >= WAYPOINT_HOLD_MS && !captureRequestPending) advanceAfterWaypoint();
      break;
    case SurveyState::FINISH_WAIT:
      stopMotors();
      if (!captureRequestPending) {
        if (missionActive) requestMissionEnd();
        surveyState = SurveyState::COMPLETE;
        autoMessage = "SURVEY_COMPLETE";
        autoRunStartedMs = 0;
        lastSystemEvent = "SURVEY_COMPLETE_PHOTOS_" + String(surveyPhotosSaved);
      }
      break;
    default:
      break;
  }
}

void printNavigationStatus() {
  Serial.println();
  Serial.println("========== SURVEY ==========");
  Serial.print("State: "); Serial.println(surveyStateName());
  Serial.print("Route built: "); Serial.println(surveyRouteBuilt ? "YES" : "NO");
  Serial.print("Lanes: "); Serial.print(surveyLaneCount); Serial.print(" spacing: "); Serial.println(surveyActualSpacingM, 2);
  Serial.print("Leg: "); Serial.print(activeLegIndex + 1); Serial.print("/"); Serial.println(max(0, static_cast<int>(surveyRoutePointCount) - 1));
  Serial.print("Progress: "); Serial.print(activeLegProgressPercent, 1); Serial.print("% XTE: "); Serial.println(activeLegCrossTrackM, 2);
  Serial.print("Heading: "); Serial.print(headingEstimateDeg, 1); Serial.print(" desired: "); Serial.print(desiredTrackBearingDeg, 1); Serial.print(" error: "); Serial.println(currentHeadingErrorDeg, 1);
  Serial.print("Photos requested/saved/failed: "); Serial.print(surveyPhotosRequested); Serial.print("/"); Serial.print(surveyPhotosSaved); Serial.print("/"); Serial.println(surveyPhotosFailed);
  Serial.println("============================");
}

// ============================================================
// Serial commands + touch-safe mobile control page
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("========== V5.1 MATCH SURVEY COMMANDS ==========");
  Serial.println("M / E              - start/end camera mission");
  Serial.println("ARM / DISARM / STOP");
  Serial.println("A_HERE / B_HERE / C_HERE / D_HERE - average ordered corners");
  Serial.println("BUILD               - build lawnmower route");
  Serial.println("LOCKBOW             - point bow at shown first target and lock 2 s");
  Serial.println("SURVEY              - start lawnmower mission");
  Serial.println("NAV                 - print survey status");
  Serial.println("========================================");
}

String usbInputLine;

void processUsbCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  String upper = command;
  upper.toUpperCase();
  if (upper == "M") requestMissionStart();
  else if (upper == "E") requestMissionEnd();
  else if (upper == "F") { pendingCaptureIsSurvey = false; requestCapture(true); }
  else if (upper == "ARM") armMotors();
  else if (upper == "DISARM") disarmMotors();
  else if (upper == "STOP" || upper == "ABORT") stopAutonomy("USER_ABORT", false);
  else if (upper == "A_HERE") startSurveyPointAveraging(SurveyPointSlot::A);
  else if (upper == "B_HERE") startSurveyPointAveraging(SurveyPointSlot::B);
  else if (upper == "C_HERE") startSurveyPointAveraging(SurveyPointSlot::C);
  else if (upper == "D_HERE") startSurveyPointAveraging(SurveyPointSlot::D);
  else if (upper == "BUILD") buildSurveyRoute();
  else if (upper == "LOCKBOW") startBowLockCalibration();
  else if (upper == "SURVEY") startSurvey();
  else if (upper == "NAV") printNavigationStatus();
  else if (upper == "GYROSIGNRESET") requestGyroSignRelearn();
  else if (upper == "R") calibrateRobotPosition();
  else printHelp();
}

void checkUsbSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      processUsbCommand(usbInputLine);
      usbInputLine = "";
      continue;
    }
    if (usbInputLine.length() < 160) usbInputLine += incoming;
    else usbInputLine = "";
  }
}

void manualMotorCommand(const String& command) {
  if (!motorsArmed) { stopMotors(); return; }
  if (surveyRunning()) stopAutonomy("MANUAL_OVERRIDE", false);
  if (command == "0") {
    manualDriveActive = false;
    stopMotors();
    if (gyroTurnLearning) {
      uint32_t duration = millis() - gyroTurnLearningStartedMs;
      float averageGyroZ = gyroTurnSampleCount > 0 ? static_cast<float>(gyroTurnSampleSum / gyroTurnSampleCount) : 0.0f;
      if (duration >= 800 && fabsf(averageGyroZ) >= 5.0f) {
        gyroCompassSign = (gyroTurnExpectedDirection * averageGyroZ >= 0.0f) ? 1 : -1;
        gyroSignVerified = true;
        preferences.putInt("gyroSign", gyroCompassSign);
        preferences.putBool("gyroSignOK", true);
        lastSystemEvent = "GYRO_SIGN_VERIFIED_" + String(static_cast<int>(gyroCompassSign));
      } else lastSystemEvent = "GYRO_SIGN_TEST_TOO_SHORT";
      gyroTurnLearning = false;
    }
    return;
  }
  manualDriveActive = true;
  lastManualCommandMs = millis();
  if (command == "l") beginGyroTurnLearning(-1);
  if (command == "r") beginGyroTurnLearning(+1);
  if (command == "f") writeMotors(1350, 1350);
  else if (command == "l") writeMotors(1430, 1320);
  else if (command == "r") writeMotors(1320, 1430);
  else if (command == "b") writeMotors(1750, 1750);
  else { manualDriveActive = false; stopMotors(); }
}

void updateManualDriveTimeout() {
  if (manualDriveActive && millis() - lastManualCommandMs > MANUAL_COMMAND_TIMEOUT_MS) {
    manualDriveActive = false;
    stopMotors();
    gyroTurnLearning = false;
    lastSystemEvent = "MANUAL_LINK_TIMEOUT";
  }
}

String controlPage() {
  return R"rawliteral(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>
html,body{overscroll-behavior:none;-webkit-touch-callout:none}
body{font-family:Arial;background:#0f1419;color:#fff;text-align:center;margin:0;padding:9px;-webkit-user-select:none;user-select:none}
.card{background:#1d2730;border-radius:14px;padding:12px;margin:9px auto;max-width:720px}
button,input,select{font-size:17px;padding:13px;margin:5px;border:0;border-radius:11px}
button{min-width:125px;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
button.hold{touch-action:none}.go{background:#23a566;color:#fff}.stop{background:#d62828;color:#fff}.manual{background:#74c0fc}.mode{background:#f0b429}.safe{background:#6c757d;color:#fff}
input{width:138px;-webkit-user-select:text;user-select:text}.mono{font-family:monospace;text-align:left;white-space:pre-wrap;font-size:13px}.small{font-size:13px;color:#cbd5e1}.ready{font-weight:700;font-size:20px;padding:10px;border-radius:10px;background:#6c2}.blocked{font-weight:700;font-size:18px;padding:10px;border-radius:10px;background:#a33}
</style></head><body>
<h2>Catamaran V5.1 Match Survey</h2>
<div class="card"><div id="readyBox" class="blocked">CHECKING...</div><div class="small" id="reason"></div><button class="safe" onclick="cmd('arm')">ARM</button><button class="safe" onclick="cmd('disarm')">DISARM</button><button class="stop" onclick="cmd('stop')">EMERGENCY STOP</button></div>
<div class="card"><h3>Manual control</h3><button class="manual hold" data-m="f">FORWARD</button><br><button class="manual hold" data-m="l">LEFT</button><button class="stop" onclick="manualStop()">STOP</button><button class="manual hold" data-m="r">RIGHT</button><br><button class="manual hold" data-m="b">REVERSE</button><p class="small">First setup only: hold LEFT or RIGHT for at least one second if gyro sign is not verified.</p></div>
<div class="card"><h3>Motor balance</h3><button class="mode" onclick="cmd('trimminus')">-5 us</button><button class="mode" onclick="cmd('trimreset')">RESET 20</button><button class="mode" onclick="cmd('trimplus')">+5 us</button></div>
<div class="card"><h3>1. Save four corners in order</h3><p class="small">Walk around the boundary: A → B → C → D. A-B is the first photo strip. Keep the GPS still for 6 seconds at each corner.</p><button class="mode" onclick="cmd('savea')">SAVE A HERE</button><button class="mode" onclick="cmd('saveb')">SAVE B HERE</button><button class="mode" onclick="cmd('savec')">SAVE C HERE</button><button class="mode" onclick="cmd('saved')">SAVE D HERE</button><br><button class="safe" onclick="cmd('cleararea')">CLEAR AREA</button></div>
<div class="card"><h3>Manual coordinate entry</h3><select id="corner"><option>A</option><option>B</option><option>C</option><option>D</option></select><br><input id="cornerLat" inputmode="decimal" placeholder="latitude"><input id="cornerLon" inputmode="decimal" placeholder="longitude"><br><button class="mode" onclick="setCorner()">SET SELECTED CORNER</button></div>
<div class="card"><h3>2. Coverage settings</h3><label>Strip spacing, m<br><input id="spacing" type="number" step="0.1" min="1.0" max="8.0" value="1.5"></label><br><label>Photo interval, ms<br><input id="photo" type="number" step="50" min="900" max="3000" value="1100"></label><br><button class="mode" onclick="saveConfig()">SAVE SETTINGS</button><button class="go" onclick="cmd('build')">BUILD COVERAGE ROUTE</button></div>
<div class="card"><h3>3. Heading and camera</h3><button class="mode" onclick="cmd('levelzero')">SET WATER LEVEL ZERO</button><br><p class="small">Place the boat near A and point the bow from A toward B.</p><button class="go" onclick="cmd('lockbow')">LOCK BOW (2 s)</button><button class="safe" onclick="cmd('clearbow')">CLEAR BOW LOCK</button><br><button class="mode" onclick="cmd('missionstart')">START PHOTO MISSION</button><button class="mode" onclick="cmd('capturetest')">TEST PHOTO</button></div>
<div class="card"><h3>4. Run</h3><button class="go" onclick="cmd('survey')">START FULL SURVEY</button><button class="mode" onclick="cmd('missionend')">END PHOTO MISSION</button></div>
<div class="card mono" id="status">Loading...</div>
<script>
let timer=null;
async function cmd(c){try{await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'})}catch(e){}setTimeout(update,180)}
async function sendManual(m){try{await fetch('/manual?m='+m,{cache:'no-store'})}catch(e){}}
function manualStart(m){manualStop(false);sendManual(m);timer=setInterval(()=>sendManual(m),200)}
function manualStop(send=true){if(timer){clearInterval(timer);timer=null}if(send)sendManual('0')}
document.querySelectorAll('.hold').forEach(b=>{b.addEventListener('pointerdown',e=>{e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(x){}manualStart(b.dataset.m)});b.addEventListener('pointerup',e=>{e.preventDefault();manualStop()});b.addEventListener('pointercancel',e=>{e.preventDefault();manualStop()});b.addEventListener('lostpointercapture',()=>manualStop());b.addEventListener('contextmenu',e=>e.preventDefault())});
document.addEventListener('selectstart',e=>{if(e.target.closest('button'))e.preventDefault()});window.addEventListener('blur',()=>manualStop());window.addEventListener('pagehide',()=>manualStop());
async function saveConfig(){const s=document.getElementById('spacing').value;const p=document.getElementById('photo').value;await fetch('/surveyconfig?spacing='+encodeURIComponent(s)+'&photo='+encodeURIComponent(p),{cache:'no-store'});update()}
async function setCorner(){const slot=document.getElementById('corner').value;const lat=document.getElementById('cornerLat').value;const lon=document.getElementById('cornerLon').value;await fetch('/setcorner?slot='+encodeURIComponent(slot)+'&lat='+encodeURIComponent(lat)+'&lon='+encodeURIComponent(lon),{cache:'no-store'});update()}
async function update(){try{const s=await(await fetch('/status',{cache:'no-store'})).json();document.getElementById('spacing').value=s.requestedSpacing;document.getElementById('photo').value=s.photoInterval;const box=document.getElementById('readyBox');box.textContent=s.ready==='YES'?'READY TO START':'NOT READY';box.className=s.ready==='YES'?'ready':'blocked';document.getElementById('reason').textContent=s.ready==='YES'?'All start checks passed':'Next action: '+s.blockReason;document.getElementById('status').textContent=
'GPS: '+s.gps+' SAT:'+s.sat+' HDOP:'+s.hdop+' SPEED:'+s.speed+' km/h\n'+
'CAM:'+s.camera+' READY:'+s.cameraReady+' SD:'+s.sd+' MISSION:'+s.mission+'\n'+
'MOTORS:'+s.motors+' L/R:'+s.left+'/'+s.right+' TRIM:'+s.rightTrim+' us\n'+
'GYRO SIGN:'+s.gyroSign+' VERIFIED:'+s.gyroSignVerified+' BOW:'+s.bowLock+' HEADING:'+s.heading+'\n'+
'A:'+s.aSet+' B:'+s.bSet+' C:'+s.cSet+' D:'+s.dSet+' ROUTE:'+s.routeBuilt+'\n'+
'AREA: '+s.length+' x '+s.width+' m ('+s.area+' m2) LANES:'+s.lanes+' SPACING:'+s.actualSpacing+' m\n'+
'ROUTE LENGTH:'+s.routeDistance+' m PHOTO INTERVAL:'+s.photoInterval+' ms\n'+
'SURVEY:'+s.auto+' '+s.autoMessage+'\n'+
'LEG:'+s.leg+'/'+s.legTotal+' TYPE:'+s.legType+' PROGRESS:'+s.progress+'% XTE:'+s.xte+' m\n'+
'TARGET DIST:'+s.distance+' m BEARING:'+s.bearing+' DESIRED:'+s.desired+' ERROR:'+s.headingError+'\n'+
'PHOTOS requested/saved/failed: '+s.photosRequested+'/'+s.photosSaved+'/'+s.photosFailed+' PENDING:'+s.capturePending+'\n'+
'ROLL/PITCH:'+s.roll+'/'+s.pitch+' GYRO Z:'+s.gyroZ+' LEVEL:'+s.level+'\n'+
'EVENT: '+s.event+'\nPOINT AVERAGING: '+s.pointAveraging+' ('+s.avgSamples+' samples)'}catch(e){document.getElementById('status').textContent='Connection lost'}}
setInterval(update,700);update();
</script></body></html>
)rawliteral";
}

String jsonNumber(double value, unsigned int decimals) {
  if (!isfinite(value)) return "null";
  return String(value, decimals);
}

void handleControlStatus() {
  updateTargetGeometry();
  String gpsState = !gpsHasData() ? "NO_DATA" : (!gpsHasFreshFix() ? "SEARCH" : "FIX");
  String missionState = missionActive ? ("M" + String(currentMissionNumber)) : "NONE";
  String bowLockState = bowLockPending ? "PENDING" : (headingInitialized ? "LOCKED" : "NO");
  String avgState = surveyPointAveragingSlot == SurveyPointSlot::NONE ? "NO" :
                    surveyPointAveragingSlot == SurveyPointSlot::A ? "A" :
                    surveyPointAveragingSlot == SurveyPointSlot::B ? "B" :
                    surveyPointAveragingSlot == SurveyPointSlot::C ? "C" : "D";
  String blockReason = surveyStartBlockReason();
  String readyState = blockReason.length() == 0 ? "YES" : "NO";
  String legType = activeLegIndex < 0 ? "APPROACH" : activeLegIsSurvey ? "PHOTO_STRIP" : "CONNECTOR";
  String json = "{";
  json += "\"ready\":\"" + readyState + "\",\"blockReason\":\"" + blockReason + "\",";
  json += "\"gps\":\"" + gpsState + "\",";
  json += "\"sat\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  json += "\"hdop\":" + jsonNumber(gps.hdop.isValid() ? gps.hdop.hdop() : 99.99, 2) + ",";
  json += "\"speed\":" + jsonNumber(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2) + ",";
  json += "\"camera\":\"" + String(cameraOnline ? "ONLINE" : "OFFLINE") + "\",";
  json += "\"cameraReady\":\"" + String(cameraRemoteReady ? "YES" : "NO") + "\",";
  json += "\"sd\":\"" + String(cameraSdReady ? "READY" : "NOT_READY") + "\",";
  json += "\"mission\":\"" + missionState + "\",";
  json += "\"motors\":\"" + String(motorsArmed ? "ARMED" : "SAFE") + "\",";
  json += "\"left\":" + String(currentLeftUs) + ",\"right\":" + String(currentRightUs) + ",\"rightTrim\":" + String(rightForwardTrimUs) + ",";
  json += "\"gyroSign\":" + String(static_cast<int>(gyroCompassSign)) + ",\"gyroSignVerified\":\"" + String(gyroSignVerified ? "YES" : "NO") + "\",";
  json += "\"bowLock\":\"" + bowLockState + "\",\"heading\":" + jsonNumber(headingInitialized ? headingEstimateDeg : 0.0, 1) + ",";
  json += "\"aSet\":\"" + String(surveyASet ? "YES" : "NO") + "\",\"bSet\":\"" + String(surveyBSet ? "YES" : "NO") + "\",\"cSet\":\"" + String(surveyCSet ? "YES" : "NO") + "\",\"dSet\":\"" + String(surveyDSet ? "YES" : "NO") + "\",";
  json += "\"routeBuilt\":\"" + String(surveyRouteBuilt ? "YES" : "NO") + "\",\"length\":" + jsonNumber(surveyLengthM, 1) + ",\"width\":" + jsonNumber(surveyWidthM, 1) + ",\"area\":" + jsonNumber(surveyAreaM2, 1) + ",\"routeDistance\":" + jsonNumber(surveyTotalRouteM, 1) + ",";
  json += "\"lanes\":" + String(surveyLaneCount) + ",\"requestedSpacing\":" + jsonNumber(surveyRequestedSpacingM, 1) + ",\"actualSpacing\":" + jsonNumber(surveyActualSpacingM, 2) + ",\"photoInterval\":" + String(surveyPhotoIntervalMs) + ",";
  json += "\"auto\":\"" + surveyStateName() + "\",\"autoMessage\":\"" + autoMessage + "\",";
  json += "\"leg\":" + String(activeLegIndex < 0 ? 0 : activeLegIndex + 1) + ",\"legTotal\":" + String(max(0, static_cast<int>(surveyRoutePointCount) - 1)) + ",\"legType\":\"" + legType + "\",";
  json += "\"progress\":" + jsonNumber(activeLegProgressPercent, 1) + ",\"xte\":" + jsonNumber(activeLegCrossTrackM, 2) + ",";
  json += "\"distance\":" + jsonNumber(currentTargetDistanceM, 1) + ",\"bearing\":" + jsonNumber(currentTargetBearingDeg, 1) + ",\"desired\":" + jsonNumber(desiredTrackBearingDeg, 1) + ",\"headingError\":" + jsonNumber(currentHeadingErrorDeg, 1) + ",";
  json += "\"photosRequested\":" + String(surveyPhotosRequested) + ",\"photosSaved\":" + String(surveyPhotosSaved) + ",\"photosFailed\":" + String(surveyPhotosFailed) + ",\"capturePending\":" + String(captureRequestPending ? "true" : "false") + ",";
  json += "\"roll\":" + jsonNumber(relativeRollDegrees, 1) + ",\"pitch\":" + jsonNumber(relativePitchDegrees, 1) + ",\"gyroZ\":" + jsonNumber(currentGyroZ, 1) + ",\"level\":" + String(levelEnough ? "true" : "false") + ",";
  json += "\"event\":\"" + lastSystemEvent + "\",\"pointAveraging\":\"" + avgState + "\",\"avgSamples\":" + String(surveyPointAverageSamples);
  json += "}";
  controlServer.send(200, "application/json", json);
}

void handleControlCommand() {
  String command = controlServer.arg("c");
  command.toLowerCase();
  if (command == "arm") armMotors();
  else if (command == "disarm") disarmMotors();
  else if (command == "stop") { motorTestStopAtMs = 0; stopAutonomy("WEB_EMERGENCY_STOP", false); }
  else if (command == "trimminus") changeRightMotorTrim(-5);
  else if (command == "trimplus") changeRightMotorTrim(5);
  else if (command == "trimreset") { rightForwardTrimUs = RIGHT_FORWARD_TRIM_DEFAULT_US; saveMotorTrimToNvs(); lastSystemEvent = "RIGHT_TRIM_RESET"; }
  else if (command == "levelzero") setCurrentWaterLevelZero();
  else if (command == "savea") startSurveyPointAveraging(SurveyPointSlot::A);
  else if (command == "saveb") startSurveyPointAveraging(SurveyPointSlot::B);
  else if (command == "savec") startSurveyPointAveraging(SurveyPointSlot::C);
  else if (command == "saved") startSurveyPointAveraging(SurveyPointSlot::D);
  else if (command == "cleararea") clearSurveyArea();
  else if (command == "build") buildSurveyRoute();
  else if (command == "lockbow") startBowLockCalibration();
  else if (command == "clearbow") clearBowLock();
  else if (command == "gyrosignreset") requestGyroSignRelearn();
  else if (command == "missionstart") requestMissionStart();
  else if (command == "capturetest") { pendingCaptureIsSurvey = false; requestCapture(true); }
  else if (command == "missionend") requestMissionEnd();
  else if (command == "survey") startSurvey();
  controlServer.send(200, "text/plain", "OK");
}

void handleSurveyConfig() {
  float spacing = controlServer.arg("spacing").toFloat();
  uint32_t photo = static_cast<uint32_t>(controlServer.arg("photo").toInt());
  configureSurvey(spacing, photo);
  controlServer.send(200, "text/plain", "OK");
}

SurveyPointSlot surveySlotFromString(String slot) {
  slot.trim();
  slot.toUpperCase();
  if (slot == "A") return SurveyPointSlot::A;
  if (slot == "B") return SurveyPointSlot::B;
  if (slot == "C") return SurveyPointSlot::C;
  if (slot == "D") return SurveyPointSlot::D;
  return SurveyPointSlot::NONE;
}

void handleSetCorner() {
  SurveyPointSlot slot = surveySlotFromString(controlServer.arg("slot"));
  double latitude = controlServer.arg("lat").toDouble();
  double longitude = controlServer.arg("lon").toDouble();
  if (!setSurveyCorner(slot, latitude, longitude)) {
    controlServer.send(400, "text/plain", "INVALID CORNER OR COORDINATES");
    return;
  }
  controlServer.send(200, "text/plain", "OK");
}

void startControlWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONTROL_AP_NAME, CONTROL_AP_PASS, 6, false, 4);
  controlServer.on("/", []() { controlServer.send(200, "text/html", controlPage()); });
  controlServer.on("/status", handleControlStatus);
  controlServer.on("/cmd", handleControlCommand);
  controlServer.on("/surveyconfig", handleSurveyConfig);
  controlServer.on("/setcorner", handleSetCorner);
  controlServer.on("/manual", []() { manualMotorCommand(controlServer.arg("m")); controlServer.send(200, "text/plain", "OK"); });
  controlServer.begin();
  Serial.print("CONTROL WIFI: "); Serial.println(CONTROL_AP_NAME);
  Serial.print("CONTROL URL: http://"); Serial.println(WiFi.softAPIP());
}

void printSystemState() {
  uint32_t nowMs = millis();
  if (nowMs - previousPrintMs < PRINT_INTERVAL_MS) return;
  previousPrintMs = nowMs;
  Serial.print("GPS:"); Serial.print(gpsHasFreshFix() ? "FIX" : "SEARCH");
  Serial.print(" SAT:"); Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
  Serial.print(" | STATE:"); Serial.print(surveyStateName());
  Serial.print(" LEG:"); Serial.print(activeLegIndex + 1); Serial.print("/"); Serial.print(max(0, static_cast<int>(surveyRoutePointCount) - 1));
  Serial.print(" | H:"); Serial.print(headingInitialized ? headingEstimateDeg : -1.0f, 1);
  Serial.print(" DES:"); Serial.print(desiredTrackBearingDeg, 1);
  Serial.print(" ERR:"); Serial.print(currentHeadingErrorDeg, 1);
  Serial.print(" | XTE:"); Serial.print(activeLegCrossTrackM, 2);
  Serial.print(" | L/R:"); Serial.print(currentLeftUs); Serial.print("/"); Serial.print(currentRightUs);
  Serial.print(" | P:"); Serial.print(surveyPhotosSaved); Serial.print("/"); Serial.print(surveyPhotosRequested);
  Serial.print(" | EVENT:"); Serial.println(lastSystemEvent);
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("======================================");
  Serial.println("CATAMARAN MAIN ESP32");
  Serial.println("AUTONOMY V5.1.1: FOUR-CORNER MATCH SURVEY + GYRO HEADING + CONTINUOUS PHOTOS");
  Serial.println("======================================");

  bootStartedMs = millis();
  preferences.begin("catamaran", false);
  loadMotorTrimFromNvs();
  loadGyroSignFromNvs();
  loadSurveyConfigFromNvs();

  Serial.print("Loaded right forward trim: ");
  Serial.print(rightForwardTrimUs);
  Serial.println(" us");
  Serial.print("Loaded gyro compass sign: ");
  Serial.print(static_cast<int>(gyroCompassSign));
  Serial.print(" | verified: ");
  Serial.println(gyroSignVerified ? "YES" : "NO");

  escLeft.setPeriodHertz(50);
  escRight.setPeriodHertz(50);
  escLeft.attach(ESC_LEFT_PIN, 900, 2100);
  escRight.attach(ESC_RIGHT_PIN, 900, 2100);
  stopMotors();

  cameraInputLine.reserve(720);
  usbInputLine.reserve(128);

  CameraSerial.begin(
    CAMERA_UART_BAUD,
    SERIAL_8N1,
    CAMERA_UART_RX_PIN,
    CAMERA_UART_TX_PIN
  );


  GpsSerial.begin(
    GPS_UART_BAUD,
    SERIAL_8N1,
    GPS_UART_RX_PIN,
    GPS_UART_TX_PIN
  );

  Serial.print("Camera UART RX: GPIO");
  Serial.println(CAMERA_UART_RX_PIN);

  Serial.print("Camera UART TX: GPIO");
  Serial.println(CAMERA_UART_TX_PIN);

  Serial.print("GPS UART RX: GPIO");
  Serial.println(GPS_UART_RX_PIN);

  Serial.print("GPS UART TX: GPIO");
  Serial.println(GPS_UART_TX_PIN);

  Serial.print("Left ESC: GPIO");
  Serial.println(ESC_LEFT_PIN);
  Serial.print("Right ESC: GPIO");
  Serial.println(ESC_RIGHT_PIN);
  Serial.println("ESC output held at 1500 us during boot");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  if (!initializeMPU()) {
    Serial.println("MPU initialization failed");

    while (true) {
      updateCameraUart();
      delay(10);
    }
  }

  Serial.println("MPU initialization OK");
  Serial.println("Sensor warm-up...");
  delay(2000);

  if (!calibrateRobotPosition()) {
    Serial.println("Initial calibration failed");

    while (true) {
      updateCameraUart();
      delay(10);
    }
  }

  startControlWebServer();
  printHelp();
  sendPing();
  delay(100);
  requestMissionStatus();
}

// ============================================================
// Loop
// ============================================================

void loop() {
  updateMPU();
  updateGPS();
  updateBowLockCalibration();
  updateCameraUart();
  checkUsbSerialCommands();
  controlServer.handleClient();
  updateSurveyPointAveraging();
  updateManualDriveTimeout();
  updateAutopilot();
  printSystemState();

  delay(1);
}
