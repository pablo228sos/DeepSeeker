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

struct DubinsPlan {
  char mode[3];
  float parameter[3];  // normalized: arc radians or straight length / radius
  float cost;
  bool valid;
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
uint32_t lastCameraStatusRequestMs = 0;
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
constexpr uint32_t CAMERA_STATUS_INTERVAL_MS = 4000;
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
  lastCameraStatusRequestMs = millis();
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
    // The main ESP32 can miss CAMERA_BOOT when both boards power up together.
    // In that case ONLINE becomes true, but CAMERA/SD readiness remains unknown.
    // Ask for a fresh status immediately instead of keeping false values forever.
    if (millis() - lastCameraStatusRequestMs > 1000) {
      requestCameraStatus();
      requestMissionStatus();
    }
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
    lastSystemEvent = String("CAM_STATUS_C=") + (cameraRemoteReady ? "1" : "0") +
                      "_SD=" + (cameraSdReady ? "1" : "0");
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
    cameraRemoteReady = false;
    cameraSdReady = false;
    Serial.println("CAMERA LINK: OFFLINE");
  }

  if (
    nowMs - lastPingMs >=
      CAMERA_PING_INTERVAL_MS
  ) {
    sendPing();
  }

  // Refresh camera, SD and mission state periodically. This also detects an SD
  // card that becomes ready after the camera firmware retries initialization.
  if (
    nowMs - lastCameraStatusRequestMs >=
      CAMERA_STATUS_INTERVAL_MS
  ) {
    requestCameraStatus();
    requestMissionStatus();
  }
}

// ============================================================
// V6.0 INTERLACED TEARDROP SURVEY
//
// The route is built specifically for the available sensors:
//   - GPS supplies position and course while the boat is moving.
//   - MPU-6050 supplies fast relative yaw and yaw rate.
//   - no magnetic compass is required after the initial LOCK BOW.
//
// Coverage order:
//   lane 0, 2, 4 ... then lane 1, 3, 5 ...
// Consecutive visited lanes therefore normally have twice the requested photo
// spacing, which gives the catamaran room for a forward-only U/teardrop turn.
// The boat follows one continuous polyline: photo strip -> outside Bezier turn
// -> next photo strip. It does not stop at strip endpoints.
// ============================================================

enum class SurveyState : uint8_t {
  IDLE,
  FOLLOW_PATH,
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
constexpr uint16_t MAX_SURVEY_PATH_NODES = 1100;

LocalPoint surveyLaneLeft[MAX_SURVEY_LANES];
LocalPoint surveyLaneRight[MAX_SURVEY_LANES];
uint8_t surveyVisitOrder[MAX_SURVEY_LANES];
uint8_t surveyFirstPassCount = 0;

// Segment i runs from node i to node i+1.
LocalPoint surveyPathNodes[MAX_SURVEY_PATH_NODES];
float surveyPathCumulativeM[MAX_SURVEY_PATH_NODES];
bool surveyPathSegmentPhoto[MAX_SURVEY_PATH_NODES - 1];
int8_t surveyPathSegmentLane[MAX_SURVEY_PATH_NODES - 1];
uint8_t surveyPathSegmentVisit[MAX_SURVEY_PATH_NODES - 1];
uint16_t surveyPathNodeCount = 0;

uint8_t surveyLaneCount = 0;
float surveyRequestedSpacingM = 1.5f;
float surveyActualSpacingM = 0.0f;
float surveyTurnRadiusM = 2.0f;
float surveyRequiredHeadlandM = 0.0f;
float surveyLengthM = 0.0f;
float surveyWidthM = 0.0f;
float surveyAreaM2 = 0.0f;
float surveyTotalRouteM = 0.0f;
bool surveyRouteBuilt = false;

uint16_t activePathSegment = 0;
int16_t activeLegIndex = -1;
bool activeLegIsSurvey = false;
int8_t activeLaneIndex = -1;
uint8_t activeVisitIndex = 0;
float activeLegLengthM = 0.0f;
float activeLegAlongM = 0.0f;
float activeLegCrossTrackM = 0.0f;
float activeLegProgressPercent = 0.0f;
float surveyPathProgressM = 0.0f;
float surveyPathProgressPercent = 0.0f;
float surveyPathDistanceM = 0.0f;
float activeSegmentProjection = 0.0f;
uint32_t activeLegStartedMs = 0;
uint32_t surveyLegStartedMs = 0;
uint32_t lastProcessedPathGpsFixMs = 0;
uint32_t lastMeaningfulProgressMs = 0;
float meaningfulProgressCheckpointM = 0.0f;
bool pathFinished = false;
bool finalPhotoRequestedForLane = false;

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
float estimatedSurveySpeedMps = 0.35f;

uint32_t surveyPhotoIntervalMs = 1100;
uint32_t lastSurveyPhotoRequestMs = 0;

constexpr float EARTH_RADIUS_M = 6371000.0f;
constexpr uint32_t AUTO_UPDATE_INTERVAL_MS = 100;
constexpr uint32_t MPU_STALE_TIMEOUT_MS = 500;
constexpr uint32_t AUTO_MAX_RUN_TIME_MS = 35UL * 60UL * 1000UL;
constexpr float AUTO_MAX_DISTANCE_FROM_A_M = 150.0f;
constexpr float AUTO_MAX_ROLL_DEG = 30.0f;
constexpr float AUTO_MAX_PITCH_DEG = 30.0f;
constexpr uint32_t AUTO_CAPTURE_TIMEOUT_MS = 12000;

constexpr float SURVEY_MIN_LENGTH_M = 5.0f;
constexpr float SURVEY_MAX_LENGTH_M = 90.0f;
constexpr float SURVEY_MIN_WIDTH_M = 2.5f;
constexpr float SURVEY_MAX_WIDTH_M = 60.0f;
constexpr float SURVEY_MIN_SPACING_M = 1.0f;
constexpr float SURVEY_MAX_SPACING_M = 8.0f;
constexpr float SURVEY_MIN_TURN_RADIUS_M = 1.4f;
constexpr float SURVEY_MAX_TURN_RADIUS_M = 4.0f;
constexpr uint32_t SURVEY_MIN_PHOTO_INTERVAL_MS = 900;
constexpr uint32_t SURVEY_MAX_PHOTO_INTERVAL_MS = 3000;
constexpr float SURVEY_START_NEAR_M = 5.0f;

// The line continues outside the photo rectangle before and after each turn.
// This prevents pure pursuit from cutting the end of the photographed strip.
constexpr float PATH_FINAL_EXTENSION_M = 1.0f;
constexpr float PATH_NODE_TARGET_SPACING_M = 1.15f;

constexpr float PATH_LOOKAHEAD_PHOTO_M = 3.2f;
constexpr float PATH_LOOKAHEAD_TURN_M = 2.3f;
constexpr float PATH_ADVANCE_END_RADIUS_M = 1.6f;
constexpr float PATH_FINAL_RADIUS_M = 2.0f;
constexpr uint8_t PATH_FORWARD_SEARCH_SEGMENTS = 18;
constexpr float PATH_MAX_PROGRESS_JUMP_M = 6.0f;
constexpr float PATH_BACKTRACK_TOLERANCE_M = 0.7f;
constexpr float PATH_MAX_DEVIATION_M = 12.0f;
constexpr uint32_t PATH_STALL_TIMEOUT_MS = 45000;
constexpr uint32_t PATH_STALL_GRACE_MS = 12000;

constexpr float SURVEY_HEADING_DEADBAND_DEG = 4.0f;
constexpr float SURVEY_HEADING_KP_US_PER_DEG = 0.54f;
constexpr float SURVEY_YAW_DAMPING_US_PER_DPS = 0.82f;
constexpr float TURN_HEADING_KP_US_PER_DEG = 0.43f;
constexpr float TURN_YAW_DAMPING_US_PER_DPS = 0.66f;
constexpr float SURVEY_MAX_STEERING_US = 34.0f;
constexpr float TURN_MAX_STEERING_US = 44.0f;
constexpr float SURVEY_STEERING_SLEW_US = 4.0f;
constexpr float TURN_STEERING_SLEW_US = 6.0f;
constexpr int SURVEY_FORWARD_LIMIT_US = 1210;
constexpr int SURVEY_WEAKEST_FORWARD_US = 1460;
constexpr int SURVEY_BASE_US = 1360;
constexpr int TURN_BASE_US = 1405;
constexpr int TURN_SLOW_BASE_US = 1435;
constexpr uint32_t BOW_LOCK_MAX_START_AGE_MS = 5UL * 60UL * 1000UL;

constexpr float DEFAULT_SURVEY_SPEED_MPS = 0.35f;
constexpr float MIN_VALID_SPEED_MPS = 0.12f;
constexpr float MAX_VALID_SPEED_MPS = 1.20f;

// GPS course is not used as direct steering. It slowly anchors gyro yaw while
// the catamaran moves. The turn limits are intentionally looser because both
// propellers remain forward and the course remains observable throughout the arc.
constexpr float HEADING_GPS_MIN_SPEED_KMPH = 0.75f;
constexpr uint32_t HEADING_GPS_CORRECTION_INTERVAL_MS = 700;
constexpr float HEADING_GPS_STRAIGHT_MAX_YAW_DPS = 8.0f;
constexpr float HEADING_GPS_TURN_MAX_YAW_DPS = 28.0f;
constexpr float HEADING_GPS_STRAIGHT_MAX_DELTA_DEG = 32.0f;
constexpr float HEADING_GPS_TURN_MAX_DELTA_DEG = 58.0f;
constexpr float HEADING_GPS_STRAIGHT_GAIN = 0.050f;
constexpr float HEADING_GPS_TURN_GAIN = 0.018f;
constexpr float HEADING_GPS_EXIT_GAIN = 0.090f;
constexpr float HEADING_GPS_STRAIGHT_MAX_STEP_DEG = 1.0f;
constexpr float HEADING_GPS_TURN_MAX_STEP_DEG = 0.7f;
constexpr float HEADING_GPS_EXIT_MAX_STEP_DEG = 1.4f;
constexpr uint32_t HEADING_GPS_EXIT_RELOCK_MS = 5000;

constexpr uint32_t BOW_LOCK_DURATION_MS = 1800;
constexpr uint32_t BOW_LOCK_SAMPLE_INTERVAL_MS = 20;
constexpr float BOW_LOCK_MAX_AVERAGE_GYRO_DPS = 2.5f;
constexpr float BOW_LOCK_MAX_AVERAGE_ABS_GYRO_DPS = 5.0f;

String surveyStateName() {
  switch (surveyState) {
    case SurveyState::IDLE: return "IDLE";
    case SurveyState::FOLLOW_PATH: return "FOLLOW_PATH";
    case SurveyState::FINISH_WAIT: return "FINISH_WAIT";
    case SurveyState::COMPLETE: return "COMPLETE";
    case SurveyState::FAILSAFE: return "FAILSAFE";
  }
  return "UNKNOWN";
}

bool surveyRunning() {
  return surveyState == SurveyState::FOLLOW_PATH ||
         surveyState == SurveyState::FINISH_WAIT;
}

String surveyPhaseName() {
  if (!surveyRunning() || activeLaneIndex < 0) return "READY";
  return activeVisitIndex < surveyFirstPassCount ? "PASS_1_EVEN_LANES" : "PASS_2_ODD_LANES";
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

float localDistance(const LocalPoint& a, const LocalPoint& b) {
  float dx = b.x - a.x;
  float dy = b.y - a.y;
  return sqrtf(dx * dx + dy * dy);
}

LocalPoint localAdd(const LocalPoint& a, const LocalPoint& b) {
  return {a.x + b.x, a.y + b.y};
}

LocalPoint localSub(const LocalPoint& a, const LocalPoint& b) {
  return {a.x - b.x, a.y - b.y};
}

LocalPoint localScale(const LocalPoint& a, float scale) {
  return {a.x * scale, a.y * scale};
}

LocalPoint localUnit(const LocalPoint& vector) {
  float length = max(0.001f, localLength(vector));
  return {vector.x / length, vector.y / length};
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

float cross2D(const LocalPoint& a, const LocalPoint& b, const LocalPoint& c) {
  return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

LocalPoint interpolateLocal(const LocalPoint& a, const LocalPoint& b, float t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
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
      fabsf(z3) < MIN_CROSS || fabsf(z4) < MIN_CROSS) return false;
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
  lastSystemEvent = expectedDirection > 0 ? "LEARNING_GYRO_SIGN_RIGHT" : "LEARNING_GYRO_SIGN_LEFT";
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
  preferences.putFloat("turnRadius", surveyTurnRadiusM);
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
  surveyTurnRadiusM = constrain(preferences.getFloat("turnRadius", 2.0f), SURVEY_MIN_TURN_RADIUS_M, SURVEY_MAX_TURN_RADIUS_M);
  surveyPhotoIntervalMs = constrain(preferences.getUInt("surveyPhoto", 1100), SURVEY_MIN_PHOTO_INTERVAL_MS, SURVEY_MAX_PHOTO_INTERVAL_MS);
}

void clearSurveyArea() {
  surveyASet = surveyBSet = surveyCSet = surveyDSet = false;
  surveyRouteBuilt = false;
  surveyPathNodeCount = 0;
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
  if (rightUs < ESC_NEUTRAL_US) {
    rightUs += rightForwardTrimUs;
    rightUs = constrain(rightUs, 1000, AUTO_NEUTRAL_LIMIT_US);
  }
  if (leftUs < ESC_NEUTRAL_US) leftUs = constrain(leftUs, 1000, AUTO_NEUTRAL_LIMIT_US);
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
  activePathSegment = 0;
  activeLegIndex = -1;
  pathFinished = false;
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

bool appendSurveyPathPoint(const LocalPoint& point, bool photoSegment, int8_t laneIndex, uint8_t visitIndex) {
  if (surveyPathNodeCount == 0) {
    surveyPathNodes[0] = point;
    surveyPathCumulativeM[0] = 0.0f;
    surveyPathNodeCount = 1;
    return true;
  }
  if (surveyPathNodeCount >= MAX_SURVEY_PATH_NODES) return false;
  uint16_t previous = surveyPathNodeCount - 1;
  float distance = localDistance(surveyPathNodes[previous], point);
  if (distance < 0.03f) return true;
  surveyPathSegmentPhoto[previous] = photoSegment;
  surveyPathSegmentLane[previous] = laneIndex;
  surveyPathSegmentVisit[previous] = visitIndex;
  surveyPathNodes[surveyPathNodeCount] = point;
  surveyPathCumulativeM[surveyPathNodeCount] = surveyPathCumulativeM[previous] + distance;
  surveyPathNodeCount++;
  return true;
}

float positiveModuloTwoPi(float angle) {
  while (angle < 0.0f) angle += 2.0f * PI;
  while (angle >= 2.0f * PI) angle -= 2.0f * PI;
  return angle;
}

void storeDubinsCandidate(
  DubinsPlan& candidate,
  char mode0,
  char mode1,
  char mode2,
  float first,
  float second,
  float third
) {
  candidate.mode[0] = mode0;
  candidate.mode[1] = mode1;
  candidate.mode[2] = mode2;
  candidate.parameter[0] = first;
  candidate.parameter[1] = second;
  candidate.parameter[2] = third;
  candidate.cost = first + second + third;
  candidate.valid = isfinite(candidate.cost) && first >= 0.0f && second >= 0.0f && third >= 0.0f;
}

uint8_t createDubinsCandidates(
  const LocalPoint& start,
  float startYaw,
  const LocalPoint& finish,
  float finishYaw,
  float radius,
  DubinsPlan candidates[6]
) {
  for (uint8_t i = 0; i < 6; i++) candidates[i].valid = false;

  float dx = (finish.x - start.x) / radius;
  float dy = (finish.y - start.y) / radius;
  float d = sqrtf(dx * dx + dy * dy);
  float theta = positiveModuloTwoPi(atan2f(dy, dx));
  float alpha = positiveModuloTwoPi(startYaw - theta);
  float beta = positiveModuloTwoPi(finishYaw - theta);
  float sinAlpha = sinf(alpha);
  float sinBeta = sinf(beta);
  float cosAlpha = cosf(alpha);
  float cosBeta = cosf(beta);
  float cosAlphaMinusBeta = cosf(alpha - beta);
  uint8_t count = 0;

  float pSquared = 2.0f + d * d - 2.0f * cosAlphaMinusBeta + 2.0f * d * (sinAlpha - sinBeta);
  if (pSquared >= -0.0001f) {
    pSquared = max(0.0f, pSquared);
    float temporary = atan2f(cosBeta - cosAlpha, d + sinAlpha - sinBeta);
    storeDubinsCandidate(candidates[count++], 'L', 'S', 'L',
      positiveModuloTwoPi(-alpha + temporary),
      sqrtf(pSquared),
      positiveModuloTwoPi(beta - temporary));
  }

  pSquared = 2.0f + d * d - 2.0f * cosAlphaMinusBeta + 2.0f * d * (-sinAlpha + sinBeta);
  if (pSquared >= -0.0001f) {
    pSquared = max(0.0f, pSquared);
    float temporary = atan2f(cosAlpha - cosBeta, d - sinAlpha + sinBeta);
    storeDubinsCandidate(candidates[count++], 'R', 'S', 'R',
      positiveModuloTwoPi(alpha - temporary),
      sqrtf(pSquared),
      positiveModuloTwoPi(-beta + temporary));
  }

  pSquared = -2.0f + d * d + 2.0f * cosAlphaMinusBeta + 2.0f * d * (sinAlpha + sinBeta);
  if (pSquared >= -0.0001f) {
    pSquared = max(0.0f, pSquared);
    float p = sqrtf(pSquared);
    float temporary = atan2f(-cosAlpha - cosBeta, d + sinAlpha + sinBeta) - atan2f(-2.0f, p);
    storeDubinsCandidate(candidates[count++], 'L', 'S', 'R',
      positiveModuloTwoPi(-alpha + temporary),
      p,
      positiveModuloTwoPi(-beta + temporary));
  }

  pSquared = d * d - 2.0f + 2.0f * cosAlphaMinusBeta - 2.0f * d * (sinAlpha + sinBeta);
  if (pSquared >= -0.0001f) {
    pSquared = max(0.0f, pSquared);
    float p = sqrtf(pSquared);
    float temporary = atan2f(cosAlpha + cosBeta, d - sinAlpha - sinBeta) - atan2f(2.0f, p);
    storeDubinsCandidate(candidates[count++], 'R', 'S', 'L',
      positiveModuloTwoPi(alpha - temporary),
      p,
      positiveModuloTwoPi(beta - temporary));
  }

  float acosArgument = (6.0f - d * d + 2.0f * cosAlphaMinusBeta + 2.0f * d * (sinAlpha - sinBeta)) / 8.0f;
  if (fabsf(acosArgument) <= 1.0001f) {
    acosArgument = constrain(acosArgument, -1.0f, 1.0f);
    float p = positiveModuloTwoPi(2.0f * PI - acosf(acosArgument));
    float t = positiveModuloTwoPi(alpha - atan2f(cosAlpha - cosBeta, d - sinAlpha + sinBeta) + p * 0.5f);
    float q = positiveModuloTwoPi(alpha - beta - t + p);
    storeDubinsCandidate(candidates[count++], 'R', 'L', 'R', t, p, q);
  }

  acosArgument = (6.0f - d * d + 2.0f * cosAlphaMinusBeta + 2.0f * d * (-sinAlpha + sinBeta)) / 8.0f;
  if (fabsf(acosArgument) <= 1.0001f) {
    acosArgument = constrain(acosArgument, -1.0f, 1.0f);
    float p = positiveModuloTwoPi(2.0f * PI - acosf(acosArgument));
    float t = positiveModuloTwoPi(-alpha - atan2f(cosAlpha - cosBeta, d + sinAlpha - sinBeta) + p * 0.5f);
    float q = positiveModuloTwoPi(beta - alpha - t + p);
    storeDubinsCandidate(candidates[count++], 'L', 'R', 'L', t, p, q);
  }

  return count;
}

void advanceDubinsPose(LocalPoint& position, float& yaw, char mode, float distanceM, float radius) {
  if (mode == 'S') {
    position.x += cosf(yaw) * distanceM;
    position.y += sinf(yaw) * distanceM;
    return;
  }

  float direction = mode == 'L' ? 1.0f : -1.0f;
  float centerX = position.x - direction * radius * sinf(yaw);
  float centerY = position.y + direction * radius * cosf(yaw);
  float nextYaw = yaw + direction * distanceM / radius;
  position.x = centerX + direction * radius * sinf(nextYaw);
  position.y = centerY - direction * radius * cosf(nextYaw);
  yaw = nextYaw;
}

bool dubinsCandidateStaysOutside(
  const LocalPoint& start,
  float startYaw,
  float radius,
  const DubinsPlan& plan,
  const LocalPoint& boundaryMiddle,
  const LocalPoint& outwardDirection,
  float& maximumHeadlandM
) {
  if (!plan.valid) return false;
  LocalPoint position = start;
  float yaw = startYaw;
  maximumHeadlandM = 0.0f;

  for (uint8_t part = 0; part < 3; part++) {
    float remaining = plan.parameter[part] * radius;
    while (remaining > 0.0001f) {
      float step = min(0.30f, remaining);
      advanceDubinsPose(position, yaw, plan.mode[part], step, radius);
      remaining -= step;
      float outward = (position.x - boundaryMiddle.x) * outwardDirection.x +
                      (position.y - boundaryMiddle.y) * outwardDirection.y;
      if (outward < -0.35f) return false;
      maximumHeadlandM = max(maximumHeadlandM, outward);
    }
  }
  return true;
}

bool appendDubinsPlan(
  const LocalPoint& start,
  float startYaw,
  const LocalPoint& exactFinish,
  float radius,
  const DubinsPlan& plan,
  int8_t nextLane,
  uint8_t nextVisit
) {
  LocalPoint position = start;
  float yaw = startYaw;
  for (uint8_t part = 0; part < 3; part++) {
    float remaining = plan.parameter[part] * radius;
    while (remaining > 0.0001f) {
      float step = min(PATH_NODE_TARGET_SPACING_M, remaining);
      advanceDubinsPose(position, yaw, plan.mode[part], step, radius);
      remaining -= step;
      if (!appendSurveyPathPoint(position, false, nextLane, nextVisit)) return false;
    }
  }
  return appendSurveyPathPoint(exactFinish, false, nextLane, nextVisit);
}

bool appendTeardropTurn(
  const LocalPoint& currentEnd,
  const LocalPoint& currentDirection,
  const LocalPoint& nextStart,
  const LocalPoint& nextDirection,
  int8_t nextLane,
  uint8_t nextVisit
) {
  float radius = surveyTurnRadiusM;
  LocalPoint outwardDirection = localUnit(localSub(currentDirection, nextDirection));
  if (currentDirection.x * outwardDirection.x + currentDirection.y * outwardDirection.y < 0.0f) {
    outwardDirection = localScale(outwardDirection, -1.0f);
  }
  LocalPoint boundaryMiddle = localScale(localAdd(currentEnd, nextStart), 0.5f);

  // Try progressively longer straight headland extensions. The Dubins solver
  // then chooses the shortest forward-only path whose curvature never exceeds
  // 1 / radius and which remains outside the photographed rectangle.
  float extension = max(2.5f, radius * 1.7f);
  for (uint8_t attempt = 0; attempt < 4; attempt++) {
    LocalPoint exitPoint = localAdd(currentEnd, localScale(currentDirection, extension));
    LocalPoint entryPoint = localSub(nextStart, localScale(nextDirection, extension));
    float startYaw = atan2f(currentDirection.y, currentDirection.x);
    float finishYaw = atan2f(nextDirection.y, nextDirection.x);

    DubinsPlan candidates[6];
    uint8_t candidateCount = createDubinsCandidates(exitPoint, startYaw, entryPoint, finishYaw, radius, candidates);
    int8_t bestIndex = -1;
    float bestCost = 99999.0f;
    float bestHeadland = 0.0f;

    for (uint8_t candidateIndex = 0; candidateIndex < candidateCount; candidateIndex++) {
      if (!candidates[candidateIndex].valid) continue;
      float candidateHeadland = 0.0f;
      if (!dubinsCandidateStaysOutside(exitPoint, startYaw, radius, candidates[candidateIndex], boundaryMiddle, outwardDirection, candidateHeadland)) continue;
      if (candidates[candidateIndex].cost < bestCost) {
        bestCost = candidates[candidateIndex].cost;
        bestIndex = static_cast<int8_t>(candidateIndex);
        bestHeadland = candidateHeadland;
      }
    }

    if (bestIndex >= 0) {
      if (!appendSurveyPathPoint(exitPoint, false, nextLane, nextVisit)) return false;
      if (!appendDubinsPlan(exitPoint, startYaw, entryPoint, radius, candidates[bestIndex], nextLane, nextVisit)) return false;
      surveyRequiredHeadlandM = max(surveyRequiredHeadlandM, bestHeadland);
      return true;
    }

    extension += radius * 1.5f;
  }
  return false;
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

  float firstStripLength = localDistance(a, b);
  float lastStripLength = localDistance(d, c);
  float leftSideWidth = localDistance(a, d);
  float rightSideWidth = localDistance(b, c);
  float maximumWidth = max(leftSideWidth, rightSideWidth);

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

  uint8_t laneCount = static_cast<uint8_t>(ceilf(maximumWidth / surveyRequestedSpacingM)) + 1;
  if (laneCount < 3) laneCount = 3;
  if (laneCount > MAX_SURVEY_LANES) {
    lastSystemEvent = "BUILD_TOO_MANY_LANES_INCREASE_SPACING";
    return false;
  }

  for (uint8_t lane = 0; lane < laneCount; lane++) {
    float t = static_cast<float>(lane) / static_cast<float>(laneCount - 1);
    surveyLaneLeft[lane] = interpolateLocal(a, d, t);
    surveyLaneRight[lane] = interpolateLocal(b, c, t);
    if (localDistance(surveyLaneLeft[lane], surveyLaneRight[lane]) < SURVEY_MIN_LENGTH_M) {
      lastSystemEvent = "BUILD_STRIP_TOO_SHORT";
      return false;
    }
  }

  uint8_t visit = 0;
  for (uint8_t lane = 0; lane < laneCount; lane += 2) surveyVisitOrder[visit++] = lane;
  surveyFirstPassCount = visit;
  for (uint8_t lane = 1; lane < laneCount; lane += 2) surveyVisitOrder[visit++] = lane;

  surveyPathNodeCount = 0;
  surveyRequiredHeadlandM = 0.0f;
  for (uint8_t visitIndex = 0; visitIndex < laneCount; visitIndex++) {
    uint8_t lane = surveyVisitOrder[visitIndex];
    bool leftToRight = (visitIndex & 1U) == 0U;
    LocalPoint start = leftToRight ? surveyLaneLeft[lane] : surveyLaneRight[lane];
    LocalPoint finish = leftToRight ? surveyLaneRight[lane] : surveyLaneLeft[lane];
    LocalPoint direction = localUnit(localSub(finish, start));

    if (visitIndex == 0) {
      if (!appendSurveyPathPoint(start, false, lane, visitIndex)) {
        lastSystemEvent = "BUILD_PATH_MEMORY";
        return false;
      }
    } else {
      if (!appendSurveyPathPoint(start, false, lane, visitIndex)) {
        lastSystemEvent = "BUILD_PATH_MEMORY";
        return false;
      }
    }

    // Exactly one long segment is marked as a photo strip. The following
    // outside extension and the complete Bezier loop are transit segments.
    if (!appendSurveyPathPoint(finish, true, lane, visitIndex)) {
      lastSystemEvent = "BUILD_PATH_MEMORY";
      return false;
    }

    if (visitIndex + 1 < laneCount) {
      uint8_t nextVisit = visitIndex + 1;
      uint8_t nextLane = surveyVisitOrder[nextVisit];
      bool nextLeftToRight = (nextVisit & 1U) == 0U;
      LocalPoint nextStart = nextLeftToRight ? surveyLaneLeft[nextLane] : surveyLaneRight[nextLane];
      LocalPoint nextFinish = nextLeftToRight ? surveyLaneRight[nextLane] : surveyLaneLeft[nextLane];
      LocalPoint nextDirection = localUnit(localSub(nextFinish, nextStart));
      if (!appendTeardropTurn(finish, direction, nextStart, nextDirection, nextLane, nextVisit)) {
        lastSystemEvent = "BUILD_PATH_MEMORY";
        return false;
      }
    } else {
      LocalPoint finalPoint = localAdd(finish, localScale(direction, PATH_FINAL_EXTENSION_M));
      if (!appendSurveyPathPoint(finalPoint, false, lane, visitIndex)) {
        lastSystemEvent = "BUILD_PATH_MEMORY";
        return false;
      }
    }
  }

  if (surveyPathNodeCount < 2) {
    lastSystemEvent = "BUILD_PATH_EMPTY";
    return false;
  }

  surveyLaneCount = laneCount;
  surveyLengthM = 0.5f * (firstStripLength + lastStripLength);
  surveyWidthM = 0.5f * (leftSideWidth + rightSideWidth);
  surveyAreaM2 = polygonArea4(a, b, c, d);
  surveyActualSpacingM = maximumWidth / static_cast<float>(laneCount - 1);
  surveyTotalRouteM = surveyPathCumulativeM[surveyPathNodeCount - 1];
  surveyRouteBuilt = true;
  headingInitialized = false;
  bowLockPending = false;
  surveyState = SurveyState::IDLE;
  activePathSegment = 0;
  saveSurveyConfigToNvs();
  lastSystemEvent = "TEARDROP_PATH_BUILT_" + String(laneCount) + "_LANES_" + String(surveyPathNodeCount) + "_NODES";
  return true;
}

void configureSurvey(float spacingM, uint32_t photoIntervalMs, float turnRadiusM) {
  surveyRequestedSpacingM = constrain(spacingM, SURVEY_MIN_SPACING_M, SURVEY_MAX_SPACING_M);
  surveyPhotoIntervalMs = constrain(photoIntervalMs, SURVEY_MIN_PHOTO_INTERVAL_MS, SURVEY_MAX_PHOTO_INTERVAL_MS);
  surveyTurnRadiusM = constrain(turnRadiusM, SURVEY_MIN_TURN_RADIUS_M, SURVEY_MAX_TURN_RADIUS_M);
  surveyRouteBuilt = false;
  saveSurveyConfigToNvs();
  lastSystemEvent = "SURVEY_CONFIG_SAVED_REBUILD_PATH";
}

void resetNavigationController() {
  steeringCorrectionUs = 0.0f;
  lastSteeringUpdateMs = millis();
  lastGpsHeadingCorrectionMs = millis();
  gpsHeadingCorrectionTotalDeg = 0.0f;
}

bool chooseSurveyLockTarget(GeoPoint& target, bool& approachStart) {
  approachStart = false;
  if (!surveyRouteBuilt || surveyLaneCount < 1 || !gps.location.isValid()) return false;
  GeoPoint startGeo = localToGeo(surveyA, surveyLaneLeft[surveyVisitOrder[0]]);
  float startDistance = distanceToGeo(startGeo);
  if (startDistance < 0.0f || startDistance > SURVEY_START_NEAR_M) return false;
  target = localToGeo(surveyA, surveyLaneRight[surveyVisitOrder[0]]);
  return true;
}

bool startBowLockCalibration() {
  GeoPoint lockTarget;
  bool approachStart = false;
  if (!chooseSurveyLockTarget(lockTarget, approachStart)) {
    lastSystemEvent = "BOW_LOCK_BLOCKED_BUILD_PATH_OR_MOVE_NEAR_A";
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
  lastSystemEvent = "LOCK_BOW_A_TO_B_HOLD_2S";
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
  lastSystemEvent = "BOW_LOCKED_TEARDROP_READY";
}

void refreshActivePathSegmentMetadata(bool forceEvent) {
  if (surveyPathNodeCount < 2) return;
  if (activePathSegment >= surveyPathNodeCount - 1) activePathSegment = surveyPathNodeCount - 2;
  bool previousPhoto = activeLegIsSurvey;
  int8_t previousLane = activeLaneIndex;

  activeLegIndex = static_cast<int16_t>(activePathSegment);
  activeLegIsSurvey = surveyPathSegmentPhoto[activePathSegment];
  activeLaneIndex = surveyPathSegmentLane[activePathSegment];
  activeVisitIndex = surveyPathSegmentVisit[activePathSegment];
  activeLegLengthM = localDistance(surveyPathNodes[activePathSegment], surveyPathNodes[activePathSegment + 1]);
  activeLegStartedMs = millis();

  if (activeLegIsSurvey && (!previousPhoto || previousLane != activeLaneIndex)) {
    surveyLegStartedMs = millis();
    lastSurveyPhotoRequestMs = 0;
    finalPhotoRequestedForLane = false;
    if (forceEvent) lastSystemEvent = "ENTER_PHOTO_LANE_" + String(activeLaneIndex + 1) + "_VISIT_" + String(activeVisitIndex + 1);
  } else if (!activeLegIsSurvey && previousPhoto && forceEvent) {
    lastSystemEvent = "ENTER_CONTINUOUS_TEARDROP_TO_VISIT_" + String(activeVisitIndex + 1);
  }
}

bool projectOnPathSegment(
  uint16_t segment,
  const LocalPoint& position,
  float& rawT,
  float& clampedT,
  float& signedCrossTrack,
  float& distanceToProjection
) {
  if (segment + 1 >= surveyPathNodeCount) return false;
  LocalPoint p0 = surveyPathNodes[segment];
  LocalPoint p1 = surveyPathNodes[segment + 1];
  LocalPoint vector = localSub(p1, p0);
  float lengthSquared = vector.x * vector.x + vector.y * vector.y;
  if (lengthSquared < 0.0001f) return false;
  LocalPoint relative = localSub(position, p0);
  rawT = (relative.x * vector.x + relative.y * vector.y) / lengthSquared;
  clampedT = constrain(rawT, 0.0f, 1.0f);
  LocalPoint projection = localAdd(p0, localScale(vector, clampedT));
  distanceToProjection = localDistance(position, projection);
  float length = sqrtf(lengthSquared);
  signedCrossTrack = (vector.x * relative.y - vector.y * relative.x) / length;
  return true;
}

LocalPoint pointAtPathDistance(float distanceM) {
  if (surveyPathNodeCount == 0) return {0.0f, 0.0f};
  if (distanceM <= 0.0f) return surveyPathNodes[0];
  float total = surveyPathCumulativeM[surveyPathNodeCount - 1];
  if (distanceM >= total) return surveyPathNodes[surveyPathNodeCount - 1];
  uint16_t segment = activePathSegment;
  while (segment + 1 < surveyPathNodeCount && surveyPathCumulativeM[segment + 1] < distanceM) segment++;
  float startDistance = surveyPathCumulativeM[segment];
  float endDistance = surveyPathCumulativeM[segment + 1];
  float t = (distanceM - startDistance) / max(0.001f, endDistance - startDistance);
  return interpolateLocal(surveyPathNodes[segment], surveyPathNodes[segment + 1], constrain(t, 0.0f, 1.0f));
}

void updatePathProgress() {
  if (!gps.location.isValid() || surveyPathNodeCount < 2) return;
  LocalPoint current = geoToLocal(surveyA, {gps.location.lat(), gps.location.lng()});

  if (lastGpsFixMs != 0 && lastGpsFixMs != lastProcessedPathGpsFixMs) {
    lastProcessedPathGpsFixMs = lastGpsFixMs;

    uint16_t bestSegment = activePathSegment;
    float bestRawT = 0.0f;
    float bestClampedT = 0.0f;
    float bestXte = 0.0f;
    float bestDistance = 9999.0f;
    float bestProgress = surveyPathProgressM;

    uint16_t lastSegment = surveyPathNodeCount - 2;
    uint16_t searchEnd = min<uint16_t>(lastSegment, activePathSegment + PATH_FORWARD_SEARCH_SEGMENTS);
    float maxForwardProgress = surveyPathProgressM + PATH_MAX_PROGRESS_JUMP_M;

    for (uint16_t segment = activePathSegment; segment <= searchEnd; segment++) {
      float rawT = 0.0f;
      float clampedT = 0.0f;
      float xte = 0.0f;
      float distance = 0.0f;
      if (!projectOnPathSegment(segment, current, rawT, clampedT, xte, distance)) continue;
      float segmentLength = localDistance(surveyPathNodes[segment], surveyPathNodes[segment + 1]);
      float candidateProgress = surveyPathCumulativeM[segment] + clampedT * segmentLength;
      if (candidateProgress + PATH_BACKTRACK_TOLERANCE_M < surveyPathProgressM) continue;
      if (candidateProgress > maxForwardProgress && surveyPathProgressM > 0.5f) continue;

      // Prefer the nearest forward piece of the route, with a small penalty
      // against unnecessary jumps when two parts of a U-turn are close in GPS.
      float score = distance + 0.06f * max(0.0f, candidateProgress - surveyPathProgressM);
      float bestScore = bestDistance + 0.06f * max(0.0f, bestProgress - surveyPathProgressM);
      if (score < bestScore) {
        bestSegment = segment;
        bestRawT = rawT;
        bestClampedT = clampedT;
        bestXte = xte;
        bestDistance = distance;
        bestProgress = candidateProgress;
      }
    }

    if (bestSegment != activePathSegment) {
      activePathSegment = bestSegment;
      refreshActivePathSegmentMetadata(true);
    }

    activeSegmentProjection = bestClampedT;
    activeLegAlongM = bestClampedT * max(0.01f, activeLegLengthM);
    activeLegCrossTrackM = bestXte;
    surveyPathDistanceM = bestDistance;
    activeLegProgressPercent = constrain(bestClampedT * 100.0f, 0.0f, 100.0f);
    surveyPathProgressM = max(surveyPathProgressM, bestProgress);
    surveyPathProgressPercent = constrain(100.0f * surveyPathProgressM / max(0.1f, surveyTotalRouteM), 0.0f, 100.0f);

    if (surveyPathProgressM >= meaningfulProgressCheckpointM + 0.8f) {
      meaningfulProgressCheckpointM = surveyPathProgressM;
      lastMeaningfulProgressMs = millis();
    }

    if (activePathSegment == lastSegment) {
      float distanceToFinal = localDistance(current, surveyPathNodes[surveyPathNodeCount - 1]);
      if (bestRawT >= 0.96f || distanceToFinal <= PATH_FINAL_RADIUS_M) pathFinished = true;
    }
    return;
  }

  // Between GPS updates keep steering from the last accepted route position.
  // Do not repeatedly advance on the same stale coordinate.
  float rawT = 0.0f;
  float clampedT = 0.0f;
  float xte = 0.0f;
  float distance = 0.0f;
  if (projectOnPathSegment(activePathSegment, current, rawT, clampedT, xte, distance)) {
    activeLegCrossTrackM = xte;
    surveyPathDistanceM = distance;
  }
}

float computePathDesiredBearing() {
  updatePathProgress();
  if (!gps.location.isValid() || surveyPathNodeCount < 2) return headingEstimateDeg;
  LocalPoint current = geoToLocal(surveyA, {gps.location.lat(), gps.location.lng()});
  float lookahead = activeLegIsSurvey ? PATH_LOOKAHEAD_PHOTO_M : PATH_LOOKAHEAD_TURN_M;
  LocalPoint desired = pointAtPathDistance(min(surveyTotalRouteM, surveyPathProgressM + lookahead));
  float dx = desired.x - current.x;
  float dy = desired.y - current.y;
  if (sqrtf(dx * dx + dy * dy) < 0.35f) {
    desired = pointAtPathDistance(min(surveyTotalRouteM, surveyPathProgressM + lookahead + 1.0f));
    dx = desired.x - current.x;
    dy = desired.y - current.y;
  }
  setActiveTarget(localToGeo(surveyA, desired));
  updateTargetGeometry();
  return bearingFromLocalVector(dx, dy);
}

void updateHeadingGpsCorrection() {
  if (!headingInitialized || surveyState != SurveyState::FOLLOW_PATH) return;
  uint32_t nowMs = millis();
  if (nowMs - lastGpsHeadingCorrectionMs < HEADING_GPS_CORRECTION_INTERVAL_MS) return;
  lastGpsHeadingCorrectionMs = nowMs;
  if (!gps.course.isValid() || gps.course.age() > AUTO_MAX_COURSE_AGE_MS) return;
  if (!gps.speed.isValid() || gps.speed.kmph() < HEADING_GPS_MIN_SPEED_KMPH) return;

  float yawRate = static_cast<float>(gyroCompassSign) * currentGyroZ;
  float maxYaw = activeLegIsSurvey ? HEADING_GPS_STRAIGHT_MAX_YAW_DPS : HEADING_GPS_TURN_MAX_YAW_DPS;
  if (fabsf(yawRate) > maxYaw) return;

  float courseDelta = normalizeHeadingError(static_cast<float>(gps.course.deg()) - headingEstimateDeg);
  float maxDelta = activeLegIsSurvey ? HEADING_GPS_STRAIGHT_MAX_DELTA_DEG : HEADING_GPS_TURN_MAX_DELTA_DEG;
  if (fabsf(courseDelta) > maxDelta) return;

  bool justEnteredPhotoStrip = activeLegIsSurvey && millis() - surveyLegStartedMs <= HEADING_GPS_EXIT_RELOCK_MS;
  float gain = activeLegIsSurvey ? HEADING_GPS_STRAIGHT_GAIN : HEADING_GPS_TURN_GAIN;
  float maxStep = activeLegIsSurvey ? HEADING_GPS_STRAIGHT_MAX_STEP_DEG : HEADING_GPS_TURN_MAX_STEP_DEG;
  if (justEnteredPhotoStrip) {
    gain = HEADING_GPS_EXIT_GAIN;
    maxStep = HEADING_GPS_EXIT_MAX_STEP_DEG;
  }

  float correctionStep = constrain(courseDelta * gain, -maxStep, maxStep);
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
  if (surveyState != SurveyState::FOLLOW_PATH || !activeLegIsSurvey) return;
  if (captureRequestPending || !missionActive || !cameraOnline) return;
  uint32_t nowMs = millis();
  bool firstPhoto = lastSurveyPhotoRequestMs == 0 || lastSurveyPhotoRequestMs < surveyLegStartedMs;
  uint32_t elapsed = firstPhoto ? surveyPhotoIntervalMs : nowMs - lastSurveyPhotoRequestMs;
  bool finalZone = activeLegProgressPercent >= 88.0f && !finalPhotoRequestedForLane;
  if (!firstPhoto && !finalZone && elapsed < surveyPhotoIntervalMs) return;
  bool quality = surveyPhotoQualityOkay();
  bool coverageDeadline = firstPhoto || finalZone || elapsed >= surveyPhotoIntervalMs * 2UL;
  if (quality || coverageDeadline) {
    if (requestSurveyCapture() && finalZone) finalPhotoRequestedForLane = true;
  }
}

void updateSurveySpeedEstimate() {
  if (!gps.speed.isValid()) return;
  float speedMps = static_cast<float>(gps.speed.mps());
  if (speedMps < MIN_VALID_SPEED_MPS || speedMps > MAX_VALID_SPEED_MPS) return;
  estimatedSurveySpeedMps = 0.88f * estimatedSurveySpeedMps + 0.12f * speedMps;
}

void driveContinuousPath() {
  desiredTrackBearingDeg = computePathDesiredBearing();
  currentHeadingErrorDeg = normalizeHeadingError(desiredTrackBearingDeg - headingEstimateDeg);
  if (fabsf(static_cast<float>(currentHeadingErrorDeg)) < SURVEY_HEADING_DEADBAND_DEG) currentHeadingErrorDeg = 0.0f;

  float yawRate = static_cast<float>(gyroCompassSign) * currentGyroZ;
  float kp = activeLegIsSurvey ? SURVEY_HEADING_KP_US_PER_DEG : TURN_HEADING_KP_US_PER_DEG;
  float kd = activeLegIsSurvey ? SURVEY_YAW_DAMPING_US_PER_DPS : TURN_YAW_DAMPING_US_PER_DPS;
  float maxSteering = activeLegIsSurvey ? SURVEY_MAX_STEERING_US : TURN_MAX_STEERING_US;
  float slew = activeLegIsSurvey ? SURVEY_STEERING_SLEW_US : TURN_STEERING_SLEW_US;

  float targetCorrection = kp * static_cast<float>(currentHeadingErrorDeg) - kd * yawRate;
  targetCorrection = constrain(targetCorrection, -maxSteering, maxSteering);
  float step = constrain(targetCorrection - steeringCorrectionUs, -slew, slew);
  steeringCorrectionUs += step;

  int baseUs = activeLegIsSurvey ? SURVEY_BASE_US : TURN_BASE_US;
  float absError = fabsf(static_cast<float>(currentHeadingErrorDeg));
  if (!activeLegIsSurvey && absError > 45.0f) baseUs = TURN_SLOW_BASE_US;
  if (activeLegIsSurvey && absError > 35.0f) baseUs = 1415;
  if (absError > 75.0f) baseUs = 1440;

  int leftUs = static_cast<int>(roundf(baseUs - steeringCorrectionUs));
  int rightUs = static_cast<int>(roundf(baseUs + steeringCorrectionUs));
  int rightRawWeakLimit = SURVEY_WEAKEST_FORWARD_US - max(0, rightForwardTrimUs);
  rightRawWeakLimit = constrain(rightRawWeakLimit, SURVEY_FORWARD_LIMIT_US, SURVEY_WEAKEST_FORWARD_US);

  // Both propellers always remain forward during the complete path, including
  // every teardrop turn. No motor is placed in neutral until the final stop.
  leftUs = constrain(leftUs, SURVEY_FORWARD_LIMIT_US, SURVEY_WEAKEST_FORWARD_US);
  rightUs = constrain(rightUs, SURVEY_FORWARD_LIMIT_US, rightRawWeakLimit);
  writeMotors(leftUs, rightUs);
}

String surveyStartBlockReason() {
  if (surveyRunning()) return "SURVEY_ALREADY_RUNNING";
  if (!surveyRouteBuilt || surveyPathNodeCount < 2) return "BUILD_TEARDROP_PATH";
  if (!motorsArmed) return "ARM_MOTORS";
  if (!gpsGoodForAuto()) return "GPS_QUALITY";
  if (!gyroSignVerified) return "VERIFY_GYRO_SIGN";
  if (!headingInitialized || bowLockPending) return "LOCK_BOW";
  if (millis() - headingLockedAtMs > BOW_LOCK_MAX_START_AGE_MS) return "LOCK_BOW_AGAIN";
  if (millis() - lastMpuGoodMs > MPU_STALE_TIMEOUT_MS) return "MPU_DATA";
  if (!cameraOnline) return "CAMERA_LINK";
  if (!cameraRemoteReady) return "CAMERA_NOT_READY";
  if (!cameraSdReady) return "SD_NOT_READY";
  if (!missionActive) return "START_PHOTO_MISSION";
  if (captureRequestPending) return "CAMERA_BUSY";
  GeoPoint startGeo = localToGeo(surveyA, surveyPathNodes[0]);
  float distanceFromStart = distanceToGeo(startGeo);
  if (distanceFromStart < 0.0f) return "GPS_POSITION";
  if (distanceFromStart > SURVEY_START_NEAR_M) return "MOVE_NEAR_A";
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
  activePathSegment = 0;
  activeLegIsSurvey = false;
  activeLaneIndex = -1;
  activeVisitIndex = 0;
  lastProcessedPathGpsFixMs = 0;
  surveyPathProgressM = 0.0f;
  surveyPathProgressPercent = 0.0f;
  meaningfulProgressCheckpointM = 0.0f;
  lastMeaningfulProgressMs = millis();
  pathFinished = false;
  steeringCorrectionUs = 0.0f;
  surveyState = SurveyState::FOLLOW_PATH;
  refreshActivePathSegmentMetadata(true);
  setActiveTarget(localToGeo(surveyA, surveyPathNodes[1]));
  lastSystemEvent = "INTERLACED_TEARDROP_SURVEY_STARTED";
  autoMessage = "CONTINUOUS_PATH_ACTIVE";
  return true;
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

  if (surveyState == SurveyState::FOLLOW_PATH) {
    driveContinuousPath();
    updateSurveySpeedEstimate();
    updateHeadingGpsCorrection();
    updateSurveyCapture();

    if (surveyPathDistanceM > PATH_MAX_DEVIATION_M) {
      stopAutonomy("PATH_DEVIATION", true);
      return;
    }
    if (nowMs - autoRunStartedMs > PATH_STALL_GRACE_MS && nowMs - lastMeaningfulProgressMs > PATH_STALL_TIMEOUT_MS) {
      stopAutonomy("PATH_NO_PROGRESS", true);
      return;
    }

    if (pathFinished) {
      stopMotors();
      surveyState = SurveyState::FINISH_WAIT;
      autoMessage = "WAIT_FINAL_CAMERA";
      lastSystemEvent = "CONTINUOUS_PATH_COMPLETE";
    }
    return;
  }

  if (surveyState == SurveyState::FINISH_WAIT) {
    stopMotors();
    if (!captureRequestPending) {
      if (missionActive) requestMissionEnd();
      surveyState = SurveyState::COMPLETE;
      autoMessage = "SURVEY_COMPLETE";
      autoRunStartedMs = 0;
      lastSystemEvent = "SURVEY_COMPLETE_PHOTOS_" + String(surveyPhotosSaved);
    }
  }
}

void printNavigationStatus() {
  Serial.println();
  Serial.println("========== V6 TEARDROP SURVEY ==========");
  Serial.print("State: "); Serial.println(surveyStateName());
  Serial.print("Phase: "); Serial.println(surveyPhaseName());
  Serial.print("Path built: "); Serial.println(surveyRouteBuilt ? "YES" : "NO");
  Serial.print("Lanes: "); Serial.print(surveyLaneCount); Serial.print(" spacing: "); Serial.println(surveyActualSpacingM, 2);
  Serial.print("Turn radius/headland: "); Serial.print(surveyTurnRadiusM, 1); Serial.print("/"); Serial.println(surveyRequiredHeadlandM, 1);
  Serial.print("Path segment: "); Serial.print(activePathSegment + 1); Serial.print("/"); Serial.println(max(0, static_cast<int>(surveyPathNodeCount) - 1));
  Serial.print("Lane visit: "); Serial.print(activeVisitIndex + 1); Serial.print("/"); Serial.print(surveyLaneCount); Serial.print(" physical lane: "); Serial.println(activeLaneIndex + 1);
  Serial.print("Mode: "); Serial.println(activeLegIsSurvey ? "PHOTO_STRIP" : "TEARDROP_TURN");
  Serial.print("Path progress: "); Serial.print(surveyPathProgressPercent, 1); Serial.print("% XTE: "); Serial.println(activeLegCrossTrackM, 2);
  Serial.print("Heading: "); Serial.print(headingEstimateDeg, 1); Serial.print(" desired: "); Serial.print(desiredTrackBearingDeg, 1); Serial.print(" error: "); Serial.println(currentHeadingErrorDeg, 1);
  Serial.print("GPS course: "); Serial.println(gps.course.isValid() ? gps.course.deg() : -1.0, 1);
  Serial.print("Photos requested/saved/failed: "); Serial.print(surveyPhotosRequested); Serial.print("/"); Serial.print(surveyPhotosSaved); Serial.print("/"); Serial.println(surveyPhotosFailed);
  Serial.println("=========================================");
}


// ============================================================
// Serial commands + touch-safe mobile control page
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("========== V6 INTERLACED TEARDROP COMMANDS ==========");
  Serial.println("M / E              - start/end camera mission");
  Serial.println("ARM / DISARM / STOP");
  Serial.println("A_HERE / B_HERE / C_HERE / D_HERE - average ordered corners");
  Serial.println("BUILD               - build interlaced continuous teardrop path");
  Serial.println("LOCKBOW             - point bow at shown first target and lock 2 s");
  Serial.println("SURVEY              - start continuous teardrop mission");
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
.card{background:#1d2730;border-radius:14px;padding:12px;margin:9px auto;max-width:760px}
button,input,select{font-size:17px;padding:13px;margin:5px;border:0;border-radius:11px}
button{min-width:125px;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
button.hold{touch-action:none}.go{background:#23a566;color:#fff}.stop{background:#d62828;color:#fff}.manual{background:#74c0fc}.mode{background:#f0b429}.safe{background:#6c757d;color:#fff}
input{width:138px;-webkit-user-select:text;user-select:text}.mono{font-family:monospace;text-align:left;white-space:pre-wrap;font-size:13px}.small{font-size:13px;color:#cbd5e1}.warn{font-size:14px;color:#ffd166}.ready{font-weight:700;font-size:20px;padding:10px;border-radius:10px;background:#6c2}.blocked{font-weight:700;font-size:18px;padding:10px;border-radius:10px;background:#a33}
</style></head><body>
<h2>Catamaran V6.0 Interlaced Teardrop</h2>
<div class="card"><div id="readyBox" class="blocked">CHECKING...</div><div class="small" id="reason"></div><button class="safe" onclick="cmd('arm')">ARM</button><button class="safe" onclick="cmd('disarm')">DISARM</button><button class="stop" onclick="cmd('stop')">EMERGENCY STOP</button></div>
<div class="card"><h3>Manual control</h3><button class="manual hold" data-m="f">FORWARD</button><br><button class="manual hold" data-m="l">LEFT</button><button class="stop" onclick="manualStop()">STOP</button><button class="manual hold" data-m="r">RIGHT</button><br><button class="manual hold" data-m="b">REVERSE</button><p class="small">First setup only: hold LEFT or RIGHT for at least one second if gyro sign is not verified.</p></div>
<div class="card"><h3>Motor balance</h3><button class="mode" onclick="cmd('trimminus')">-5 us</button><button class="mode" onclick="cmd('trimreset')">RESET 20</button><button class="mode" onclick="cmd('trimplus')">+5 us</button></div>
<div class="card"><h3>1. Saved search area</h3><p class="small">A → B → C → D around the boundary. A-B is the first photo direction. Existing points survive normal reflashing.</p><button class="mode" onclick="cmd('savea')">SAVE A HERE</button><button class="mode" onclick="cmd('saveb')">SAVE B HERE</button><button class="mode" onclick="cmd('savec')">SAVE C HERE</button><button class="mode" onclick="cmd('saved')">SAVE D HERE</button><br><button class="safe" onclick="cmd('cleararea')">CLEAR AREA</button></div>
<div class="card"><h3>Manual coordinate entry</h3><select id="corner"><option>A</option><option>B</option><option>C</option><option>D</option></select><br><input id="cornerLat" inputmode="decimal" placeholder="latitude"><input id="cornerLon" inputmode="decimal" placeholder="longitude"><br><button class="mode" onclick="setCorner()">SET SELECTED CORNER</button></div>
<div class="card"><h3>2. Coverage and turn settings</h3><label>Photo-strip spacing, m<br><input id="spacing" type="number" step="0.1" min="1.0" max="8.0" value="1.5"></label><br><label>Photo interval, ms<br><input id="photo" type="number" step="50" min="900" max="3000" value="1100"></label><br><label>Minimum turn radius, m<br><input id="turnRadius" type="number" step="0.5" min="1.4" max="4.0" value="2.0"></label><p class="warn">The solver keeps curvature at or above this radius and shows the actual clear-water headland required beyond both ends.</p><button class="mode" onclick="saveConfig()">SAVE SETTINGS</button><button class="go" onclick="cmd('build')">BUILD TEARDROP PATH</button></div>
<div class="card"><h3>3. Heading and camera</h3><button class="mode" onclick="cmd('levelzero')">SET WATER LEVEL ZERO</button><br><p class="small">Place the boat near A and point the bow from A toward B.</p><button class="go" onclick="cmd('lockbow')">LOCK BOW (2 s)</button><button class="safe" onclick="cmd('clearbow')">CLEAR BOW LOCK</button><br><button class="mode" onclick="cmd('camerastatus')">REFRESH CAMERA / SD</button><br><button class="mode" onclick="cmd('missionstart')">START PHOTO MISSION</button><button class="mode" onclick="cmd('capturetest')">TEST PHOTO</button></div>
<div class="card"><h3>4. Continuous mission</h3><p class="small">Order: lanes 1,3,5... then 2,4,6.... No stopping at lane ends; both propellers stay forward through every smooth teardrop.</p><button class="go" onclick="cmd('survey')">START CONTINUOUS SURVEY</button><button class="mode" onclick="cmd('missionend')">END PHOTO MISSION</button></div>
<div class="card mono" id="status">Loading...</div>
<script>
let timer=null;
async function cmd(c){try{await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'})}catch(e){}setTimeout(update,180)}
async function sendManual(m){try{await fetch('/manual?m='+m,{cache:'no-store'})}catch(e){}}
function manualStart(m){manualStop(false);sendManual(m);timer=setInterval(()=>sendManual(m),200)}
function manualStop(send=true){if(timer){clearInterval(timer);timer=null}if(send)sendManual('0')}
document.querySelectorAll('.hold').forEach(b=>{b.addEventListener('pointerdown',e=>{e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(x){}manualStart(b.dataset.m)});b.addEventListener('pointerup',e=>{e.preventDefault();manualStop()});b.addEventListener('pointercancel',e=>{e.preventDefault();manualStop()});b.addEventListener('lostpointercapture',()=>manualStop());b.addEventListener('contextmenu',e=>e.preventDefault())});
document.addEventListener('selectstart',e=>{if(e.target.closest('button'))e.preventDefault()});window.addEventListener('blur',()=>manualStop());window.addEventListener('pagehide',()=>manualStop());
async function saveConfig(){const s=document.getElementById('spacing').value;const p=document.getElementById('photo').value;const t=document.getElementById('turnRadius').value;await fetch('/surveyconfig?spacing='+encodeURIComponent(s)+'&photo='+encodeURIComponent(p)+'&turn='+encodeURIComponent(t),{cache:'no-store'});update()}
async function setCorner(){const slot=document.getElementById('corner').value;const lat=document.getElementById('cornerLat').value;const lon=document.getElementById('cornerLon').value;await fetch('/setcorner?slot='+encodeURIComponent(slot)+'&lat='+encodeURIComponent(lat)+'&lon='+encodeURIComponent(lon),{cache:'no-store'});update()}
async function update(){try{const s=await(await fetch('/status',{cache:'no-store'})).json();document.getElementById('spacing').value=s.requestedSpacing;document.getElementById('photo').value=s.photoInterval;document.getElementById('turnRadius').value=s.turnRadius;const box=document.getElementById('readyBox');box.textContent=s.ready==='YES'?'READY TO START':'NOT READY';box.className=s.ready==='YES'?'ready':'blocked';document.getElementById('reason').textContent=s.ready==='YES'?'All start checks passed':'Next action: '+s.blockReason;document.getElementById('status').textContent=
'GPS: '+s.gps+' SAT:'+s.sat+' HDOP:'+s.hdop+' SPEED:'+s.speed+' km/h COURSE:'+s.gpsCourse+'\n'+
'CAM:'+s.camera+' READY:'+s.cameraReady+' SD:'+s.sd+' MISSION:'+s.mission+'\n'+
'MOTORS:'+s.motors+' L/R:'+s.left+'/'+s.right+' TRIM:'+s.rightTrim+' us\n'+
'GYRO SIGN:'+s.gyroSign+' VERIFIED:'+s.gyroSignVerified+' BOW:'+s.bowLock+' HEADING:'+s.heading+'\n'+
'A:'+s.aSet+' B:'+s.bSet+' C:'+s.cSet+' D:'+s.dSet+' PATH:'+s.routeBuilt+'\n'+
'AREA: '+s.length+' x '+s.width+' m ('+s.area+' m2) LANES:'+s.lanes+' SPACING:'+s.actualSpacing+' m\n'+
'TURN RADIUS:'+s.turnRadius+' m REQUIRED CLEAR WATER:'+s.requiredHeadland+' m EACH END\n'+
'PATH: '+s.pathProgress+' / '+s.routeDistance+' m ('+s.pathPercent+'%) NODES:'+s.pathNodes+'\n'+
'SURVEY:'+s.auto+' '+s.autoMessage+' PHASE:'+s.phase+'\n'+
'VISIT:'+s.visit+'/'+s.lanes+' PHYSICAL LANE:'+s.lane+' MODE:'+s.legType+'\n'+
'SEGMENT:'+s.segment+'/'+s.segmentTotal+' SEG PROGRESS:'+s.progress+'% XTE:'+s.xte+' m PATH DIST:'+s.pathDistance+' m\n'+
'TARGET DIST:'+s.distance+' m DESIRED:'+s.desired+' ERROR:'+s.headingError+' GPS CORR:'+s.gpsCorrection+'\n'+
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
  String legType = activeLegIsSurvey ? "PHOTO_STRIP" : "TEARDROP_TURN";
  String json = "{";
  json += "\"ready\":\"" + readyState + "\",\"blockReason\":\"" + blockReason + "\",";
  json += "\"gps\":\"" + gpsState + "\",";
  json += "\"sat\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  json += "\"hdop\":" + jsonNumber(gps.hdop.isValid() ? gps.hdop.hdop() : 99.99, 2) + ",";
  json += "\"speed\":" + jsonNumber(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2) + ",";
  json += "\"gpsCourse\":" + jsonNumber(gps.course.isValid() ? gps.course.deg() : -1.0, 1) + ",";
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
  json += "\"turnRadius\":" + jsonNumber(surveyTurnRadiusM, 1) + ",\"requiredHeadland\":" + jsonNumber(surveyRequiredHeadlandM, 1) + ",";
  json += "\"auto\":\"" + surveyStateName() + "\",\"autoMessage\":\"" + autoMessage + "\",\"phase\":\"" + surveyPhaseName() + "\",";
  json += "\"visit\":" + String(surveyRunning() ? activeVisitIndex + 1 : 0) + ",\"lane\":" + String(activeLaneIndex >= 0 ? activeLaneIndex + 1 : 0) + ",\"legType\":\"" + legType + "\",";
  json += "\"segment\":" + String(surveyRunning() ? activePathSegment + 1 : 0) + ",\"segmentTotal\":" + String(max(0, static_cast<int>(surveyPathNodeCount) - 1)) + ",";
  json += "\"progress\":" + jsonNumber(activeLegProgressPercent, 1) + ",\"xte\":" + jsonNumber(activeLegCrossTrackM, 2) + ",\"pathDistance\":" + jsonNumber(surveyPathDistanceM, 2) + ",";
  json += "\"pathProgress\":" + jsonNumber(surveyPathProgressM, 1) + ",\"pathPercent\":" + jsonNumber(surveyPathProgressPercent, 1) + ",\"pathNodes\":" + String(surveyPathNodeCount) + ",";
  json += "\"distance\":" + jsonNumber(currentTargetDistanceM, 1) + ",\"bearing\":" + jsonNumber(currentTargetBearingDeg, 1) + ",\"desired\":" + jsonNumber(desiredTrackBearingDeg, 1) + ",\"headingError\":" + jsonNumber(currentHeadingErrorDeg, 1) + ",\"gpsCorrection\":" + jsonNumber(gpsHeadingCorrectionTotalDeg, 1) + ",";
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
  else if (command == "camerastatus") { requestCameraStatus(); requestMissionStatus(); lastSystemEvent = "CAMERA_STATUS_REFRESH_SENT"; }
  else if (command == "missionstart") requestMissionStart();
  else if (command == "capturetest") { pendingCaptureIsSurvey = false; requestCapture(true); }
  else if (command == "missionend") requestMissionEnd();
  else if (command == "survey") startSurvey();
  controlServer.send(200, "text/plain", "OK");
}

void handleSurveyConfig() {
  float spacing = controlServer.arg("spacing").toFloat();
  uint32_t photo = static_cast<uint32_t>(controlServer.arg("photo").toInt());
  float turnRadius = controlServer.arg("turn").toFloat();
  configureSurvey(spacing, photo, turnRadius);
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
  Serial.print(" PHASE:"); Serial.print(surveyPhaseName());
  Serial.print(" VISIT:"); Serial.print(surveyRunning() ? activeVisitIndex + 1 : 0); Serial.print("/"); Serial.print(surveyLaneCount);
  Serial.print(" LANE:"); Serial.print(activeLaneIndex + 1);
  Serial.print(" MODE:"); Serial.print(activeLegIsSurvey ? "PHOTO" : "TURN");
  Serial.print(" | PATH:"); Serial.print(surveyPathProgressPercent, 1); Serial.print("%");
  Serial.print(" H:"); Serial.print(headingInitialized ? headingEstimateDeg : -1.0f, 1);
  Serial.print(" DES:"); Serial.print(desiredTrackBearingDeg, 1);
  Serial.print(" ERR:"); Serial.print(currentHeadingErrorDeg, 1);
  Serial.print(" XTE:"); Serial.print(activeLegCrossTrackM, 2);
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
  Serial.println("AUTONOMY V6.0: INTERLACED LANES + CONTINUOUS TEARDROP TURNS");
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

  // Initial requests are harmless if the camera is still booting; the periodic
  // refresh in updateCameraUart() will repeat them until a valid reply arrives.
  sendPing();
  requestCameraStatus();
  requestMissionStatus();

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
