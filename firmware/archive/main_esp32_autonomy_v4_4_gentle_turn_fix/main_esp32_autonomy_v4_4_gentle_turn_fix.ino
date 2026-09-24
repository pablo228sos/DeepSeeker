#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

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
// Use a little more speed while acquiring the first GPS course.
// NEO-6M course is unreliable when the boat is barely moving.
constexpr int AUTO_COURSE_ACQUIRE_US = 1370;
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

bool targetAveraging = false;
uint32_t targetAverageStartedMs = 0;
uint32_t targetAverageLastSampleMs = 0;
double targetAverageLatSum = 0.0;
double targetAverageLonSum = 0.0;
uint32_t targetAverageSamples = 0;
constexpr uint32_t TARGET_AVERAGE_DURATION_MS = 10000;
constexpr uint32_t TARGET_AVERAGE_SAMPLE_INTERVAL_MS = 200;

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
constexpr float AUTO_MIN_COURSE_SPEED_KMPH = 0.75f;
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

  previousUpdateUs = micros();
  previousPrintMs = millis();

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
    if (captureRequestPending && responseId == pendingCaptureRequestId) {
      captureRequestPending = false;
      lastCaptureResult = 1;
    }

    lastSystemEvent = "CAPTURE_OK_" + csvField(response, 2);
    Serial.println("PHOTO + TELEMETRY + GEO SAVED SUCCESSFULLY");
    Serial.print("Saved path: ");
    Serial.println(csvField(response, 2));
    return;
  }

  if (response.startsWith("CAPTURE_PARTIAL,")) {
    uint32_t responseId = csvField(response, 1).toInt();
    if (captureRequestPending && responseId == pendingCaptureRequestId) {
      captureRequestPending = false;
      lastCaptureResult = -1;
    }

    lastSystemEvent = "CAPTURE_PARTIAL";
    Serial.println("PHOTO SAVED, BUT TELEMETRY/GEO WRITE FAILED");
    return;
  }

  if (response.startsWith("CAPTURE_ERROR,")) {
    uint32_t responseId = csvField(response, 1).toInt();
    if (captureRequestPending && responseId == pendingCaptureRequestId) {
      captureRequestPending = false;
      lastCaptureResult = -1;
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
// Motor layer + single-waypoint autonomous controller
// ============================================================

enum class AutoState : uint8_t {
  IDLE,
  ACQUIRE_COURSE,
  NAVIGATE,
  SETTLE,
  CAPTURE_WAIT,
  COMPLETE,
  FAILSAFE
};

AutoState autoState = AutoState::IDLE;
String autoMessage = "IDLE";

bool targetSet = false;
double targetLatitude = 0.0;
double targetLongitude = 0.0;
double currentTargetDistanceM = -1.0;
double currentTargetBearingDeg = -1.0;
double currentHeadingErrorDeg = 0.0;

// Smoothed GPS-course controller. The old firmware used a pure P controller,
// which reacts to delayed/noisy GPS course and can create a snake trajectory.
bool filteredCourseValid = false;
float filteredCourseDeg = 0.0f;
float previousHeadingErrorDeg = 0.0f;
float filteredHeadingErrorRateDegPerSec = 0.0f;
float steeringCorrectionUs = 0.0f;
uint32_t lastSteeringUpdateMs = 0;
uint32_t lastCourseFilterUpdateMs = 0;

uint32_t autoStateStartedMs = 0;
uint32_t previousAutoUpdateMs = 0;

constexpr uint32_t AUTO_UPDATE_INTERVAL_MS = 100;
constexpr uint32_t AUTO_COURSE_ACQUIRE_TIMEOUT_MS = 8000;
constexpr uint32_t AUTO_SETTLE_MIN_MS = 900;
constexpr uint32_t AUTO_SETTLE_FORCE_CAPTURE_MS = 2000;
constexpr uint32_t AUTO_CAPTURE_TIMEOUT_MS = 15000;
constexpr float AUTO_ARRIVAL_RADIUS_M = 3.0f;
constexpr float AUTO_MAX_ROLL_DEG = 22.0f;
constexpr float AUTO_MAX_PITCH_DEG = 22.0f;

constexpr uint32_t AUTO_COURSE_ACQUIRE_MIN_MS = 2500;
constexpr float AUTO_HEADING_DEADBAND_DEG = 8.0f;
constexpr float AUTO_COURSE_FILTER_GAIN = 0.30f;
// Gentle raw-GPS controller. GPS course is delayed, so steering must remain mild.
constexpr float AUTO_HEADING_KP_US_PER_DEG = 0.35f;
constexpr float AUTO_HEADING_KD_US_PER_DEG_PER_SEC = 0.0f;
constexpr float AUTO_MAX_CORRECTION_US = 28.0f;
constexpr float AUTO_STEERING_SLEW_US_PER_UPDATE = 4.0f;
// Final forward command is never allowed to approach/cross neutral during AUTO.
constexpr int AUTO_WEAKEST_FORWARD_US = 1450;

// First-water-test geofence/runtime limits.
constexpr float AUTO_MIN_START_DISTANCE_M = 8.0f;
constexpr float AUTO_MAX_START_DISTANCE_M = 35.0f;
constexpr float AUTO_MAX_DISTANCE_FROM_START_M = 50.0f;
constexpr uint32_t AUTO_MAX_RUN_TIME_MS = 120000;

double autoStartLatitude = 0.0;
double autoStartLongitude = 0.0;
uint32_t autoRunStartedMs = 0;

String autoStateName() {
  switch (autoState) {
    case AutoState::IDLE: return "IDLE";
    case AutoState::ACQUIRE_COURSE: return "ACQUIRE_COURSE";
    case AutoState::NAVIGATE: return "NAVIGATE";
    case AutoState::SETTLE: return "SETTLE";
    case AutoState::CAPTURE_WAIT: return "CAPTURE_WAIT";
    case AutoState::COMPLETE: return "COMPLETE";
    case AutoState::FAILSAFE: return "FAILSAFE";
  }
  return "UNKNOWN";
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
  Serial.print("Right forward trim: ");
  Serial.print(rightForwardTrimUs);
  Serial.println(" us");
}

void resetNavigationController() {
  filteredCourseValid = false;
  filteredCourseDeg = 0.0f;
  previousHeadingErrorDeg = 0.0f;
  filteredHeadingErrorRateDegPerSec = 0.0f;
  steeringCorrectionUs = 0.0f;
  lastSteeringUpdateMs = millis();
  lastCourseFilterUpdateMs = 0;
}

bool gpsCourseReady();

void updateFilteredGpsCourse() {
  if (!gpsCourseReady()) return;

  uint32_t nowMs = millis();

  if (
    filteredCourseValid &&
    nowMs - lastCourseFilterUpdateMs < 300
  ) {
    return;
  }

  float rawCourse = static_cast<float>(gps.course.deg());

  if (!filteredCourseValid) {
    filteredCourseDeg = normalizeHeading360(rawCourse);
    filteredCourseValid = true;
  } else {
    float delta = normalizeHeadingError(rawCourse - filteredCourseDeg);
    filteredCourseDeg = normalizeHeading360(
      filteredCourseDeg + AUTO_COURSE_FILTER_GAIN * delta
    );
  }

  lastCourseFilterUpdateMs = nowMs;
}

void writeMotors(int leftUs, int rightUs) {
  // Apply field-calibrated correction only to forward right-motor commands.
  // Positive trim increases the pulse toward neutral and weakens that motor.
  if (rightUs < ESC_NEUTRAL_US) {
    rightUs += rightForwardTrimUs;
    // V4.3 applied trim after the AUTO limiter, so 1470 + 35 became 1505.
    // That stopped/reversed one motor and caused the abrupt pivot seen on water.
    rightUs = constrain(rightUs, 1000, AUTO_WEAKEST_FORWARD_US);
  }

  if (leftUs < ESC_NEUTRAL_US) {
    leftUs = constrain(leftUs, 1000, AUTO_WEAKEST_FORWARD_US);
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

void stopMotors() {
  currentLeftUs = ESC_NEUTRAL_US;
  currentRightUs = ESC_NEUTRAL_US;
  escLeft.writeMicroseconds(ESC_NEUTRAL_US);
  escRight.writeMicroseconds(ESC_NEUTRAL_US);
}

bool armMotors() {
  if (millis() - bootStartedMs < ESC_ARM_DELAY_MS) {
    Serial.println("ARM BLOCKED: wait for ESC neutral arming delay");
    return false;
  }

  stopMotors();
  motorsArmed = true;
  Serial.println("MOTORS ARMED (software)");
  return true;
}

void disarmMotors() {
  manualDriveActive = false;
  stopMotors();
  motorsArmed = false;
  autoState = AutoState::IDLE;
  autoMessage = "DISARMED";
  Serial.println("MOTORS DISARMED");
}

void startTimedMotorTest(int leftUs, int rightUs) {
  if (!motorsArmed) {
    Serial.println("MOTOR TEST BLOCKED: send ARM first");
    return;
  }

  manualDriveActive = false;
  autoState = AutoState::IDLE;
  autoMessage = "MANUAL_TEST";
  writeMotors(leftUs, rightUs);
  motorTestStopAtMs = millis() + MOTOR_TEST_DURATION_MS;
}

void updateMotorTestTimeout() {
  if (motorTestStopAtMs != 0 && millis() >= motorTestStopAtMs) {
    motorTestStopAtMs = 0;
    stopMotors();
    Serial.println("MOTOR TEST FINISHED -> STOP");
  }
}

bool gpsGoodForAuto() {
  if (!gpsHasFreshFix()) return false;
  if (!gps.satellites.isValid()) return false;
  if (gps.satellites.value() < AUTO_MIN_SATELLITES) return false;
  if (gps.hdop.isValid() && gps.hdop.hdop() > AUTO_MAX_HDOP) return false;
  return true;
}

bool gpsCourseReady() {
  return
    gps.course.isValid() &&
    gps.course.age() <= AUTO_MAX_COURSE_AGE_MS &&
    gps.speed.isValid() &&
    gps.speed.kmph() >= AUTO_MIN_COURSE_SPEED_KMPH;
}

void setTarget(double latitude, double longitude);

void saveTargetToNvs() {
  preferences.putBool("targetSet", targetSet);
  if (targetSet) {
    preferences.putDouble("targetLat", targetLatitude);
    preferences.putDouble("targetLon", targetLongitude);
  }
}

void loadTargetFromNvs() {
  bool saved = preferences.getBool("targetSet", false);
  if (!saved) return;

  double latitude = preferences.getDouble("targetLat", 0.0);
  double longitude = preferences.getDouble("targetLon", 0.0);
  if (
    latitude >= -90.0 && latitude <= 90.0 &&
    longitude >= -180.0 && longitude <= 180.0 &&
    (fabs(latitude) > 0.0001 || fabs(longitude) > 0.0001)
  ) {
    targetLatitude = latitude;
    targetLongitude = longitude;
    targetSet = true;
    Serial.print("SAVED TARGET LOADED: ");
    Serial.print(targetLatitude, 7);
    Serial.print(",");
    Serial.println(targetLongitude, 7);
  }
}

void startTargetAveraging() {
  if (!gpsGoodForAuto()) {
    Serial.println("TARGET AVERAGE BLOCKED: need fresh GPS fix");
    return;
  }

  stopMotors();
  manualDriveActive = false;
  targetAveraging = true;
  targetAverageStartedMs = millis();
  targetAverageLastSampleMs = 0;
  targetAverageLatSum = 0.0;
  targetAverageLonSum = 0.0;
  targetAverageSamples = 0;
  Serial.println("TARGET AVERAGING STARTED: keep robot still for 10 seconds");
}

void updateTargetAveraging() {
  if (!targetAveraging) return;

  uint32_t nowMs = millis();
  if (
    gpsGoodForAuto() &&
    (targetAverageLastSampleMs == 0 ||
     nowMs - targetAverageLastSampleMs >= TARGET_AVERAGE_SAMPLE_INTERVAL_MS)
  ) {
    targetAverageLastSampleMs = nowMs;
    targetAverageLatSum += gps.location.lat();
    targetAverageLonSum += gps.location.lng();
    targetAverageSamples++;
  }

  if (nowMs - targetAverageStartedMs < TARGET_AVERAGE_DURATION_MS) return;

  targetAveraging = false;
  if (targetAverageSamples < 10) {
    Serial.println("TARGET AVERAGE FAILED: not enough valid GPS samples");
    return;
  }

  double latitude = targetAverageLatSum / static_cast<double>(targetAverageSamples);
  double longitude = targetAverageLonSum / static_cast<double>(targetAverageSamples);
  setTarget(latitude, longitude);
  Serial.print("TARGET AVERAGE COMPLETE, samples: ");
  Serial.println(targetAverageSamples);
}

void setTarget(double latitude, double longitude) {
  if (
    latitude < -90.0 || latitude > 90.0 ||
    longitude < -180.0 || longitude > 180.0
  ) {
    Serial.println("TARGET ERROR: invalid latitude/longitude");
    return;
  }

  targetLatitude = latitude;
  targetLongitude = longitude;
  targetSet = true;
  saveTargetToNvs();

  Serial.print("TARGET SET: ");
  Serial.print(targetLatitude, 7);
  Serial.print(",");
  Serial.println(targetLongitude, 7);
}

void stopAutonomy(const String& reason, bool failsafe) {
  manualDriveActive = false;
  stopMotors();
  captureRequestPending = false;
  autoState = failsafe ? AutoState::FAILSAFE : AutoState::IDLE;
  autoStateStartedMs = millis();
  autoMessage = reason;
  autoRunStartedMs = 0;

  lastSystemEvent = failsafe ? ("FAILSAFE_" + reason) : reason;
  Serial.print(failsafe ? "AUTO FAILSAFE: " : "AUTO STOP: ");
  Serial.println(reason);
}

bool startSingleWaypointAuto() {
  if (!motorsArmed) {
    Serial.println("AUTO BLOCKED: motors are not armed");
    return false;
  }
  if (!targetSet) {
    Serial.println("AUTO BLOCKED: set TARGET,lat,lon first");
    return false;
  }
  if (!gpsGoodForAuto()) {
    Serial.println("AUTO BLOCKED: GPS quality is insufficient");
    return false;
  }
  if (!cameraOnline) {
    Serial.println("AUTO BLOCKED: camera is offline");
    return false;
  }
  if (!missionActive) {
    Serial.println("AUTO BLOCKED: start camera mission with M first");
    return false;
  }

  updateTargetGeometry();
  if (
    currentTargetDistanceM < AUTO_MIN_START_DISTANCE_M ||
    currentTargetDistanceM > AUTO_MAX_START_DISTANCE_M
  ) {
    Serial.print("AUTO BLOCKED: first-test target must be 8-35 m away, current: ");
    Serial.print(currentTargetDistanceM, 1);
    Serial.println(" m");
    return false;
  }

  autoStartLatitude = gps.location.lat();
  autoStartLongitude = gps.location.lng();
  autoRunStartedMs = millis();
  manualDriveActive = false;
  resetNavigationController();
  lastCaptureResult = 0;
  captureRequestPending = false;
  autoState = AutoState::ACQUIRE_COURSE;
  autoStateStartedMs = millis();
  autoMessage = "STARTED";
  Serial.println("AUTO SINGLE WAYPOINT STARTED");
  return true;
}

void updateTargetGeometry() {
  if (!targetSet || !gps.location.isValid()) {
    currentTargetDistanceM = -1.0;
    currentTargetBearingDeg = -1.0;
    return;
  }

  currentTargetDistanceM = TinyGPSPlus::distanceBetween(
    gps.location.lat(), gps.location.lng(),
    targetLatitude, targetLongitude
  );

  currentTargetBearingDeg = TinyGPSPlus::courseTo(
    gps.location.lat(), gps.location.lng(),
    targetLatitude, targetLongitude
  );
}

void driveTowardTarget() {
  // GPS course is a direction of movement, not the direction of the bow.
  // It becomes useful only after the boat has gained enough speed.
  if (!gpsCourseReady()) {
    currentHeadingErrorDeg = 0.0;
    steeringCorrectionUs = 0.0f;
    writeMotors(AUTO_COURSE_ACQUIRE_US, AUTO_COURSE_ACQUIRE_US);
    return;
  }

  const float currentCourse =
    normalizeHeading360(static_cast<float>(gps.course.deg()));

  currentHeadingErrorDeg = normalizeHeadingError(
    static_cast<float>(currentTargetBearingDeg) - currentCourse
  );

  if (fabsf(currentHeadingErrorDeg) < AUTO_HEADING_DEADBAND_DEG) {
    currentHeadingErrorDeg = 0.0f;
  }

  const float absoluteError =
    fabsf(static_cast<float>(currentHeadingErrorDeg));

  float targetCorrection =
    AUTO_HEADING_KP_US_PER_DEG *
    static_cast<float>(currentHeadingErrorDeg);

  // A NEO-6M course update is delayed. Never pivot from one delayed reading.
  // Even for a 180-degree error, both motors continue moving forward.
  float dynamicMaxCorrection = 10.0f;
  if (absoluteError >= 25.0f) dynamicMaxCorrection = 16.0f;
  if (absoluteError >= 60.0f) dynamicMaxCorrection = 22.0f;
  if (absoluteError >= 110.0f) dynamicMaxCorrection = AUTO_MAX_CORRECTION_US;

  targetCorrection = constrain(
    targetCorrection,
    -dynamicMaxCorrection,
    dynamicMaxCorrection
  );

  // Smooth only the motor command. The raw GPS course itself is not filtered.
  float correctionStep = constrain(
    targetCorrection - steeringCorrectionUs,
    -AUTO_STEERING_SLEW_US_PER_UPDATE,
    AUTO_STEERING_SLEW_US_PER_UPDATE
  );
  steeringCorrectionUs += correctionStep;

  int baseUs = AUTO_CRUISE_US;

  if (currentTargetDistanceM > 15.0) {
    baseUs = AUTO_FAST_US;
  } else if (currentTargetDistanceM < 7.0) {
    baseUs = AUTO_SLOW_US;
  }

  // Large direction errors use a wide gentle arc instead of stopping one side.
  if (absoluteError > 35.0f) baseUs = max(baseUs, 1380);
  if (absoluteError > 70.0f) baseUs = max(baseUs, 1400);
  if (absoluteError > 120.0f) baseUs = max(baseUs, 1415);

  // Positive error means the target is clockwise/right of the movement course:
  // strengthen the left motor and weaken the right motor.
  int leftUs =
    static_cast<int>(roundf(baseUs - steeringCorrectionUs));
  int rightUs =
    static_cast<int>(roundf(baseUs + steeringCorrectionUs));

  // Account for right trim before writeMotors() adds it. This guarantees that
  // the final right command cannot cross neutral during an automatic turn.
  int rightRawWeakLimit = AUTO_WEAKEST_FORWARD_US;
  if (rightForwardTrimUs > 0) {
    rightRawWeakLimit -= rightForwardTrimUs;
  }
  rightRawWeakLimit = constrain(
    rightRawWeakLimit,
    AUTO_FORWARD_LIMIT_US,
    AUTO_WEAKEST_FORWARD_US
  );

  leftUs = constrain(
    leftUs,
    AUTO_FORWARD_LIMIT_US,
    AUTO_WEAKEST_FORWARD_US
  );
  rightUs = constrain(
    rightUs,
    AUTO_FORWARD_LIMIT_US,
    rightRawWeakLimit
  );

  writeMotors(leftUs, rightUs);
}

void updateAutopilot() {
  updateMotorTestTimeout();

  uint32_t nowMs = millis();
  if (nowMs - previousAutoUpdateMs < AUTO_UPDATE_INTERVAL_MS) {
    return;
  }
  previousAutoUpdateMs = nowMs;

  updateTargetGeometry();

  if (
    autoState == AutoState::IDLE ||
    autoState == AutoState::COMPLETE ||
    autoState == AutoState::FAILSAFE
  ) {
    return;
  }

  if (!motorsArmed) {
    stopAutonomy("MOTORS_DISARMED", true);
    return;
  }

  if (!gpsGoodForAuto()) {
    stopAutonomy("GPS_LOST_OR_POOR", true);
    return;
  }

  if (autoRunStartedMs != 0 && nowMs - autoRunStartedMs > AUTO_MAX_RUN_TIME_MS) {
    stopAutonomy("AUTO_TIME_LIMIT", true);
    return;
  }

  if (autoRunStartedMs != 0) {
    double distanceFromStart = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(),
      autoStartLatitude, autoStartLongitude
    );
    if (distanceFromStart > AUTO_MAX_DISTANCE_FROM_START_M) {
      stopAutonomy("GEOFENCE_FROM_START", true);
      return;
    }
  }

  if (
    fabsf(relativeRollDegrees) > AUTO_MAX_ROLL_DEG ||
    fabsf(relativePitchDegrees) > AUTO_MAX_PITCH_DEG
  ) {
    stopAutonomy("EXCESSIVE_TILT", true);
    return;
  }

  if (!cameraOnline) {
    stopAutonomy("CAMERA_OFFLINE", true);
    return;
  }

  switch (autoState) {
    case AutoState::ACQUIRE_COURSE:
      if (currentTargetDistanceM <= AUTO_ARRIVAL_RADIUS_M) {
        stopMotors();
        autoState = AutoState::SETTLE;
        autoStateStartedMs = nowMs;
        autoMessage = "ARRIVED";
        break;
      }

      if (
        nowMs - autoStateStartedMs >= AUTO_COURSE_ACQUIRE_MIN_MS &&
        gpsCourseReady()
      ) {
        autoState = AutoState::NAVIGATE;
        autoStateStartedMs = nowMs;
        autoMessage = "COURSE_OK_GENTLE";
        lastSystemEvent = "AUTO_NAVIGATE_GENTLE";
        break;
      }

      if (nowMs - autoStateStartedMs > AUTO_COURSE_ACQUIRE_TIMEOUT_MS) {
        stopAutonomy("NO_GPS_COURSE", true);
        break;
      }

      writeMotors(AUTO_COURSE_ACQUIRE_US, AUTO_COURSE_ACQUIRE_US);
      break;

    case AutoState::NAVIGATE:
      if (currentTargetDistanceM <= AUTO_ARRIVAL_RADIUS_M) {
        stopMotors();
        autoState = AutoState::SETTLE;
        autoStateStartedMs = nowMs;
        autoMessage = "ARRIVED";
        break;
      }

      driveTowardTarget();
      break;

    case AutoState::SETTLE:
      stopMotors();

      if (
        nowMs - autoStateStartedMs >= AUTO_SETTLE_MIN_MS &&
        captureReady
      ) {
        if (requestCapture(false)) {
          autoState = AutoState::CAPTURE_WAIT;
          autoStateStartedMs = nowMs;
          autoMessage = "CAPTURE_SENT_STABLE";
          lastSystemEvent = "CAPTURE_SENT_STABLE";
        } else {
          stopAutonomy("CAPTURE_REQUEST_REJECTED", true);
        }
      } else if (
        nowMs - autoStateStartedMs >= AUTO_SETTLE_FORCE_CAPTURE_MS
      ) {
        if (requestCapture(true)) {
          autoState = AutoState::CAPTURE_WAIT;
          autoStateStartedMs = nowMs;
          autoMessage = "CAPTURE_SENT_FALLBACK";
          lastSystemEvent = "CAPTURE_SENT_FALLBACK";
        } else {
          stopAutonomy("FORCED_CAPTURE_REJECTED", true);
        }
      }
      break;

    case AutoState::CAPTURE_WAIT:
      stopMotors();

      if (lastCaptureResult == 1) {
        autoState = AutoState::COMPLETE;
        autoStateStartedMs = nowMs;
        autoMessage = "WAYPOINT_CAPTURED";
        lastSystemEvent = "WAYPOINT_CAPTURED";
        autoRunStartedMs = 0;
        Serial.println("AUTO COMPLETE: waypoint reached and image saved");
      } else if (lastCaptureResult < 0) {
        stopAutonomy("CAPTURE_FAILED", true);
      } else if (nowMs - autoStateStartedMs > AUTO_CAPTURE_TIMEOUT_MS) {
        stopAutonomy("CAPTURE_TIMEOUT", true);
      }
      break;

    default:
      break;
  }
}

void printNavigationStatus() {
  updateTargetGeometry();

  Serial.println();
  Serial.println("========== NAVIGATION ==========");
  Serial.print("Motors armed: ");
  Serial.println(motorsArmed ? "YES" : "NO");
  Serial.print("Auto state: ");
  Serial.println(autoStateName());
  Serial.print("Message: ");
  Serial.println(autoMessage);
  Serial.print("Motor PWM L/R: ");
  Serial.print(currentLeftUs);
  Serial.print(" / ");
  Serial.println(currentRightUs);
  Serial.print("Target set: ");
  Serial.println(targetSet ? "YES" : "NO");
  if (targetSet) {
    Serial.print("Target: ");
    Serial.print(targetLatitude, 7);
    Serial.print(",");
    Serial.println(targetLongitude, 7);
    Serial.print("Distance: ");
    Serial.print(currentTargetDistanceM, 1);
    Serial.println(" m");
    Serial.print("Bearing: ");
    Serial.print(currentTargetBearingDeg, 1);
    Serial.println(" deg");
    Serial.print("Heading error: ");
    Serial.print(currentHeadingErrorDeg, 1);
    Serial.println(" deg");
  }
  Serial.println("===============================");
  Serial.println();
}

// ============================================================
// Команды Serial Monitor
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("========== COMMANDS ==========");
  Serial.println("M                 - start a new camera mission");
  Serial.println("E                 - end active camera mission");
  Serial.println("C                 - manual photo + telemetry + geo");
  Serial.println("F                 - forced bench photo");
  Serial.println("P / S / G / R     - camera ping/status, GPS, MPU recalibrate");
  Serial.println("ARM               - enable motor output after 12 s neutral delay");
  Serial.println("DISARM            - stop and disable motors");
  Serial.println("STOP or ABORT     - immediate stop/autonomy abort");
  Serial.println("TESTF             - 1.2 s slow straight test at 1380 us");
  Serial.println("TESTM             - 1.2 s medium straight test at 1300 us");
  Serial.println("TEST1200          - 1.2 s stronger straight test at 1200 us");
  Serial.println("TESTL / TESTR     - conservative differential turn tests");
  Serial.println("TARGETHERE        - average current GPS position for 10 s");
  Serial.println("TARGET,lat,lon    - set one GPS waypoint");
  Serial.println("AUTO              - navigate, stop, stabilize and capture");
  Serial.println("NAV               - detailed navigation status");
  Serial.println("H                 - show this help");
  Serial.println("Serial Monitor line ending: Newline");
  Serial.println("==============================");
  Serial.println();
}

String usbInputLine;

void processUsbCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  String upper = command;
  upper.toUpperCase();

  if (upper == "M") {
    requestMissionStart();
  } else if (upper == "E") {
    requestMissionEnd();
  } else if (upper == "T") {
    requestMissionStatus();
  } else if (upper == "C") {
    requestCapture(false);
  } else if (upper == "F") {
    requestCapture(true);
  } else if (upper == "P") {
    sendPing();
  } else if (upper == "S") {
    requestCameraStatus();
  } else if (upper == "G") {
    printGpsStatus();
  } else if (upper == "R") {
    calibrateRobotPosition();
  } else if (upper == "H") {
    printHelp();
  } else if (upper == "ARM") {
    armMotors();
  } else if (upper == "DISARM") {
    disarmMotors();
  } else if (upper == "STOP" || upper == "ABORT") {
    motorTestStopAtMs = 0;
    stopAutonomy("USER_ABORT", false);
  } else if (upper == "TESTF") {
    startTimedMotorTest(1380, 1380);
  } else if (upper == "TESTM") {
    startTimedMotorTest(1300, 1300);
  } else if (upper == "TEST1200") {
    startTimedMotorTest(1200, 1200);
  } else if (upper == "TESTL") {
    startTimedMotorTest(1430, 1320);
  } else if (upper == "TESTR") {
    startTimedMotorTest(1320, 1430);
  } else if (upper == "TARGETHERE") {
    startTargetAveraging();
  } else if (upper == "AUTO") {
    startSingleWaypointAuto();
  } else if (upper == "NAV") {
    printNavigationStatus();
  } else if (upper.startsWith("TARGET,")) {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);

    if (firstComma < 0 || secondComma < 0) {
      Serial.println("TARGET FORMAT: TARGET,42.1234567,77.1234567");
      return;
    }

    double latitude = command.substring(firstComma + 1, secondComma).toDouble();
    double longitude = command.substring(secondComma + 1).toDouble();
    setTarget(latitude, longitude);
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    printHelp();
  }
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

    if (usbInputLine.length() < 120) {
      usbInputLine += incoming;
    } else {
      usbInputLine = "";
      Serial.println("USB COMMAND ERROR: line too long");
    }
  }
}

void manualMotorCommand(const String& command) {
  if (!motorsArmed) {
    stopMotors();
    return;
  }

  if (autoState != AutoState::IDLE && autoState != AutoState::COMPLETE) {
    stopAutonomy("MANUAL_OVERRIDE", false);
  }

  if (command == "0") {
    manualDriveActive = false;
    stopMotors();
    return;
  }

  manualDriveActive = true;
  lastManualCommandMs = millis();

  // Conservative recovery/manual values for the first water test.
  if (command == "f") writeMotors(1350, 1350);
  else if (command == "l") writeMotors(1430, 1320);
  else if (command == "r") writeMotors(1320, 1430);
  else if (command == "b") writeMotors(1750, 1750);
  else {
    manualDriveActive = false;
    stopMotors();
  }
}

void updateManualDriveTimeout() {
  if (
    manualDriveActive &&
    millis() - lastManualCommandMs > MANUAL_COMMAND_TIMEOUT_MS
  ) {
    manualDriveActive = false;
    stopMotors();
    Serial.println("MANUAL LINK TIMEOUT -> STOP");
  }
}

String controlPage() {
  return R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>
html,body{overscroll-behavior:none;-webkit-touch-callout:none}
body{font-family:Arial;background:#101418;color:#fff;text-align:center;margin:0;padding:12px;-webkit-user-select:none;user-select:none}
.card{background:#1d2730;border-radius:12px;padding:12px;margin:10px auto;max-width:700px}
button,input{font-size:18px;padding:13px;margin:5px;border:0;border-radius:10px}
button{min-width:120px;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
button.hold{touch-action:none}.go{background:#29a36a;color:#fff}.stop{background:#d62828;color:#fff}
.manual{background:#74c0fc}.mode{background:#f0b429}.safe{background:#6c757d;color:#fff}
input{width:260px;-webkit-user-select:text;user-select:text}.mono{font-family:monospace;text-align:left;white-space:pre-wrap}
</style>
</head>
<body>
<h2>Catamaran Control</h2>
<div class="card">
<button class="safe" onclick="cmd('arm')">ARM</button>
<button class="safe" onclick="cmd('disarm')">DISARM</button>
<button class="stop" onclick="cmd('stop')">EMERGENCY STOP</button>
</div>
<div class="card">
<h3>Manual hold control</h3>
<button class="manual hold" data-m="f">FORWARD</button><br>
<button class="manual hold" data-m="l">LEFT</button>
<button class="stop" onclick="manualStop()">STOP</button>
<button class="manual hold" data-m="r">RIGHT</button><br>
<button class="manual hold" data-m="b">REVERSE</button>
</div>
<div class="card">
<h3>Motor calibration (1.2 s)</h3>
<button class="mode" onclick="cmd('testslow')">1380 SLOW</button>
<button class="mode" onclick="cmd('testmedium')">1300 MEDIUM</button>
<button class="mode" onclick="cmd('test1200')">1200 STRONG</button><br>
<button class="mode" onclick="cmd('testleft')">TURN LEFT</button>
<button class="mode" onclick="cmd('testright')">TURN RIGHT</button>
</div>
<div class="card">
<h3>Right motor balance</h3>
<p>Positive trim weakens the stronger right motor.</p>
<button class="mode" onclick="cmd('trimminus')">-5 us</button>
<button class="mode" onclick="cmd('trimreset')">RESET 20 us</button>
<button class="mode" onclick="cmd('trimplus')">+5 us</button>
</div>
<div class="card">
<h3>Target and water level</h3>
<button class="mode" onclick="cmd('levelzero')">SET CURRENT WATER LEVEL ZERO</button><br>
<button class="mode" onclick="cmd('targethere')">AVERAGE TARGET HERE (10 s)</button><br>
<input id="lat" placeholder="latitude">
<input id="lon" placeholder="longitude"><br>
<button class="mode" onclick="setTarget()">SET COORDINATES</button>
</div>
<div class="card">
<h3>Mission</h3>
<button class="mode" onclick="cmd('missionstart')">START PHOTO MISSION</button>
<button class="mode" onclick="cmd('capturetest')">FORCE TEST PHOTO</button><br>
<button class="go" onclick="cmd('auto')">START SINGLE-WAYPOINT AUTO</button>
<button class="mode" onclick="cmd('camerastatus')">REFRESH CAMERA STATUS</button>
<button class="mode" onclick="cmd('missionend')">END MISSION</button>
</div>
<div class="card mono" id="status">Loading...</div>
<script>
let timer=null;
async function cmd(c){try{await fetch('/cmd?c='+encodeURIComponent(c),{cache:'no-store'})}catch(e){}setTimeout(update,150)}
async function sendManual(m){try{await fetch('/manual?m='+m,{cache:'no-store'})}catch(e){}}
function manualStart(m){manualStop(false);sendManual(m);timer=setInterval(()=>sendManual(m),200)}
function manualStop(send=true){if(timer){clearInterval(timer);timer=null}if(send)sendManual('0')}
document.querySelectorAll('.hold').forEach(b=>{
 b.addEventListener('pointerdown',e=>{
  e.preventDefault();
  try{b.setPointerCapture(e.pointerId)}catch(x){}
  manualStart(b.dataset.m);
 });
 b.addEventListener('pointerup',e=>{e.preventDefault();manualStop()});
 b.addEventListener('pointercancel',e=>{e.preventDefault();manualStop()});
 b.addEventListener('lostpointercapture',()=>manualStop());
 b.addEventListener('contextmenu',e=>e.preventDefault());
});
document.addEventListener('selectstart',e=>{if(e.target.closest('button'))e.preventDefault()});
window.addEventListener('blur',()=>manualStop());
window.addEventListener('pagehide',()=>manualStop());
async function setTarget(){
 const lat=document.getElementById('lat').value;
 const lon=document.getElementById('lon').value;
 await fetch('/target?lat='+encodeURIComponent(lat)+'&lon='+encodeURIComponent(lon));
 update();
}
async function update(){
 try{
  const s=await (await fetch('/status')).json();
  document.getElementById('status').textContent=
   'GPS: '+s.gps+'  SAT: '+s.sat+'  HDOP: '+s.hdop+'\n'+
   'LAT: '+s.lat+'\nLON: '+s.lon+'\n'+
   'SPEED: '+s.speed+' km/h  COURSE: '+s.course+' deg\n'+
   'CAM LINK: '+s.camera+'  CAM READY: '+s.cameraReady+'  SD: '+s.sd+'\n'+
   'MISSION: '+s.mission+'  CAPTURE PENDING: '+s.capturePending+'\n'+
   'MOTORS: '+s.motors+'  L/R: '+s.left+'/'+s.right+'\n'+
   'RIGHT TRIM: '+s.rightTrim+' us\n'+
   'AUTO: '+s.auto+'  MESSAGE: '+s.autoMessage+'\n'+
   'DIST: '+s.distance+' m  BEARING: '+s.bearing+' deg  ERROR: '+s.headingError+' deg\n'+
   'ROLL/PITCH: '+s.roll+' / '+s.pitch+'  GYRO Z: '+s.gyroZ+'\n'+
   'STABLE: '+s.stable+'  LEVEL: '+s.level+'  CAPTURE READY: '+s.captureReady+'\n'+
   'TARGET: '+s.targetLat+', '+s.targetLon+'\n'+
   'EVENT: '+s.event+'\n'+
   'TARGET AVERAGING: '+s.averaging+' ('+s.avgSamples+' samples)';
 }catch(e){document.getElementById('status').textContent='Connection lost';}
}
setInterval(update,1000);update();
</script>
</body>
</html>
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

  String json = "{";
  json += "\"gps\":\"" + gpsState + "\",";
  json += "\"sat\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  json += "\"hdop\":" + jsonNumber(gps.hdop.isValid() ? gps.hdop.hdop() : 99.99, 2) + ",";
  json += "\"lat\":" + jsonNumber(gps.location.isValid() ? gps.location.lat() : 0.0, 7) + ",";
  json += "\"lon\":" + jsonNumber(gps.location.isValid() ? gps.location.lng() : 0.0, 7) + ",";
  json += "\"speed\":" + jsonNumber(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2) + ",";
  json += "\"course\":" + jsonNumber(gps.course.isValid() ? gps.course.deg() : 0.0, 1) + ",";
  json += "\"camera\":\"" + String(cameraOnline ? "ONLINE" : "OFFLINE") + "\",";
  json += "\"cameraReady\":\"" + String(cameraRemoteReady ? "YES" : "NO") + "\",";
  json += "\"sd\":\"" + String(cameraSdReady ? "READY" : "NOT_READY") + "\",";
  json += "\"mission\":\"" + missionState + "\",";
  json += "\"capturePending\":" + String(captureRequestPending ? "true" : "false") + ",";
  json += "\"motors\":\"" + String(motorsArmed ? "ARMED" : "SAFE") + "\",";
  json += "\"left\":" + String(currentLeftUs) + ",";
  json += "\"right\":" + String(currentRightUs) + ",";
  json += "\"rightTrim\":" + String(rightForwardTrimUs) + ",";
  json += "\"auto\":\"" + autoStateName() + "\",";
  json += "\"autoMessage\":\"" + autoMessage + "\",";
  json += "\"distance\":" + jsonNumber(currentTargetDistanceM, 1) + ",";
  json += "\"bearing\":" + jsonNumber(currentTargetBearingDeg, 1) + ",";
  json += "\"headingError\":" + jsonNumber(currentHeadingErrorDeg, 1) + ",";
  json += "\"roll\":" + jsonNumber(relativeRollDegrees, 1) + ",";
  json += "\"pitch\":" + jsonNumber(relativePitchDegrees, 1) + ",";
  json += "\"gyroZ\":" + jsonNumber(currentGyroZ, 1) + ",";
  json += "\"stable\":" + String(stableConfirmed ? "true" : "false") + ",";
  json += "\"level\":" + String(levelEnough ? "true" : "false") + ",";
  json += "\"captureReady\":" + String(captureReady ? "true" : "false") + ",";
  json += "\"event\":\"" + lastSystemEvent + "\",";
  json += "\"targetLat\":" + jsonNumber(targetSet ? targetLatitude : 0.0, 7) + ",";
  json += "\"targetLon\":" + jsonNumber(targetSet ? targetLongitude : 0.0, 7) + ",";
  json += "\"averaging\":" + String(targetAveraging ? "true" : "false") + ",";
  json += "\"avgSamples\":" + String(targetAverageSamples);
  json += "}";
  controlServer.send(200, "application/json", json);
}

void handleControlCommand() {
  String command = controlServer.arg("c");
  command.toLowerCase();

  if (command == "arm") armMotors();
  else if (command == "disarm") disarmMotors();
  else if (command == "stop") {
    motorTestStopAtMs = 0;
    manualDriveActive = false;
    stopAutonomy("WEB_EMERGENCY_STOP", false);
  }
  else if (command == "testslow") startTimedMotorTest(1380, 1380);
  else if (command == "testmedium") startTimedMotorTest(1300, 1300);
  else if (command == "test1200") startTimedMotorTest(1200, 1200);
  else if (command == "testleft") startTimedMotorTest(1430, 1320);
  else if (command == "testright") startTimedMotorTest(1320, 1430);
  else if (command == "trimminus") changeRightMotorTrim(-5);
  else if (command == "trimplus") changeRightMotorTrim(5);
  else if (command == "trimreset") {
    rightForwardTrimUs = RIGHT_FORWARD_TRIM_DEFAULT_US;
    saveMotorTrimToNvs();
    lastSystemEvent = "RIGHT_TRIM_RESET";
  }
  else if (command == "levelzero") setCurrentWaterLevelZero();
  else if (command == "targethere") startTargetAveraging();
  else if (command == "missionstart") requestMissionStart();
  else if (command == "capturetest") requestCapture(true);
  else if (command == "camerastatus") requestCameraStatus();
  else if (command == "missionend") requestMissionEnd();
  else if (command == "auto") startSingleWaypointAuto();

  controlServer.send(200, "text/plain", "OK");
}

void handleSetTarget() {
  double latitude = controlServer.arg("lat").toDouble();
  double longitude = controlServer.arg("lon").toDouble();
  setTarget(latitude, longitude);
  controlServer.send(200, "text/plain", "OK");
}

void startControlWebServer() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONTROL_AP_NAME, CONTROL_AP_PASS, 6, false, 4);

  controlServer.on("/", []() {
    controlServer.send(200, "text/html", controlPage());
  });
  controlServer.on("/status", handleControlStatus);
  controlServer.on("/cmd", handleControlCommand);
  controlServer.on("/target", handleSetTarget);
  controlServer.on("/manual", []() {
    manualMotorCommand(controlServer.arg("m"));
    controlServer.send(200, "text/plain", "OK");
  });
  controlServer.begin();

  Serial.print("CONTROL WIFI: ");
  Serial.println(CONTROL_AP_NAME);
  Serial.print("CONTROL URL: http://");
  Serial.println(WiFi.softAPIP());
}

void printSystemState() {
  uint32_t nowMs = millis();

  if (
    nowMs - previousPrintMs <
      PRINT_INTERVAL_MS
  ) {
    return;
  }

  previousPrintMs = nowMs;

  Serial.print("ROLL:");
  Serial.print(relativeRollDegrees, 2);

  Serial.print(" PITCH:");
  Serial.print(relativePitchDegrees, 2);

  Serial.print(" | GYRO:");
  Serial.print(currentGyroX, 2);
  Serial.print(",");
  Serial.print(currentGyroY, 2);
  Serial.print(",");
  Serial.print(currentGyroZ, 2);

  Serial.print(" | ACC:");
  Serial.print(currentTotalAcceleration, 3);
  Serial.print("g");

  Serial.print(" | STABLE:");
  Serial.print(stableConfirmed ? "YES" : "NO");

  Serial.print(" | LEVEL:");
  Serial.print(levelEnough ? "YES" : "NO");

  Serial.print(" | CAPTURE_READY:");
  Serial.print(captureReady ? "YES" : "NO");

  Serial.print(" | CAMERA:");
  Serial.print(cameraOnline ? "ONLINE" : "OFFLINE");

  Serial.print(" | GPS:");
  if (!gpsHasData()) {
    Serial.print("NO_DATA");
  } else if (!gpsHasFreshFix()) {
    Serial.print("SEARCH");
  } else {
    Serial.print("FIX");
    Serial.print(" SAT:");
    if (gps.satellites.isValid()) {
      Serial.print(gps.satellites.value());
    } else {
      Serial.print("?");
    }

    Serial.print(" LAT:");
    Serial.print(gps.location.lat(), 6);

    Serial.print(" LON:");
    Serial.print(gps.location.lng(), 6);
  }

  Serial.print(" | MISSION:");
  if (missionActive) {
    Serial.print("M");
    if (currentMissionNumber < 1000) Serial.print("0");
    if (currentMissionNumber < 100) Serial.print("0");
    if (currentMissionNumber < 10) Serial.print("0");
    Serial.print(currentMissionNumber);
  } else {
    Serial.print("NONE");
  }

  Serial.print(" | MOTORS:");
  Serial.print(motorsArmed ? "ARMED" : "SAFE");
  Serial.print("[");
  Serial.print(currentLeftUs);
  Serial.print(",");
  Serial.print(currentRightUs);
  Serial.print("]");

  Serial.print(" | AUTO:");
  Serial.print(autoStateName());

  if (targetSet && currentTargetDistanceM >= 0.0) {
    Serial.print(" DIST:");
    Serial.print(currentTargetDistanceM, 1);
    Serial.print("m");
  }

  Serial.print(" | TEMP:");
  Serial.print(currentTemperature, 1);
  Serial.println("C");
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
  Serial.println("AUTONOMY V4.4: GENTLE GPS TURN + SAFE MOTOR LIMIT + WATER PHOTO");
  Serial.println("======================================");

  bootStartedMs = millis();
  preferences.begin("catamaran", false);
  loadTargetFromNvs();
  loadMotorTrimFromNvs();

  Serial.print("Loaded right forward trim: ");
  Serial.print(rightForwardTrimUs);
  Serial.println(" us");

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
  updateCameraUart();
  checkUsbSerialCommands();
  controlServer.handleClient();
  updateTargetAveraging();
  updateManualDriveTimeout();
  updateAutopilot();
  printSystemState();

  delay(1);
}
