#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>

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
uint32_t lastCameraRxMs = 0;
uint32_t lastPingMs = 0;
uint32_t nextCaptureRequestId = 1;

bool missionActive = false;
uint32_t currentMissionNumber = 0;
String currentMissionPath = "";


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

constexpr float MAX_STABLE_GYRO_X = 2.0f;
constexpr float MAX_STABLE_GYRO_Y = 2.0f;
constexpr float MAX_STABLE_GYRO_Z = 3.0f;

constexpr float MIN_STABLE_ACCEL = 0.88f;
constexpr float MAX_STABLE_ACCEL = 1.12f;

constexpr uint32_t REQUIRED_STABLE_TIME_MS = 400;

constexpr float MAX_CAPTURE_ROLL = 10.0f;
constexpr float MAX_CAPTURE_PITCH = 10.0f;

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

void requestCapture(bool forceCapture) {
  if (!cameraOnline) {
    Serial.println("CAPTURE BLOCKED: camera is offline");
    return;
  }

  if (!missionActive) {
    Serial.println("CAPTURE BLOCKED: start a mission with M first");
    return;
  }

  if (!forceCapture && !captureReady) {
    Serial.println("CAPTURE BLOCKED: robot is not stable/level");
    return;
  }

  if (!forceCapture && !gpsHasFreshFix()) {
    Serial.println("CAPTURE BLOCKED: GPS fix is not fresh");
    return;
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

  Serial.print("Capture request ID: ");
  Serial.println(requestId);

  if (forceCapture) {
    Serial.println("Forced mission capture requested");
  }
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
    requestMissionStatus();
    return;
  }

  if (response.startsWith("MISSION_OK,")) {
    currentMissionNumber = csvField(response, 1).toInt();
    currentMissionPath = csvField(response, 2);
    missionActive = true;

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
    Serial.println("MISSION COMMAND FAILED");
    return;
  }

  if (response.startsWith("STATUS,")) {
    int missionPos = response.indexOf("MISSION_ACTIVE=");
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
    Serial.println("PHOTO + TELEMETRY SAVED SUCCESSFULLY");
    Serial.print("Saved path: ");
    Serial.println(csvField(response, 2));
    return;
  }

  if (response.startsWith("CAPTURE_PARTIAL,")) {
    Serial.println("PHOTO SAVED, BUT TELEMETRY WRITE FAILED");
    return;
  }

  if (response.startsWith("CAPTURE_ERROR,")) {
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
// Команды Serial Monitor
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("========== COMMANDS ==========");
  Serial.println("M - start a new mission");
  Serial.println("E - end active mission");
  Serial.println("T - show mission status");
  Serial.println("C - save photo + telemetry when ready");
  Serial.println("F - force photo + telemetry bench test");
  Serial.println("P - camera PING");
  Serial.println("S - camera hardware STATUS");
  Serial.println("G - detailed GPS status");
  Serial.println("R - recalibrate MPU and level zero");
  Serial.println("H - show this help");
  Serial.println("==============================");
  Serial.println();
}

void checkUsbSerialCommands() {
  while (Serial.available() > 0) {
    char command = static_cast<char>(Serial.read());

    if (command == '\r' || command == '\n') {
      continue;
    }

    switch (command) {
      case 'M':
      case 'm':
        requestMissionStart();
        break;

      case 'E':
      case 'e':
        requestMissionEnd();
        break;

      case 'T':
      case 't':
        requestMissionStatus();
        break;

      case 'C':
      case 'c':
        requestCapture(false);
        break;

      case 'F':
      case 'f':
        requestCapture(true);
        break;

      case 'P':
      case 'p':
        sendPing();
        break;

      case 'S':
      case 's':
        requestCameraStatus();
        break;

      case 'G':
      case 'g':
        printGpsStatus();
        break;

      case 'R':
      case 'r':
        calibrateRobotPosition();
        break;

      case 'H':
      case 'h':
        printHelp();
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(command);
        printHelp();
        break;
    }
  }
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
  Serial.println("MPU + GPS + CAMERA MISSIONS V3");
  Serial.println("======================================");

  cameraInputLine.reserve(720);

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
  printSystemState();

  delay(1);
}
