#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ============================================================
// Wi-Fi
// ============================================================

const char* WIFI_NAME = "CATAMARAN-CAM";
const char* WIFI_PASSWORD = "12345678";

WebServer server(80);

// ============================================================
// UART-связь с основной ESP32
// Основная ESP32 GPIO14 TX -> S3 GPIO1 RX
// Основная ESP32 GPIO13 RX <- S3 GPIO2 TX
// ============================================================

constexpr uint32_t CONTROLLER_UART_BAUD = 115200;
constexpr int CONTROLLER_UART_RX_PIN = 1;
constexpr int CONTROLLER_UART_TX_PIN = 2;

HardwareSerial ControllerSerial(1);
String controllerInputLine;

// ============================================================
// Настройки изображения
// ============================================================

// 0 или 1
#define CAMERA_VERTICAL_FLIP      0
#define CAMERA_HORIZONTAL_MIRROR  0

// Пока тестируем в 1024x768.
// После успешных 100/100 можно заменить на FRAMESIZE_QXGA.
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_QXGA;

// Чем меньше число, тем выше качество JPEG.
// Рабочий диапазон обычно 8–20.
const int CAMERA_JPEG_QUALITY = 10;

// ============================================================
// GOOUUU ESP32-S3-CAM V1.5 + OV3660
// Рабочая распиновка камеры
// ============================================================

#define CAM_PWDN   -1
#define CAM_RESET  -1

#define CAM_XCLK   15
#define CAM_SIOD    4
#define CAM_SIOC    5

#define CAM_D0     11
#define CAM_D1      9
#define CAM_D2      8
#define CAM_D3     10
#define CAM_D4     12
#define CAM_D5     18
#define CAM_D6     17
#define CAM_D7     16

#define CAM_VSYNC   6
#define CAM_HREF    7
#define CAM_PCLK   13

// ============================================================
// Встроенная microSD, SD_MMC 1-bit
// ============================================================

#define SD_CLK 39
#define SD_CMD 38
#define SD_D0  40

// ============================================================
// Глобальное состояние
// ============================================================

bool cameraReady = false;
bool sdReady = false;

constexpr uint32_t SD_RETRY_INTERVAL_MS = 3000;
uint32_t lastSdRetryMs = 0;

uint32_t photoNumber = 1;

// ============================================================
// Mission storage
// ============================================================

const char* MISSIONS_ROOT = "/missions";
const char* MISSION_COUNTER_FILE = "/mission_counter.txt";
const char* MISSION_STATE_FILE = "/mission_state.txt";

bool missionActive = false;
uint32_t currentMissionNumber = 0;
uint32_t missionPhotoNumber = 1;
String currentMissionPath = "";


// ============================================================
// Состояние стресс-теста
// ============================================================

struct StressTestState {
  bool active = false;

  uint32_t targetPhotos = 100;
  uint32_t attempted = 0;
  uint32_t successful = 0;
  uint32_t failed = 0;

  uint32_t intervalMs = 1000;
  uint32_t lastCaptureAt = 0;
  uint32_t startedAt = 0;

  String lastFile = "";
  String lastError = "";
};

StressTestState stressTest;

// ============================================================
// Инициализация камеры
// ============================================================

bool initCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = CAM_D0;
  config.pin_d1 = CAM_D1;
  config.pin_d2 = CAM_D2;
  config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4;
  config.pin_d5 = CAM_D5;
  config.pin_d6 = CAM_D6;
  config.pin_d7 = CAM_D7;

  config.pin_xclk = CAM_XCLK;
  config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href = CAM_HREF;

  config.pin_sccb_sda = CAM_SIOD;
  config.pin_sccb_scl = CAM_SIOC;

  config.pin_pwdn = CAM_PWDN;
  config.pin_reset = CAM_RESET;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM found");

    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("WARNING: PSRAM not found");
    Serial.println("Using safe low-resolution mode");

    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t error = esp_camera_init(&config);

  if (error != ESP_OK) {
    Serial.printf(
      "Camera initialization failed: 0x%X\n",
      error
    );

    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();

  if (sensor == nullptr) {
    Serial.println("Camera sensor pointer is null");
    return false;
  }

  sensor->set_vflip(
    sensor,
    CAMERA_VERTICAL_FLIP
  );

  sensor->set_hmirror(
    sensor,
    CAMERA_HORIZONTAL_MIRROR
  );

  sensor->set_brightness(sensor, 0);
  sensor->set_saturation(sensor, 0);
  sensor->set_contrast(sensor, 0);

  Serial.printf(
    "Camera sensor PID: 0x%04X\n",
    sensor->id.PID
  );

  // Несколько первых кадров отбрасываем,
  // чтобы настроились экспозиция и баланс белого.
  for (int i = 0; i < 4; i++) {
    camera_fb_t* frame = esp_camera_fb_get();

    if (frame != nullptr) {
      esp_camera_fb_return(frame);
    }

    delay(200);
  }

  Serial.println("Camera initialized successfully");
  return true;
}

// ============================================================
// Инициализация SD
// ============================================================

bool initSD() {
  Serial.println("Initializing SD card...");

  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0)) {
    Serial.println("SD_MMC.setPins failed");
    return false;
  }

  // true = однобитный режим
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD card mount failed");
    return false;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card detected");
    return false;
  }

  uint64_t cardSizeMB =
    SD_MMC.cardSize() / (1024ULL * 1024ULL);

  uint64_t totalMB =
    SD_MMC.totalBytes() / (1024ULL * 1024ULL);

  uint64_t usedMB =
    SD_MMC.usedBytes() / (1024ULL * 1024ULL);

  Serial.printf("SD card size: %llu MB\n", cardSizeMB);
  Serial.printf("SD total: %llu MB\n", totalMB);
  Serial.printf("SD used: %llu MB\n", usedMB);

  if (!SD_MMC.exists("/photos")) {
    if (!SD_MMC.mkdir("/photos")) {
      Serial.println("Failed to create /photos");
      return false;
    }
  }

  // Ищем следующий свободный номер.
  while (photoNumber < 1000000) {
    char path[48];

    snprintf(
      path,
      sizeof(path),
      "/photos/IMG_%06lu.JPG",
      static_cast<unsigned long>(photoNumber)
    );

    if (!SD_MMC.exists(path)) {
      break;
    }

    photoNumber++;
  }

  Serial.printf(
    "Next photo number: %lu\n",
    static_cast<unsigned long>(photoNumber)
  );

  Serial.println("SD initialized successfully");
  return true;
}

bool retrySdMountBasic() {
  if (sdReady) {
    return true;
  }

  Serial.println("Retrying SD card mount...");
  SD_MMC.end();
  delay(80);

  sdReady = initSD();

  if (sdReady) {
    Serial.println("SD card is now READY");
  }

  return sdReady;
}


// ============================================================
// Сохранение одного снимка
// ============================================================

bool savePhoto(
  String& savedPath,
  size_t& savedBytes,
  uint32_t& captureTimeMs,
  String& errorMessage
) {
  savedPath = "";
  savedBytes = 0;
  captureTimeMs = 0;
  errorMessage = "";

  if (!cameraReady) {
    errorMessage = "Camera is not ready";
    return false;
  }

  if (!sdReady) {
    errorMessage = "SD card is not ready";
    return false;
  }

  uint32_t startedAt = millis();

  camera_fb_t* frame = esp_camera_fb_get();

  if (frame == nullptr) {
    errorMessage = "Camera capture failed";
    return false;
  }

  if (frame->format != PIXFORMAT_JPEG) {
    errorMessage = "Frame is not JPEG";
    esp_camera_fb_return(frame);
    return false;
  }

  const size_t frameLength = frame->len;
  const uint16_t frameWidth = frame->width;
  const uint16_t frameHeight = frame->height;

  char path[48];

  snprintf(
    path,
    sizeof(path),
    "/photos/IMG_%06lu.JPG",
    static_cast<unsigned long>(photoNumber)
  );

  File file = SD_MMC.open(path, FILE_WRITE);

  if (!file) {
    errorMessage = "Could not create file";
    esp_camera_fb_return(frame);
    return false;
  }

  size_t written = file.write(
    frame->buf,
    frame->len
  );

  file.flush();
  file.close();

  esp_camera_fb_return(frame);

  captureTimeMs = millis() - startedAt;

  if (written != frameLength) {
    errorMessage = "Incomplete SD write";

    Serial.printf(
      "Write error: expected %u, written %u\n",
      static_cast<unsigned int>(frameLength),
      static_cast<unsigned int>(written)
    );

    return false;
  }

  savedPath = path;
  savedBytes = written;

  Serial.printf(
    "Saved: %s | %ux%u | %u bytes | %lu ms\n",
    path,
    frameWidth,
    frameHeight,
    static_cast<unsigned int>(written),
    static_cast<unsigned long>(captureTimeMs)
  );

  photoNumber++;

  return true;
}

// ============================================================
// Mission helpers
// ============================================================

String makeMissionPath(uint32_t missionNumber) {
  char path[32];
  snprintf(
    path,
    sizeof(path),
    "/missions/M%04lu",
    static_cast<unsigned long>(missionNumber)
  );
  return String(path);
}

bool ensureDirectory(const String& path) {
  if (SD_MMC.exists(path)) {
    return true;
  }

  return SD_MMC.mkdir(path);
}

uint32_t readMissionCounter() {
  if (!SD_MMC.exists(MISSION_COUNTER_FILE)) {
    return 0;
  }

  File file = SD_MMC.open(MISSION_COUNTER_FILE, FILE_READ);
  if (!file) {
    return 0;
  }

  uint32_t value = file.parseInt();
  file.close();
  return value;
}

bool writeMissionCounter(uint32_t value) {
  File file = SD_MMC.open(MISSION_COUNTER_FILE, FILE_WRITE);
  if (!file) {
    return false;
  }

  file.print(value);
  file.print("\n");
  file.flush();
  file.close();
  return true;
}

bool writeMissionState() {
  File file = SD_MMC.open(MISSION_STATE_FILE, FILE_WRITE);
  if (!file) {
    return false;
  }

  file.print(currentMissionNumber);
  file.print(",");
  file.print(missionPhotoNumber);
  file.print("\n");
  file.flush();
  file.close();
  return true;
}

void clearMissionState() {
  if (SD_MMC.exists(MISSION_STATE_FILE)) {
    SD_MMC.remove(MISSION_STATE_FILE);
  }
}

bool createGeoFile(const String& missionPath) {
  String geoPath = missionPath + "/geo.txt";

  if (SD_MMC.exists(geoPath)) {
    return true;
  }

  File file = SD_MMC.open(geoPath, FILE_WRITE);
  if (!file) {
    return false;
  }

  // OpenDroneMap image geolocation file. For EPSG:4326:
  // X = longitude, Y = latitude.
  file.println("EPSG:4326");
  file.flush();
  file.close();
  return true;
}

bool createTelemetryFile(const String& missionPath) {
  String csvPath = missionPath + "/telemetry.csv";

  if (SD_MMC.exists(csvPath)) {
    return true;
  }

  File file = SD_MMC.open(csvPath, FILE_WRITE);
  if (!file) {
    return false;
  }

  file.println(
    "image,utc,latitude,longitude,altitude_m,satellites,hdop,"
    "speed_kmph,course_deg,roll_deg,pitch_deg,gyro_x_dps,"
    "gyro_y_dps,gyro_z_dps,total_acc_g,stable,level,jpeg_bytes,capture_ms"
  );

  file.flush();
  file.close();
  return true;
}

bool startNewMission(String& errorMessage) {
  errorMessage = "";

  if (!sdReady && !retrySdMountBasic()) {
    errorMessage = "SD_NOT_READY";
    return false;
  }

  if (missionActive) {
    errorMessage = "MISSION_ALREADY_ACTIVE";
    return false;
  }

  if (!ensureDirectory(MISSIONS_ROOT)) {
    errorMessage = "MISSIONS_ROOT_CREATE_FAILED";
    return false;
  }

  uint32_t candidate = readMissionCounter() + 1;

  while (candidate < 10000) {
    String candidatePath = makeMissionPath(candidate);
    if (!SD_MMC.exists(candidatePath)) {
      break;
    }
    candidate++;
  }

  if (candidate >= 10000) {
    errorMessage = "MISSION_NUMBER_LIMIT";
    return false;
  }

  currentMissionNumber = candidate;
  currentMissionPath = makeMissionPath(currentMissionNumber);
  missionPhotoNumber = 1;

  if (!ensureDirectory(currentMissionPath)) {
    errorMessage = "MISSION_DIR_CREATE_FAILED";
    return false;
  }

  String imagesPath = currentMissionPath + "/images";
  if (!ensureDirectory(imagesPath)) {
    errorMessage = "IMAGES_DIR_CREATE_FAILED";
    return false;
  }

  if (!createTelemetryFile(currentMissionPath)) {
    errorMessage = "TELEMETRY_CREATE_FAILED";
    return false;
  }

  if (!createGeoFile(currentMissionPath)) {
    errorMessage = "GEO_CREATE_FAILED";
    return false;
  }

  missionActive = true;

  if (!writeMissionCounter(currentMissionNumber)) {
    missionActive = false;
    errorMessage = "COUNTER_WRITE_FAILED";
    return false;
  }

  if (!writeMissionState()) {
    missionActive = false;
    errorMessage = "STATE_WRITE_FAILED";
    return false;
  }

  return true;
}

bool resumeMissionFromState() {
  missionActive = false;
  currentMissionNumber = 0;
  missionPhotoNumber = 1;
  currentMissionPath = "";

  if (!sdReady || !SD_MMC.exists(MISSION_STATE_FILE)) {
    return false;
  }

  File file = SD_MMC.open(MISSION_STATE_FILE, FILE_READ);
  if (!file) {
    return false;
  }

  String line = file.readStringUntil('\n');
  file.close();
  line.trim();

  int comma = line.indexOf(',');
  if (comma < 1) {
    clearMissionState();
    return false;
  }

  uint32_t missionNumber = line.substring(0, comma).toInt();
  uint32_t nextPhoto = line.substring(comma + 1).toInt();

  if (missionNumber == 0 || nextPhoto == 0) {
    clearMissionState();
    return false;
  }

  String missionPath = makeMissionPath(missionNumber);
  String imagesPath = missionPath + "/images";
  String csvPath = missionPath + "/telemetry.csv";

  if (
    !SD_MMC.exists(missionPath) ||
    !SD_MMC.exists(imagesPath) ||
    !SD_MMC.exists(csvPath)
  ) {
    clearMissionState();
    return false;
  }

  if (!createGeoFile(missionPath)) {
    clearMissionState();
    return false;
  }

  currentMissionNumber = missionNumber;
  missionPhotoNumber = nextPhoto;
  currentMissionPath = missionPath;
  missionActive = true;
  return true;
}

bool endMission(String& errorMessage) {
  errorMessage = "";

  if (!missionActive) {
    errorMessage = "NO_ACTIVE_MISSION";
    return false;
  }

  clearMissionState();
  missionActive = false;
  currentMissionNumber = 0;
  missionPhotoNumber = 1;
  currentMissionPath = "";
  return true;
}

bool appendMissionTelemetry(
  const String& imagePath,
  const String& telemetryPayload,
  size_t jpegBytes,
  uint32_t captureTimeMs
) {
  if (!missionActive) {
    return false;
  }

  String csvPath = currentMissionPath + "/telemetry.csv";
  File file = SD_MMC.open(csvPath, FILE_APPEND);

  if (!file) {
    return false;
  }

  int slash = imagePath.lastIndexOf('/');
  String imageName =
    slash >= 0 ? imagePath.substring(slash + 1) : imagePath;

  file.print(imageName);
  file.print(",");
  file.print(telemetryPayload);
  file.print(",");
  file.print(static_cast<unsigned int>(jpegBytes));
  file.print(",");
  file.println(captureTimeMs);

  file.flush();
  file.close();
  return true;
}

String getCsvField(const String& text, int fieldIndex) {
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

bool appendMissionGeo(
  const String& imagePath,
  const String& telemetryPayload
) {
  if (!missionActive) {
    return false;
  }

  // telemetryPayload fields sent by Main ESP32:
  // 0 utc, 1 latitude, 2 longitude, 3 altitude, ...
  String latitude = getCsvField(telemetryPayload, 1);
  String longitude = getCsvField(telemetryPayload, 2);
  String altitude = getCsvField(telemetryPayload, 3);

  latitude.trim();
  longitude.trim();
  altitude.trim();

  if (
    latitude.length() == 0 || longitude.length() == 0 ||
    latitude == "NA" || longitude == "NA"
  ) {
    return false;
  }

  if (altitude.length() == 0 || altitude == "NA") {
    altitude = "0";
  }

  int slash = imagePath.lastIndexOf('/');
  String imageName =
    slash >= 0 ? imagePath.substring(slash + 1) : imagePath;

  String geoPath = currentMissionPath + "/geo.txt";
  File file = SD_MMC.open(geoPath, FILE_APPEND);
  if (!file) {
    return false;
  }

  // ODM format for EPSG:4326: image_name longitude latitude altitude
  file.print(imageName);
  file.print(" ");
  file.print(longitude);
  file.print(" ");
  file.print(latitude);
  file.print(" ");
  file.println(altitude);
  file.flush();
  file.close();
  return true;
}

bool saveMissionPhoto(
  String& savedPath,
  size_t& savedBytes,
  uint32_t& captureTimeMs,
  String& errorMessage
) {
  savedPath = "";
  savedBytes = 0;
  captureTimeMs = 0;
  errorMessage = "";

  if (!cameraReady) {
    errorMessage = "CAMERA_NOT_READY";
    return false;
  }

  if (!sdReady) {
    errorMessage = "SD_NOT_READY";
    return false;
  }

  if (!missionActive) {
    errorMessage = "NO_ACTIVE_MISSION";
    return false;
  }

  uint32_t startedAt = millis();
  camera_fb_t* frame = esp_camera_fb_get();

  if (frame == nullptr) {
    errorMessage = "CAMERA_CAPTURE_FAILED";
    return false;
  }

  if (frame->format != PIXFORMAT_JPEG) {
    esp_camera_fb_return(frame);
    errorMessage = "FRAME_NOT_JPEG";
    return false;
  }

  char path[128];
  snprintf(
    path,
    sizeof(path),
    "%s/images/IMG_%06lu.JPG",
    currentMissionPath.c_str(),
    static_cast<unsigned long>(missionPhotoNumber)
  );

  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) {
    esp_camera_fb_return(frame);
    errorMessage = "IMAGE_FILE_CREATE_FAILED";
    return false;
  }

  const size_t frameLength = frame->len;
  size_t written = file.write(frame->buf, frame->len);
  file.flush();
  file.close();
  esp_camera_fb_return(frame);

  captureTimeMs = millis() - startedAt;

  if (written != frameLength) {
    errorMessage = "INCOMPLETE_SD_WRITE";
    return false;
  }

  savedPath = String(path);
  savedBytes = written;
  missionPhotoNumber++;

  if (!writeMissionState()) {
    errorMessage = "STATE_UPDATE_FAILED";
    return false;
  }

  Serial.printf(
    "Mission photo saved: %s | %u bytes | %lu ms\n",
    path,
    static_cast<unsigned int>(written),
    static_cast<unsigned long>(captureTimeMs)
  );

  return true;
}

// ============================================================
// Запуск стресс-теста
// ============================================================

bool startStressTest(
  uint32_t photoCount,
  uint32_t intervalMs
) {
  if (stressTest.active) {
    return false;
  }

  if (!cameraReady || !sdReady) {
    return false;
  }

  stressTest.active = true;

  stressTest.targetPhotos = photoCount;
  stressTest.attempted = 0;
  stressTest.successful = 0;
  stressTest.failed = 0;

  stressTest.intervalMs = intervalMs;
  stressTest.lastCaptureAt = 0;
  stressTest.startedAt = millis();

  stressTest.lastFile = "";
  stressTest.lastError = "";

  Serial.println();
  Serial.println("==================================");
  Serial.println("STRESS TEST STARTED");
  Serial.printf(
    "Target photos: %lu\n",
    static_cast<unsigned long>(photoCount)
  );
  Serial.printf(
    "Interval: %lu ms\n",
    static_cast<unsigned long>(intervalMs)
  );
  Serial.println("==================================");

  return true;
}

// ============================================================
// Остановка стресс-теста
// ============================================================

void stopStressTest(const String& reason) {
  if (!stressTest.active) {
    return;
  }

  stressTest.active = false;

  uint32_t duration =
    millis() - stressTest.startedAt;

  Serial.println();
  Serial.println("==================================");
  Serial.println("STRESS TEST FINISHED");
  Serial.print("Reason: ");
  Serial.println(reason);

  Serial.printf(
    "Attempted: %lu\n",
    static_cast<unsigned long>(stressTest.attempted)
  );

  Serial.printf(
    "Successful: %lu\n",
    static_cast<unsigned long>(stressTest.successful)
  );

  Serial.printf(
    "Failed: %lu\n",
    static_cast<unsigned long>(stressTest.failed)
  );

  Serial.printf(
    "Duration: %lu ms\n",
    static_cast<unsigned long>(duration)
  );

  Serial.println("==================================");
}

// ============================================================
// Неблокирующее выполнение стресс-теста
// ============================================================

void updateStressTest() {
  if (!stressTest.active) {
    return;
  }

  if (stressTest.attempted >= stressTest.targetPhotos) {
    stopStressTest("Target reached");
    return;
  }

  uint32_t now = millis();

  if (
    stressTest.lastCaptureAt != 0 &&
    now - stressTest.lastCaptureAt < stressTest.intervalMs
  ) {
    return;
  }

  stressTest.lastCaptureAt = now;
  stressTest.attempted++;

  String path;
  String errorMessage;

  size_t bytes = 0;
  uint32_t captureTimeMs = 0;

  bool success = savePhoto(
    path,
    bytes,
    captureTimeMs,
    errorMessage
  );

  if (success) {
    stressTest.successful++;
    stressTest.lastFile = path;
    stressTest.lastError = "";

    Serial.printf(
      "[%lu/%lu] OK | %s | %u bytes | %lu ms\n",
      static_cast<unsigned long>(stressTest.attempted),
      static_cast<unsigned long>(stressTest.targetPhotos),
      path.c_str(),
      static_cast<unsigned int>(bytes),
      static_cast<unsigned long>(captureTimeMs)
    );
  } else {
    stressTest.failed++;
    stressTest.lastError = errorMessage;

    Serial.printf(
      "[%lu/%lu] FAILED | %s\n",
      static_cast<unsigned long>(stressTest.attempted),
      static_cast<unsigned long>(stressTest.targetPhotos),
      errorMessage.c_str()
    );
  }

  if (stressTest.attempted >= stressTest.targetPhotos) {
    stopStressTest("Target reached");
  }
}

// ============================================================
// Главная веб-страница
// ============================================================

void handleRoot() {
  String page;

  page.reserve(6000);

  page += R"HTML(
<!DOCTYPE html>
<html lang="ru">
<head>
  <meta charset="UTF-8">

  <meta
    name="viewport"
    content="width=device-width, initial-scale=1"
  >

  <title>Catamaran Camera Test</title>

  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 18px;
      background: #f1f1f1;
    }

    .card {
      max-width: 850px;
      margin: 0 auto 16px auto;
      padding: 16px;
      background: white;
      border-radius: 14px;
      box-shadow: 0 3px 15px rgba(0, 0, 0, 0.12);
    }

    img {
      width: 100%;
      max-width: 800px;
      border-radius: 12px;
      background: #222;
      min-height: 220px;
    }

    button {
      margin: 6px;
      padding: 13px 17px;
      font-size: 15px;
      border: 0;
      border-radius: 10px;
      cursor: pointer;
    }

    .primary {
      background: #1769e0;
      color: white;
    }

    .success {
      background: #16833d;
      color: white;
    }

    .danger {
      background: #c62c2c;
      color: white;
    }

    .status {
      line-height: 1.7;
      font-family: monospace;
      text-align: left;
      white-space: pre-wrap;
    }

    #message {
      min-height: 24px;
      font-weight: bold;
    }
  </style>
</head>

<body>
  <div class="card">
    <h1>Catamaran Camera</h1>
)HTML";

  page += "<p>Camera: <b>";
  page += cameraReady ? "OK" : "ERROR";
  page += "</b></p>";

  page += "<p>SD card: <b>";
  page += sdReady ? "OK" : "ERROR";
  page += "</b></p>";

  page += "<p>PSRAM: <b>";
  page += psramFound() ? "OK" : "NOT FOUND";
  page += "</b></p>";

  page += "<p>PSRAM size: <b>";
  page += String(
    ESP.getPsramSize() / (1024 * 1024)
  );
  page += " MB</b></p>";

  page += R"HTML(
  </div>

  <div class="card">
    <img
      id="cameraImage"
      src="/capture"
      alt="Camera image"
    >

    <div>
      <button
        class="primary"
        onclick="refreshImage()"
      >
        Обновить кадр
      </button>

      <button
        class="success"
        onclick="saveSinglePhoto()"
      >
        Снять и сохранить
      </button>
    </div>

    <p id="message"></p>
  </div>

  <div class="card">
    <h2>Стресс-тест SD и камеры</h2>

    <p>
      100 фотографий, одна фотография в секунду.
    </p>

    <button
      class="success"
      onclick="startStressTest()"
    >
      Запустить 100 фото
    </button>

    <button
      class="danger"
      onclick="stopStressTest()"
    >
      Остановить
    </button>

    <pre
      id="stressStatus"
      class="status"
    >
Загрузка статуса...
    </pre>
  </div>

  <div class="card">
    <a href="/files">
      Посмотреть файлы на SD
    </a>
  </div>

  <script>
    function refreshImage() {
      document.getElementById("cameraImage").src =
        "/capture?t=" + Date.now();
    }

    function saveSinglePhoto() {
      const message =
        document.getElementById("message");

      message.innerText = "Сохраняю...";

      fetch("/save")
        .then(response => response.text())
        .then(text => {
          message.innerHTML = text;
          refreshImage();
        })
        .catch(error => {
          message.innerText =
            "Ошибка запроса: " + error;
        });
    }

    function startStressTest() {
      fetch("/stress/start")
        .then(response => response.text())
        .then(text => {
          document.getElementById(
            "message"
          ).innerText = text;
        });
    }

    function stopStressTest() {
      fetch("/stress/stop")
        .then(response => response.text())
        .then(text => {
          document.getElementById(
            "message"
          ).innerText = text;
        });
    }

    function updateStatus() {
      fetch("/status")
        .then(response => response.json())
        .then(data => {
          let text = "";

          text += "Активен: ";
          text += data.stress_active ? "ДА" : "НЕТ";
          text += "\n";

          text += "Попыток: ";
          text += data.attempted;
          text += " / ";
          text += data.target;
          text += "\n";

          text += "Успешно: ";
          text += data.successful;
          text += "\n";

          text += "Ошибок: ";
          text += data.failed;
          text += "\n";

          text += "Последний файл: ";
          text += data.last_file || "-";
          text += "\n";

          text += "Последняя ошибка: ";
          text += data.last_error || "-";

          document.getElementById(
            "stressStatus"
          ).innerText = text;
        })
        .catch(error => {
          document.getElementById(
            "stressStatus"
          ).innerText =
            "Ошибка получения статуса";
        });
    }

    setInterval(updateStatus, 1000);
    updateStatus();
  </script>
</body>
</html>
)HTML";

  server.send(
    200,
    "text/html; charset=utf-8",
    page
  );
}

// ============================================================
// Отправка одного JPEG в браузер
// ============================================================

void handleCapture() {
  if (!cameraReady) {
    server.send(
      503,
      "text/plain",
      "Camera is not ready"
    );

    return;
  }

  camera_fb_t* frame = esp_camera_fb_get();

  if (frame == nullptr) {
    server.send(
      500,
      "text/plain",
      "Capture failed"
    );

    return;
  }

  server.sendHeader(
    "Cache-Control",
    "no-store, no-cache, must-revalidate"
  );

  server.setContentLength(frame->len);
  server.send(200, "image/jpeg", "");

  WiFiClient client = server.client();

  size_t sent = 0;

  while (sent < frame->len && client.connected()) {
    size_t chunkSize =
      min(
        static_cast<size_t>(4096),
        frame->len - sent
      );

    size_t written = client.write(
      frame->buf + sent,
      chunkSize
    );

    if (written == 0) {
      break;
    }

    sent += written;
    yield();
  }

  esp_camera_fb_return(frame);
}

// ============================================================
// Сохранение одиночного снимка
// ============================================================

void handleSave() {
  String path;
  String errorMessage;

  size_t bytes = 0;
  uint32_t captureTimeMs = 0;

  bool success = savePhoto(
    path,
    bytes,
    captureTimeMs,
    errorMessage
  );

  if (!success) {
    server.send(
      500,
      "text/html; charset=utf-8",
      "Ошибка: " + errorMessage
    );

    return;
  }

  String response = "Сохранено: <b>";
  response += path;
  response += "</b><br>Размер: ";
  response += String(bytes);
  response += " байт<br>Время: ";
  response += String(captureTimeMs);
  response += " мс";

  server.send(
    200,
    "text/html; charset=utf-8",
    response
  );
}

// ============================================================
// Запуск теста через браузер
// ============================================================

void handleStressStart() {
  if (stressTest.active) {
    server.send(
      409,
      "text/plain; charset=utf-8",
      "Стресс-тест уже запущен"
    );

    return;
  }

  if (!cameraReady || !sdReady) {
    server.send(
      503,
      "text/plain; charset=utf-8",
      "Камера или SD не готовы"
    );

    return;
  }

  if (!startStressTest(100, 1000)) {
    server.send(
      500,
      "text/plain; charset=utf-8",
      "Не удалось запустить тест"
    );

    return;
  }

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Стресс-тест запущен"
  );
}

// ============================================================
// Остановка теста через браузер
// ============================================================

void handleStressStop() {
  if (!stressTest.active) {
    server.send(
      200,
      "text/plain; charset=utf-8",
      "Стресс-тест уже остановлен"
    );

    return;
  }

  stopStressTest("Stopped by user");

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Стресс-тест остановлен"
  );
}

// ============================================================
// JSON-статус
// ============================================================

void handleStatus() {
  String json;

  json.reserve(700);

  json += "{";

  json += "\"camera_ready\":";
  json += cameraReady ? "true" : "false";
  json += ",";

  json += "\"sd_ready\":";
  json += sdReady ? "true" : "false";
  json += ",";

  json += "\"psram_mb\":";
  json += String(
    ESP.getPsramSize() / (1024 * 1024)
  );
  json += ",";

  json += "\"stress_active\":";
  json += stressTest.active ? "true" : "false";
  json += ",";

  json += "\"target\":";
  json += String(stressTest.targetPhotos);
  json += ",";

  json += "\"attempted\":";
  json += String(stressTest.attempted);
  json += ",";

  json += "\"successful\":";
  json += String(stressTest.successful);
  json += ",";

  json += "\"failed\":";
  json += String(stressTest.failed);
  json += ",";

  json += "\"last_file\":\"";
  json += stressTest.lastFile;
  json += "\",";

  json += "\"last_error\":\"";
  json += stressTest.lastError;
  json += "\"";

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}

// ============================================================
// Список файлов
// ============================================================

void handleFiles() {
  if (!sdReady) {
    server.send(
      503,
      "text/plain",
      "SD card is not ready"
    );

    return;
  }

  String listPath = missionActive
    ? currentMissionPath + "/images"
    : String("/photos");

  File directory = SD_MMC.open(listPath);

  if (!directory || !directory.isDirectory()) {
    server.send(
      500,
      "text/plain",
      "Could not open " + listPath
    );

    return;
  }

  String page;

  page.reserve(10000);

  page +=
    "<html>"
    "<head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' "
    "content='width=device-width,initial-scale=1'>"
    "<title>SD files</title>"
    "</head>"
    "<body>"
    "<h2>Фотографии на SD</h2>";

  page += "<p>Папка: <b>";
  page += listPath;
  page += "</b></p>";

  File file = directory.openNextFile();

  int count = 0;

  while (file && count < 300) {
    if (!file.isDirectory()) {
      page += "<p>";
      page += file.name();
      page += " — ";
      page += String(file.size());
      page += " байт</p>";

      count++;
    }

    file.close();
    file = directory.openNextFile();
  }

  directory.close();

  if (count == 0) {
    page += "<p>Фотографий пока нет.</p>";
  }

  page += "<p>Показано файлов: ";
  page += String(count);
  page += "</p>";

  page += "<p><a href='/'>Назад</a></p>";
  page += "</body></html>";

  server.send(
    200,
    "text/html; charset=utf-8",
    page
  );
}

// ============================================================
// UART protocol
//
// Main -> camera:
//   PING
//   STATUS
//   MISSION_START
//   MISSION_END
//   MISSION_STATUS
//   CAPTURE,<id>,<utc>,<lat>,<lon>,<alt>,<sat>,<hdop>,<speed>,
//           <course>,<roll>,<pitch>,<gx>,<gy>,<gz>,<acc>,<stable>,<level>
// ============================================================

void sendControllerStatus() {
  String response;
  response.reserve(220);

  response += "STATUS,CAMERA=";
  response += cameraReady ? "1" : "0";
  response += ",SD=";
  response += sdReady ? "1" : "0";
  response += ",PSRAM_MB=";
  response += String(ESP.getPsramSize() / (1024 * 1024));
  response += ",MISSION_ACTIVE=";
  response += missionActive ? "1" : "0";
  response += ",MISSION=";
  response += String(currentMissionNumber);
  response += ",NEXT_MISSION_PHOTO=";
  response += String(missionPhotoNumber);

  ControllerSerial.println(response);
  Serial.print("UART TX: ");
  Serial.println(response);
}

void sendMissionStatus() {
  String response;
  response.reserve(180);

  response += "MISSION_STATUS,ACTIVE=";
  response += missionActive ? "1" : "0";
  response += ",NUMBER=";
  response += String(currentMissionNumber);
  response += ",PATH=";
  response += currentMissionPath.length() ? currentMissionPath : "-";
  response += ",NEXT=";
  response += String(missionPhotoNumber);

  ControllerSerial.println(response);
  Serial.print("UART TX: ");
  Serial.println(response);
}

void processControllerCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  Serial.print("UART RX: ");
  Serial.println(command);

  if (command == "PING") {
    ControllerSerial.println("PONG");
    Serial.println("UART TX: PONG");
    return;
  }

  if (command == "STATUS") {
    sendControllerStatus();
    return;
  }

  if (command == "MISSION_STATUS") {
    sendMissionStatus();
    return;
  }

  if (command == "MISSION_START") {
    String errorMessage;

    if (!startNewMission(errorMessage)) {
      String response = "MISSION_ERROR," + errorMessage;
      ControllerSerial.println(response);
      Serial.print("UART TX: ");
      Serial.println(response);
      return;
    }

    String response = "MISSION_OK,";
    response += String(currentMissionNumber);
    response += ",";
    response += currentMissionPath;

    ControllerSerial.println(response);
    Serial.print("UART TX: ");
    Serial.println(response);
    return;
  }

  if (command == "MISSION_END") {
    uint32_t endedMission = currentMissionNumber;
    String errorMessage;

    if (!endMission(errorMessage)) {
      String response = "MISSION_ERROR," + errorMessage;
      ControllerSerial.println(response);
      Serial.print("UART TX: ");
      Serial.println(response);
      return;
    }

    String response = "MISSION_ENDED," + String(endedMission);
    ControllerSerial.println(response);
    Serial.print("UART TX: ");
    Serial.println(response);
    return;
  }

  if (command.startsWith("CAPTURE,")) {
    int idEnd = command.indexOf(',', 8);

    if (idEnd < 0) {
      ControllerSerial.println("CAPTURE_ERROR,0,BAD_COMMAND_FORMAT");
      return;
    }

    String requestId = command.substring(8, idEnd);
    String telemetryPayload = command.substring(idEnd + 1);
    requestId.trim();
    telemetryPayload.trim();

    if (requestId.length() == 0) {
      requestId = "0";
    }

    if (!missionActive) {
      String response =
        "CAPTURE_ERROR," + requestId + ",NO_ACTIVE_MISSION";
      ControllerSerial.println(response);
      Serial.print("UART TX: ");
      Serial.println(response);
      return;
    }

    if (stressTest.active) {
      String response =
        "CAPTURE_ERROR," + requestId + ",BUSY_STRESS_TEST";
      ControllerSerial.println(response);
      Serial.print("UART TX: ");
      Serial.println(response);
      return;
    }

    String path;
    String errorMessage;
    size_t bytes = 0;
    uint32_t captureTimeMs = 0;

    bool success = saveMissionPhoto(
      path,
      bytes,
      captureTimeMs,
      errorMessage
    );

    if (!success) {
      String response = "CAPTURE_ERROR," + requestId + "," + errorMessage;
      ControllerSerial.println(response);
      Serial.print("UART TX: ");
      Serial.println(response);
      return;
    }

    bool telemetrySaved = appendMissionTelemetry(
      path,
      telemetryPayload,
      bytes,
      captureTimeMs
    );

    bool geoSaved = appendMissionGeo(path, telemetryPayload);

    if (!telemetrySaved || !geoSaved) {
      String response = "CAPTURE_PARTIAL,";
      response += requestId;
      response += ",";
      response += path;
      response += ",";
      if (!telemetrySaved && !geoSaved) {
        response += "TELEMETRY_AND_GEO_WRITE_FAILED";
      } else if (!telemetrySaved) {
        response += "TELEMETRY_WRITE_FAILED";
      } else {
        response += "GEO_WRITE_FAILED";
      }
      ControllerSerial.println(response);
      Serial.print("UART TX: ");
      Serial.println(response);
      return;
    }

    String response;
    response.reserve(200);
    response = "CAPTURE_OK,";
    response += requestId;
    response += ",";
    response += path;
    response += ",";
    response += String(bytes);
    response += ",";
    response += String(captureTimeMs);

    ControllerSerial.println(response);
    Serial.print("UART TX: ");
    Serial.println(response);
    return;
  }

  String response = "ERROR,UNKNOWN_COMMAND," + command;
  ControllerSerial.println(response);
  Serial.print("UART TX: ");
  Serial.println(response);
}

void updateControllerUart() {
  while (ControllerSerial.available() > 0) {
    char incoming =
      static_cast<char>(ControllerSerial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      processControllerCommand(controllerInputLine);
      controllerInputLine = "";
      continue;
    }

    if (controllerInputLine.length() < 620) {
      controllerInputLine += incoming;
    } else {
      controllerInputLine = "";
      ControllerSerial.println("ERROR,LINE_TOO_LONG");
      Serial.println("UART RX error: line too long");
    }
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  ControllerSerial.begin(
    CONTROLLER_UART_BAUD,
    SERIAL_8N1,
    CONTROLLER_UART_RX_PIN,
    CONTROLLER_UART_TX_PIN
  );

  controllerInputLine.reserve(640);

  Serial.println();
  Serial.println("==================================");
  Serial.println("CATAMARAN CAMERA MISSION V2.2 + SD HOT-INSERT RETRY");
  Serial.println("==================================");

  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());

  Serial.print("Flash: ");
  Serial.print(
    ESP.getFlashChipSize() / (1024 * 1024)
  );
  Serial.println(" MB");

  Serial.print("PSRAM detected: ");
  Serial.println(psramFound() ? "YES" : "NO");

  Serial.print("PSRAM size: ");
  Serial.print(
    ESP.getPsramSize() / (1024 * 1024)
  );
  Serial.println(" MB");

  Serial.print("Controller UART RX pin: GPIO");
  Serial.println(CONTROLLER_UART_RX_PIN);

  Serial.print("Controller UART TX pin: GPIO");
  Serial.println(CONTROLLER_UART_TX_PIN);

  cameraReady = initCamera();
  sdReady = initSD();

  if (sdReady && resumeMissionFromState()) {
    Serial.print("Resumed mission: " );
    Serial.print(currentMissionPath);
    Serial.print(" | next photo: " );
    Serial.println(missionPhotoNumber);
  } else {
    Serial.println("No active mission to resume");
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool wifiStarted = WiFi.softAP(
    WIFI_NAME,
    WIFI_PASSWORD,
    1,
    false,
    4
  );

  if (!wifiStarted) {
    Serial.println("Wi-Fi AP start failed");
    return;
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/save", HTTP_GET, handleSave);
  server.on("/files", HTTP_GET, handleFiles);
  server.on("/status", HTTP_GET, handleStatus);

  server.on(
    "/stress/start",
    HTTP_GET,
    handleStressStart
  );

  server.on(
    "/stress/stop",
    HTTP_GET,
    handleStressStop
  );

  server.onNotFound([]() {
    server.send(
      404,
      "text/plain",
      "Not found"
    );
  });

  server.begin();

  Serial.println();
  Serial.println("Wi-Fi started");
  Serial.print("Network: ");
  Serial.println(WIFI_NAME);

  Serial.print("Password: ");
  Serial.println(WIFI_PASSWORD);

  Serial.println("Open: http://192.168.4.1");
  Serial.println("==================================");

  ControllerSerial.println("CAMERA_BOOT");
  sendControllerStatus();
  if (missionActive) {
    String resumed = "MISSION_RESUMED,";
    resumed += String(currentMissionNumber);
    resumed += ",";
    resumed += currentMissionPath;
    resumed += ",";
    resumed += String(missionPhotoNumber);
    ControllerSerial.println(resumed);
    Serial.print("UART TX: " );
    Serial.println(resumed);
  }
}

// ============================================================
// Loop
// ============================================================

void loop() {
  updateControllerUart();
  server.handleClient();
  updateStressTest();

  if (!sdReady && millis() - lastSdRetryMs >= SD_RETRY_INTERVAL_MS) {
    lastSdRetryMs = millis();

    if (retrySdMountBasic()) {
      if (!missionActive && resumeMissionFromState()) {
        Serial.print("Mission resumed after SD retry: ");
        Serial.println(currentMissionPath);
      }

      sendControllerStatus();
    }
  }

  delay(2);
}