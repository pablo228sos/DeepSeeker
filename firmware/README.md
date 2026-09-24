# Firmware

## Final field snapshots

### Main controller

`main-controller/main_esp32_autonomy_v6_0_interlaced_teardrop/`

Target: classic ESP32 development board.

External Arduino libraries:

- TinyGPSPlus
- ESP32Servo

The ESP32 Arduino core supplies Wi-Fi, WebServer, Preferences, Wire, and HardwareSerial.

### Camera controller

`camera-controller/esp32_s3_camera_mission_v2_2_sd_retry/`

Target: GOOUUU ESP32-S3-CAM V1.5 with OV3660 and PSRAM.

The sketch uses the camera, FS, SD_MMC, Wi-Fi, and WebServer components supplied by the ESP32 Arduino core. Board pin definitions are specific to the tested camera module.

### Manual 10 x 10 m field mode

`field-mode/main_esp32_10x10_manual_orthophoto_v1_0/`

This variant was prepared for deliberate parallel-lane image collection when a full autonomous area mission was too risky for the available test window.

## Upload order

1. Remove or mechanically isolate propellers.
2. Upload the camera sketch to the ESP32-S3-CAM.
3. Confirm camera initialization, PSRAM, SD mounting, and a saved test image.
4. Upload the main-controller sketch to the ESP32.
5. Verify UART communication and camera-ready status.
6. Confirm ESC neutral before allowing any motor command.

## Historical credentials

The preserved field snapshots use the test password `12345678`. This is part of the historical record, not a secure deployment default. Change it before reuse.

## Version archive

`archive/` contains the sequence of field iterations. These are evidence of development and may contain incomplete or experimental behavior. Use the final snapshots above unless reproducing a specific trial.
