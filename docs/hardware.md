# Hardware and wiring

## Field configuration

| Component | Main ESP32 connection | Notes |
| --- | --- | --- |
| Left ESC signal | GPIO25 | Tested physical left thruster |
| Right ESC signal | GPIO2 | Tested physical right thruster; boot-strap pin, keep the ESC signal electrically quiet during reset |
| GPS TX | GPIO16 / UART2 RX | Required for GNSS input |
| GPS RX | GPIO17 / UART2 TX | Optional unless configuring the receiver |
| MPU-6050 SDA | GPIO21 | I2C |
| MPU-6050 SCL | GPIO22 | I2C |
| Camera UART TX | GPIO14 to S3 GPIO1 | Main controller to camera |
| Camera UART RX | GPIO13 from S3 GPIO2 | Camera to main controller |
| Common ground | GND | Required across both ESP32 boards, GPS, IMU, and ESC signal grounds |

## Camera board

The final camera firmware targets a GOOUUU ESP32-S3-CAM V1.5 with an OV3660. Camera and SD pins are board-specific and are defined near the top of the camera sketch. The SD card uses SD_MMC one-bit mode on GPIO39, GPIO38, and GPIO40.

## Power

- Do not power both ESP32 boards from multiple 5 V regulators unless their interaction is understood.
- Keep all control grounds common.
- Use a regulator with enough transient current for Wi-Fi, camera, PSRAM, and SD writes.
- Keep motor power wiring away from GPS, camera, and UART wiring.
- Add local decoupling near each controller and verify voltage during simultaneous motor and camera activity.

## ESC behavior measured on the boat

The final main-controller snapshot uses:

| Pulse | Meaning on this vehicle |
| --- | --- |
| 1500 us | Neutral / stop |
| Lower than 1500 us | Forward thrust |
| 1000 us | Maximum tested forward command |

This mapping is specific to the tested ESC setup. Confirm neutral and direction with propellers removed before copying it to another vehicle.

## Mechanical files

Two sonar-mount iterations are preserved in [`../hardware/cad/`](../hardware/cad/). The sonar work remained experimental and was not the basis of the final imaging result.
