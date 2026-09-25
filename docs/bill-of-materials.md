# Recovered bill of materials

This list describes the hardware that can be confirmed from the final firmware, photographs, purchase records, and surviving project files. Exact part numbers are left blank where the camp record does not support a confident claim.

| Subsystem | Recovered component | Status in final system |
| --- | --- | --- |
| Main controller | ESP32 development board | Motor control, GNSS, IMU, mission logic, and Wi-Fi control page |
| Camera controller | GOOUUU ESP32-S3-CAM V1.5 | Camera, PSRAM, microSD, and mission storage |
| Camera | OV3660 | Downward-looking JPEG capture |
| Positioning | GY-NEO6MV2 / NEO-6M-class GNSS module | UART position, speed, course, satellites, and HDOP |
| Inertial sensing | GY-521 MPU-6050 | Roll, pitch, angular rate, and stability diagnostics |
| Storage | microSD card | Numbered mission folders, JPEG, `telemetry.csv`, and `geo.txt` |
| Propulsion | Two electric underwater thrusters | Differential thrust |
| Motor control | Two bidirectional ESCs | 1500 us neutral; lower pulse commanded forward on the tested boat |
| Structure | Twin-pontoon catamaran hull with orange upper body | Surface platform and electronics enclosure |
| Power | Traction battery and regulated controller supply | Exact battery and regulator model not recovered |
| Long-range radio experiment | NRF24L01 PA/LNA module and external antenna hardware | Purchased and evaluated, not part of the final validated data path |
| Sonar experiment | Waterproof ultrasonic transducer and analog amplifier | Experimental; not used to support the final mapping result |

## Confirmed controller wiring

| Signal | Pin |
| --- | ---: |
| Left ESC | GPIO25 |
| Right ESC | GPIO2 |
| GPS RX from module TX | GPIO16 |
| GPS TX to module RX | GPIO17 |
| MPU-6050 SDA | GPIO21 |
| MPU-6050 SCL | GPIO22 |
| Main TX to camera RX | GPIO14 to S3 GPIO1 |
| Main RX from camera TX | GPIO13 from S3 GPIO2 |

Full electrical and power notes are in [`hardware.md`](hardware.md). Camera and SD pin definitions remain in the camera firmware because they are specific to the tested ESP32-S3-CAM board.

## Still missing from the archive

- Battery chemistry, capacity, and connector specification.
- Exact ESC and thruster commercial model numbers.
- Measured hull dimensions and mass.
- A final wiring-diagram drawing separate from the source-code pin table.

These fields should be measured from the surviving vehicle before a hardware revision is published.
