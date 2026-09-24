# Field results

## M0014: diagnostic mission

| Measure | Result |
| --- | ---: |
| Saved images | 362 |
| Resolution | 2048 x 1536 |
| Median image interval | about 1 s |
| Mean recorded speed | about 1.76 km/h |
| Median GPS displacement per frame | about 0.55 m |
| Median capture time | about 300 ms |
| Frames above the diagnostic detail threshold | about 8% |

M0014 was a useful failure. It proved that the camera and telemetry pipeline could run for hundreds of frames, but it did not provide enough stable image detail for a defensible mosaic. Sharpness remained poor even when speed and inertial stability were favorable, which pointed to focus, the housing window, turbidity, distance to the bottom, and illumination as the dominant problems.

## M0016: strongest field mission

| Measure | Result |
| --- | ---: |
| Saved images | 208 / 208 |
| Duration | 279 s |
| Outbound frames | 122 |
| Turning frames excluded | 9 |
| Return frames | 77 |
| Median speed | 0.945 km/h / 0.262 m/s |
| Median capture time | 445.5 ms |
| Stable-frame fraction | 38.0% |
| Median satellites | 6 |
| Median HDOP | 1.56 |
| Best-scoring frame | `IMG_000012.JPG` |

M0016 produced a continuous visual representation of the surveyed bottom corridor. Rocky sections retained recognizable texture. The central sandy section remained less reliable because it had weaker texture, greater haze, and changing sunlight patterns.

The result supports three conclusions:

1. The two-controller platform can capture and preserve a complete synchronized field sequence.
2. Redundant outbound and return observations improve continuity by allowing a clearer frame to replace a weaker one.
3. Better navigation alone is not enough; optical calibration and controlled camera geometry are required for metric mapping.

## What was demonstrated

- Reliable local image storage with synchronized GNSS and IMU telemetry.
- A complete 208-frame field run without a missing expected image.
- A connected feature-aligned reconstruction of one underwater transect.
- Practical identification of the optical and geometric limits of the low-cost setup.

## What was not demonstrated

- A validated autonomous 10 x 10 m coverage mission.
- A survey-grade orthophoto or bathymetric map.
- Reliable material classification from sonar or imagery.
- Centimeter-level positioning.

The raw evidence and derived files for both missions are available under [`../data/`](../data/).
