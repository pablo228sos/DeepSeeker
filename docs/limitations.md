# Limitations

DeepSeeker is a field-tested research prototype. The following limits are part of the result, not footnotes to hide.

## Positioning

The NEO-6M-class receiver provides consumer GNSS positioning. Five or six satellites and an HDOP near 1.5 can support route guidance, but they do not guarantee centimeter accuracy. Repeated coordinates, lag, multipath, and short-term drift are visible in the logs.

## Heading

The MPU-6050 measures angular rate and acceleration but has no magnetometer. Absolute heading is inferred mainly from GNSS course while moving. Heading is therefore weak at low speed and during the first part of a turn.

## Imaging geometry

The camera was not calibrated underwater through the installed window. Camera height above the bottom was not logged, the lakebed was not planar, and the housing introduces refraction. The resulting mosaic cannot be treated as a metrically rectified map.

## Water and light

Turbidity, suspended particles, glare, caustic sunlight, waves, and low-texture sand change from frame to frame. These effects can defeat feature matching even when route overlap is high.

## Mission coverage

M0016 was one outbound-and-return corridor, not a complete area grid. The connected result demonstrates transect reconstruction only.

## Sonar

The sonar and analog-amplifier tests are part of the project history, but their lake readings were inconsistent and were not used to support the final mapping claim. Material discrimination was not validated.

## Safety

The vehicle has no certified obstacle avoidance, redundant navigation computer, geofence guarantee, or independent emergency-stop radio. All tests require direct supervision and a recovery plan.
