# Roadmap

The next stage is not adding more interface features. It is closing the measurement gaps exposed by the field data.

## 1. Recover the physical specification

- Measure hull length, beam, mass, camera offset, and antenna offset.
- Record battery, regulator, ESC, and thruster model numbers.
- Draw a complete power and signal schematic.
- Photograph the internal electronics and camera installation.

## 2. Validate autonomous coverage

- Use a tethered-water test before every unrestricted mission.
- Record the four-corner input, generated lanes, commanded PWM, GNSS course, cross-track error, and interventions.
- Define pass criteria for lane completion, turn overshoot, area coverage, and safe return.
- Compare manual and autonomous routes over the same marked area.

## 3. Calibrate the imaging system

- Calibrate the camera underwater through the installed housing window.
- Measure camera-to-bottom distance during capture.
- Add scale targets and independently surveyed control points.
- Repeat parallel lanes with measured forward and lateral overlap.

## 4. Improve positioning and heading

- Evaluate RTK GNSS for the surface platform.
- Add a calibrated magnetometer or dual-antenna heading source.
- Log controller state and heading uncertainty beside every frame.

## 5. Rebuild the reconstruction pipeline

- Reimplement the feature-aligned corridor workflow as versioned source code.
- Add synthetic and field regression datasets.
- Export a complete provenance record for every generated mosaic.
- Validate scale and placement against independent measurements before using the word orthophoto without qualification.

## 6. Revisit sonar separately

Sonar depth and material experiments should remain a separate workstream until a repeatable calibration rig and known targets are available. Camera results must not be used to imply sonar validation, or vice versa.
