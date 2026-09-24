# Field operations

## Before launch

1. Inspect the hull, cable penetrations, camera window, propellers, and battery restraint.
2. Clean both sides of the camera window and check for condensation.
3. Insert a formatted microSD card and confirm that the camera reports ready.
4. Power the vehicle with propellers clear of hands and loose material.
5. Wait for ESC arming and confirm neutral at 1500 us.
6. Wait for at least five satellites and acceptable HDOP.
7. Hold the boat over visible bottom and save several test frames.
8. Inspect full-resolution stone edges before committing to a mission.

## Mission capture

- Prefer 0.20-0.30 m/s on imaging lines.
- Keep the camera height as constant and as low as safe.
- Use dense forward overlap and 0.7-0.8 m lane spacing for the first area survey.
- Pause or mark images during turns.
- Use a second perpendicular pass when time and battery allow.
- Record water clarity, weather, waves, approximate depth, camera height, and any manual intervention.

## Recovery and data handling

1. Stop the motors before lifting the vehicle.
2. Power down before opening the electronics enclosure.
3. Copy the complete mission directory without renaming its images.
4. Calculate a SHA-256 checksum for the raw archive.
5. Keep raw files read-only and write all processed outputs to a separate directory.

## Minimum field note

```text
Mission ID:
Date and UTC interval:
Location:
Firmware versions:
Weather / waves:
Water clarity:
Estimated depth:
Camera height above bottom:
Route type and lane spacing:
Manual interventions:
Result and observed failures:
```
