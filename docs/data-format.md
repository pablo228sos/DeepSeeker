# Data format

## Mission directory

Each camera mission contains numbered JPEG files plus two text products:

```text
M0016/
  IMG_000001.JPG
  IMG_000002.JPG
  ...
  telemetry.csv
  geo.txt
```

## `telemetry.csv`

| Column | Unit or meaning |
| --- | --- |
| `image` | JPEG filename |
| `utc` | ISO 8601 UTC timestamp |
| `latitude`, `longitude` | WGS84 decimal degrees |
| `altitude_m` | GNSS altitude in metres |
| `satellites` | Satellites used by receiver |
| `hdop` | Horizontal dilution of precision |
| `speed_kmph` | GNSS ground speed |
| `course_deg` | GNSS course over ground |
| `roll_deg`, `pitch_deg` | IMU attitude estimate |
| `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps` | Angular rate in degrees per second |
| `total_acc_g` | Acceleration magnitude in g |
| `stable`, `level` | Onboard diagnostic flags, 0 or 1 |
| `jpeg_bytes` | Saved JPEG size |
| `capture_ms` | Capture and storage duration |

The `stable` and `level` flags are diagnostic labels based on onboard thresholds. They are not proof that a frame is sharp.

## `geo.txt`

The first line declares the coordinate reference system. Remaining lines contain image name, longitude, latitude, and altitude:

```text
EPSG:4326
IMG_000001.JPG 77.0738157 42.6398862 1621.30
```

The longitude-before-latitude order follows the OpenDroneMap geolocation-file convention.

## Integrity

Raw archives are immutable inputs. Derived metrics and figures must be written outside the archive and described by a method note. [`../data/manifest.csv`](../data/manifest.csv) records checksums and mission-level contents.
