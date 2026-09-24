# Analysis

`deepseeker_analysis.py` recalculates mission-level metrics directly from camera telemetry and creates a route and quality diagnostic.

## Setup

```bash
python -m pip install -r analysis/requirements.txt
```

The `summarize` command uses only the Python standard library. Matplotlib is required only by `plot`.

## M0016

```bash
python analysis/deepseeker_analysis.py summarize \
  data/M0016/source/telemetry.csv \
  --output analysis/output/M0016_summary.json

python analysis/deepseeker_analysis.py plot \
  data/M0016/source/telemetry.csv \
  --quality-csv data/M0016/derived/M0016_quality_metrics.csv \
  --turn-start 123 \
  --turn-end 131 \
  --output analysis/output/M0016_track_and_quality.png
```

`raw_gps_polyline_m` is the sum of distances between consecutive raw GNSS fixes. It includes GNSS jitter and must not be presented as a calibrated traveled distance.

## Tests

```bash
python -m unittest discover -s analysis/tests -v
```

The historical M0014 and M0016 derived packages are preserved exactly as recovered. The script here provides a clean, reviewable baseline for repeating their telemetry-level analysis.
