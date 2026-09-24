# Reproducibility

## Numerical results

Mission-level timing, GNSS, speed, capture, and stability summaries can be recalculated directly from `telemetry.csv`:

```bash
python analysis/deepseeker_analysis.py summarize data/M0014/source/telemetry.csv
python analysis/deepseeker_analysis.py summarize data/M0016/source/telemetry.csv
```

## Diagnostic figure

```bash
python -m pip install -r analysis/requirements.txt
python analysis/deepseeker_analysis.py plot \
  data/M0016/source/telemetry.csv \
  --quality-csv data/M0016/derived/M0016_quality_metrics.csv \
  --output analysis/output/M0016_track_and_quality.png
```

The quality CSV is a derived dataset. The script uses its `quality_score` column but computes the route coordinates directly from the source telemetry.

## Raw images

The raw RAR archives are tracked with Git LFS. Extract them before running image-based experiments. Do not overwrite their contents.

## OpenDroneMap experiment

1. Extract a mission archive.
2. Keep `geo.txt` in the root of the image project.
3. Import the directory into ODM or WebODM.
4. Record the ODM version and every non-default option.
5. Treat the result as experimental until scale, underwater calibration, and independent control measurements are available.

## Known reproducibility boundary

The repository contains the final M0016 derived mosaics and method notes, but the original one-off reconstruction script was not recovered with the scattered camp files. The telemetry summaries and diagnostic plots are reproducible here. Reimplementing the complete feature-aligned mosaic pipeline remains a tracked follow-up rather than a false claim of full reproducibility.
