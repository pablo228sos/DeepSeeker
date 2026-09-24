# Field datasets

This directory preserves the two mission datasets used for the main diagnostic and final result.

## M0014

- 362 JPEG images at 2048 x 1536.
- Complete `telemetry.csv` and OpenDroneMap-compatible `geo.txt`.
- Important diagnostic run: the acquisition system worked, but optical softness and haze prevented reliable stitching.
- The derived directory contains per-frame metrics, contact sheets, enhancement comparisons, and the failed stitching diagnostic.

## M0016

- 208 JPEG images with complete telemetry.
- One manually controlled outbound-and-return transect.
- Strongest field result and source of the connected corridor mosaic.
- The derived directory contains image-quality metrics, route diagnostics, frame selection, pushbroom-style strips, and feature-aligned results.

## Directory policy

```text
M00xx/
  raw/       immutable archive from the field card
  source/    extracted telemetry and geo files for easy inspection
  derived/   metrics, figures, mosaics, and method notes
```

Do not replace raw archives with processed files. New processing results should state the input checksum, software version, parameters, and limitations.

The raw archives use Git LFS. Run `git lfs install` before cloning or committing this repository.
