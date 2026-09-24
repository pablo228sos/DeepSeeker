# M0016 Experimental Strip Orthomosaic — Research Result

## Dataset and acquisition

Mission M0016 contains **208 geotagged images** from one manually controlled outbound-and-return transect. The sequence was divided into **122 outbound frames**, **9 turning frames**, and **77 return frames**. Turning frames were excluded because camera rotation and acceleration reduce geometric consistency. The total recording duration was approximately **279 s**. Median speed was **0.94 km/h**, median capture time was **446 ms**, and **38.0%** of frames were classified as stable by the onboard inertial filter.

## Processing method

GPS coordinates were converted to local metric coordinates and smoothed into a monotonic 0–10 m transect model. Return images were rotated by 180° so both passes shared the same ground orientation. Each image was assigned a quality score using edge sharpness, contrast, tonal range, and inertial stability. At 0.2 m stations, the higher-quality observation from the outbound or return pass was selected. Selected image strips were white-balanced, contrast-enhanced, mildly sharpened, and fused into a pushbroom-style corridor mosaic.

## Result

The output demonstrates a continuous representation of the lake bottom along the surveyed line. Rocky regions are visibly distinguishable and preserve useful texture. The central sandy region is less reliable because turbidity, low texture, and moving sunlight patterns reduce repeatable visual features. Combining two passes improves continuity because the processing pipeline can select the clearer observation at each station.

This result should be described as an **experimental strip orthomosaic or corridor mosaic**, not as a complete 10 × 10 m orthophotoplan. Only one line was surveyed, underwater camera calibration was unavailable, camera height above the bottom was not measured, and consumer GPS cannot provide survey-grade placement. A complete area product requires the same acquisition procedure over parallel lanes with lateral overlap.

## Quantitative results

- Total images: **208**
- Frames used as source pools: **122 outbound + 77 return**
- Frames excluded during turning: **9**
- Median image capture time: **446 ms**
- Median reported speed: **0.94 km/h**
- Stable-frame fraction: **38.0%**
- Median satellites: **6**
- Median HDOP: **1.56**
- Best quality frame: **IMG_000012.JPG**

## Research interpretation

The trial confirms that a low-cost camera can produce usable bottom imagery when stones provide texture and the water is locally clear. Two-pass acquisition is beneficial because it creates redundant observations and allows low-quality frames to be replaced. The primary remaining limitations are underwater focus, turbidity, illumination changes, and the absence of calibrated scale. These findings directly support the next design iteration: slower straight-line motion, dense parallel lanes, fixed camera height, calibrated optics, and deliberate exclusion of turns from the imaging sequence.
