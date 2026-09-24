# M0014 photo and orthomosaic diagnostic

## Dataset

- 362 JPEG images at 2048×1536.
- Median interval between images: approximately 1 second.
- Mean recorded speed: approximately 1.76 km/h (0.49 m/s).
- Median GPS displacement between consecutive images: approximately 0.55 m.
- Median camera capture/save time: approximately 300 ms.

## Result of stitching attempt

A feature- and optical-flow-based stitch was attempted on frames 270–290, the clearest continuous section. The output is included as `M0014_orthomosaic_attempt_diagnostic.png`.

It is not a valid metric orthophoto. The algorithm cannot find enough stable common details between most consecutive frames. The main limitations are severe optical softness/haze and insufficient reliable image texture, not simply route coverage.

## Image-quality findings

- Only about 8% of frames exceed a moderate local-detail threshold.
- The clearest continuous group is approximately frames 270–288.
- Measured sharpness has almost no relationship with recorded speed or the `stable` flag.
- Very slow or nearly stationary frames are also soft.

This indicates that timing alone will not solve the problem. The dominant cause is optical: turbidity, camera focus through the waterproof window, dirty/condensed window, excessive camera-to-bottom distance, or a combination of these.

## Recommended capture logic for the next 10×10 m run

1. Before the mission, stop the robot above visible bottom and capture 5 test images.
2. Do not start the grid unless the test image contains clearly defined stone edges at full resolution.
3. Use 0.20–0.30 m/s on photo strips.
4. Use an 800–900 ms photo interval.
5. Use strips 0.7–0.8 m apart for the first pass.
6. Repeat a second pass perpendicular to the first when time permits.
7. Photograph only while moving approximately straight; pause automatic capture during turns.
8. Continue after one failed capture; stop only after several consecutive failures.
9. Log the distance traveled since the last saved image and request another image after 0.20–0.25 m, with a time fallback of 900 ms.
10. Reject or mark frames captured during high yaw rate or strong roll/pitch, but force a capture if the distance threshold is exceeded.

## Hardware check before changing software again

- Clean both sides of the camera window.
- Check for condensation or a water film.
- Test focus in a bucket or at the real operating depth using stones with sharp edges.
- Adjust the OV3660 lens focus physically if the module permits it.
- Keep the camera as close to the bottom as safely possible.
- Add downward lighting only if it does not illuminate suspended particles directly in front of the lens.
