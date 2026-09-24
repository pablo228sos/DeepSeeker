# Navigation and coverage

## Mission definition

The field workflow records four GNSS corner positions. Each point is averaged for six seconds instead of accepting a single noisy fix. The controller checks the resulting quadrilateral before generating a route.

The planned path uses parallel survey lanes joined by rounded turns. The later firmware history includes lawnmower coverage, four-corner matching, connector-stop fixes, and an interlaced teardrop path. Those versions are preserved in `firmware/archive/` so that route behavior can be traced to a specific test.

## Heading estimation

GNSS alone gives position and, while moving, course over ground. It does not directly tell the controller where the bow points when the boat is stationary or moving slowly. The MPU-6050 supplies yaw-rate damping and attitude diagnostics, but without a magnetometer it does not provide an absolute north reference.

The controller therefore relies on GNSS course during forward motion, local path geometry, differential thrust, and IMU damping. This works best with continuous, gentle turns and enough forward speed for the GNSS course to become meaningful.

## GNSS gates in the final firmware

- At least 5 satellites for autonomous operation.
- HDOP no greater than 3.5.
- Position and course freshness checks.
- A local stop if manual commands expire.

These are operational thresholds, not guarantees of metric accuracy. M0016 commonly used 5-6 satellites and had median HDOP 1.56.

## Why earlier autonomous runs circled

Several effects can produce circles or a figure-eight path:

- Course over ground becomes unstable near zero speed.
- A turn command can remain active while the GPS course lags behind the real bow motion.
- Unequal thrusters create a persistent steering bias.
- Closely spaced waypoints can fall inside GNSS noise.
- A return target can be approached with the wrong assumed heading.

The later firmware addresses these issues with motor trim, quality gates, approach logic, rounded connectors, speed scheduling, and yaw-rate damping. Full autonomous area coverage was not validated to the same level as the synchronized imaging pipeline, so it remains an experimental subsystem.
