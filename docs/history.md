# Development history

The version archive documents how the vehicle moved from basic manual control to synchronized imaging and experimental autonomous coverage.

| Version | Main change |
| --- | --- |
| v4.1 | Wi-Fi water-test control |
| v4.2 | Smoother motion commands |
| v4.3 | Reliability changes after water tests |
| v4.4 | Gentler turning behavior |
| v4.5 | MPU-6050 yaw-rate damping and bow control experiments |
| v5.0 | Lawn-mower survey generation |
| v5.1 | Four recorded corners matched to a survey route |
| v5.1.1 | Compile corrections |
| v5.1.2 | Camera status handling |
| v5.1.3 | Connector stop behavior |
| v6.0 | Interlaced teardrop route and final integrated controller snapshot |

The camera line likewise progressed from basic UART capture to geotagged mission folders and microSD retry handling. The final preserved camera snapshot is `v2.2_sd_retry`.

These files are historical field artifacts. They are intentionally not normalized into one polished codebase because that would erase the exact versions used during testing.
