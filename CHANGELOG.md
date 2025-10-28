# Changelog

All notable changes to the NeuroNav SLAM SDK will be documented in this file.

## [0.0.1] - 2025-10-27

Initial release of NeuroNav SLAM SDK.

### Added

**Core Features**
- 2-line API for running SLAM: `sensor = RealSenseSensor()` + `run_slam(sensor, RTABMapSLAM())`
- Hardware abstraction layer for swapping sensors without code changes
- Configuration system with `SensorConfig` and `SlamConfig` dataclasses

**Sensor Support**
- Intel RealSense D435i/D455/D415 with IMU support
- Luxonis OAK-D Pro/W stereo cameras
- Automatic ROS2 driver launch and topic management

**SLAM Integration**
- RTAB-Map RGB-D SLAM with odometry and loop closure
- Real-time pose estimation on `/localization_pose`
- Automatic database management

**Visualization**
- Foxglove Bridge integration for web-based 3D visualization
- Point cloud, camera feeds, and pose display at `ws://localhost:8765`

**Infrastructure**
- Production error handling with comprehensive exception hierarchy
- Structured logging system
- Process management with health checks and graceful shutdown
- Docker support with pre-configured images

**Developer Tools**
- Example scripts for RealSense, OAK-D, and custom configurations
- Templates for adding new sensors (< 20 min) and SLAM algorithms (< 1 hour)
- Abstract base classes (`SensorBase`, `SlamBase`) for extensibility
- Full type hints and PEP 561 compliance

**Licensing**
- Licensed under Apache 2.0

---

Built on [RTAB-Map](https://github.com/introlab/rtabmap) by Mathieu Labbé & IntRoLab
