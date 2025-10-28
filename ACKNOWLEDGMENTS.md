# Acknowledgments

NeuroNav SLAM SDK builds upon the outstanding work of several open-source projects and hardware manufacturers. We are deeply grateful for their contributions to the robotics and computer vision communities.

## Core SLAM Technology

### RTAB-Map (Real-Time Appearance-Based Mapping)

**Project**: https://github.com/introlab/rtabmap
**Website**: http://introlab.github.io/rtabmap/
**License**: BSD 3-Clause License
**Copyright**: Copyright (c) 2010-2025, Mathieu Labbé - IntRoLab - Université de Sherbrooke. All rights reserved.
**Authors**: Mathieu Labbé and François Michaud

RTAB-Map is a RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. NeuroNav SDK integrates with RTAB-Map through ROS2 interfaces and does not distribute or modify RTAB-Map source code.

**Key Features We Leverage:**
- Real-time loop closure detection
- Memory management for long-term mapping
- Multi-session mapping capabilities
- ROS2 integration
- Rich visualization tools

**Citation:**
```bibtex
@article{labbe2019rtabmap,
  title={RTAB-Map as an open-source lidar and visual simultaneous localization and mapping library for large-scale and long-term online operation},
  author={Labb{\'e}, Mathieu and Michaud, Fran{\c{c}}ois},
  journal={Journal of Field Robotics},
  volume={36},
  number={2},
  pages={416--446},
  year={2019},
  publisher={Wiley Online Library}
}
```

**Why RTAB-Map?**
- Production-proven in numerous robotic applications
- Excellent ROS2 integration
- Active maintenance and community support
- Memory-efficient for long-term operation
- Robust loop closure detection

## Camera Hardware & SDKs

### Intel RealSense

**Website**: https://www.intelrealsense.com/
**SDK**: https://github.com/IntelRealSense/librealsense
**ROS2 Wrapper**: https://github.com/IntelRealSense/realsense-ros
**License**: Apache 2.0

Intel RealSense depth cameras (D435i, D455, D415) provide high-quality RGB-D data with integrated IMU for robust SLAM.

**Supported Models:**
- **D435i**: Indoor SLAM with IMU
- **D455**: Extended range with IMU
- **D415**: High accuracy stereo vision

**Key Features:**
- High-quality depth sensing
- Integrated IMU (D435i, D455)
- Cross-platform SDK
- Excellent ROS2 integration
- Active infrared stereo technology

**Why RealSense?**
- Industry-standard for robotic vision
- Affordable and widely available
- Excellent documentation and support
- Proven reliability in production
- Rich SDK with Python bindings

### Luxonis OAK-D

**Company**: Luxonis Inc.
**Website**: https://www.luxonis.com/
**Documentation**: https://docs.luxonis.com/
**SDK**: https://github.com/luxonis/depthai
**ROS2 Driver**: https://github.com/luxonis/depthai-ros
**License**: MIT

OAK-D (OpenCV AI Kit with Depth) provides on-board AI processing with stereo depth perception and integrated IMU.

**Supported Models:**
- **OAK-D Pro**: With IMU for SLAM
- **OAK-D Pro W**: Wide FOV variant
- **OAK-D Lite**: Compact version

**Key Features:**
- On-board Intel Myriad X VPU
- Stereo depth up to 38m
- 4K 60 FPS camera
- Integrated IMU (Pro models)
- Edge AI capabilities
- USB3 single cable connection

**Why OAK-D?**
- Powerful on-board processing
- Excellent depth quality
- Compact form factor
- Active open-source community
- Neural network capabilities
- Cost-effective

## ROS2 Ecosystem

**Project**: Robot Operating System 2 (ROS2)
**Website**: https://www.ros.org/
**Documentation**: https://docs.ros.org/
**License**: Various (Apache 2.0, BSD, etc.)

ROS2 provides the middleware and tools that enable seamless integration between sensors, algorithms, and visualization.

**Key ROS2 Packages We Use:**
- `realsense2_camera`: RealSense camera driver
- `depthai_ros`: OAK-D camera driver
- `rtabmap_ros`: RTAB-Map SLAM integration
- `imu_filter_madgwick`: IMU orientation filtering
- `foxglove_bridge`: WebSocket bridge for visualization

**Organizations:**
- Open Robotics
- ROS Industrial Consortium
- Worldwide ROS community

## Visualization

### Foxglove Studio

**Website**: https://foxglove.dev/
**GitHub**: https://github.com/foxglove/studio
**License**: MPL 2.0

Foxglove Studio provides powerful web-based visualization for robotics data, replacing traditional RViz in our modern workflow.

**Features We Use:**
- Real-time 3D visualization
- Point cloud rendering
- Image streaming
- Topic introspection
- Cross-platform support

## Python Ecosystem

We rely on the excellent Python scientific computing ecosystem:

- **NumPy**: Numerical computing - https://numpy.org/
- **OpenCV**: Computer vision - https://opencv.org/
- **PyYAML**: Configuration parsing - https://pyyaml.org/
- **psutil**: Process management - https://github.com/giampaolo/psutil
- **pytest**: Testing framework - https://pytest.org/
- **black**: Code formatting - https://github.com/psf/black
- **mypy**: Type checking - https://mypy-lang.org/

## Community Contributions

NeuroNav SDK was made possible by:

- The broader robotics open-source community
- SLAM researchers who shared their knowledge
- Hardware manufacturers who provide SDKs
- ROS2 maintainers and contributors
- Everyone who files issues and contributes improvements

## Our Commitment

We are committed to:

1. **Giving Back**: Contributing improvements back to upstream projects
2. **Open Source**: Keeping NeuroNav SDK open and accessible
3. **Documentation**: Providing clear examples and guides
4. **Attribution**: Properly crediting all technologies we build upon
5. **Support**: Helping others integrate SLAM into their robots

## How to Acknowledge

If you use NeuroNav SLAM SDK in your work, please cite:

**For software/technical reports:**
```bibtex
@software{neuronav_slam_sdk,
  title={NeuroNav SLAM SDK: Production-Ready SLAM for Robotics},
  author={{NeuroNav Team}},
  year={2025},
  url={https://github.com/neuronav/neuronav-slam-sdk},
  version={0.1.0}
}
```

**For academic papers:**
```bibtex
@misc{neuronav2025slam,
  title={NeuroNav SLAM SDK: A Production-Ready Framework for Robotic Simultaneous Localization and Mapping},
  author={{NeuroNav Team}},
  year={2025},
  howpublished={\url{https://github.com/neuronav/neuronav-slam-sdk}},
  note={Software framework built on RTAB-Map}
}
```

**And please also cite RTAB-Map** as the core SLAM technology:

```bibtex
@article{labbe2019rtabmap,
  title={RTAB-Map as an open-source lidar and visual simultaneous localization and mapping library for large-scale and long-term online operation},
  author={Labb{\'e}, Mathieu and Michaud, Fran{\c{c}}ois},
  journal={Journal of Field Robotics},
  volume={36},
  number={2},
  pages={416--446},
  year={2019},
  publisher={Wiley Online Library}
}
```

## Contact

For questions about NeuroNav SDK:
- GitHub Issues: https://github.com/neuronav/neuronav-slam-sdk/issues
- Email: eldaniz@neuronav.io

For questions about upstream projects, please contact them directly through their respective channels.

---

Thank you to everyone who makes open robotics possible! 🤖❤️
