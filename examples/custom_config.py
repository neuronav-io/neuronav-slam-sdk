#!/usr/bin/env python3
"""
Custom Configuration Example

This example shows how to customize sensor and SLAM settings.
"""

from neuronav import (
    RealSenseSensor,
    RTABMapSLAM,
    run_slam,
    SensorConfig,
    SlamConfig
)

# Configure sensor
sensor_config = SensorConfig(
    rgb_width=640,
    rgb_height=480,
    depth_width=640,
    depth_height=480,
    fps=30,
    enable_imu=True
)

# Configure SLAM
slam_config = SlamConfig(
    enable_visualization=True,
    enable_loop_closing=True,
    custom_params={
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false'
    }
)

# Run SLAM with custom configuration
sensor = RealSenseSensor()
run_slam(
    sensor,
    RTABMapSLAM(),
    sensor_config=sensor_config,
    slam_config=slam_config
)
