#!/usr/bin/env python3
"""
Example: Adding a New Sensor

This template shows how to add a new sensor to the SDK.
Follow this pattern to integrate ZED, Azure Kinect, etc.

Time to implement: < 20 minutes
"""

import subprocess
import time
from typing import Dict, Optional
from neuronav import SensorBase, SensorConfig, run_slam, RTABMapSLAM


class MySensor(SensorBase):
    """
    Template for implementing a new sensor

    Required methods to implement:
    - configure()
    - start()
    - stop()
    - get_sensor_name()
    - get_ros_topics()
    """

    def __init__(self):
        super().__init__()
        self._camera_process: Optional[subprocess.Popen] = None

    def configure(self, config: SensorConfig) -> None:
        """Configure your sensor"""
        self._config = config

    def start(self) -> None:
        """Start your sensor driver"""
        if self._is_running:
            return

        # Example: Launch your sensor ROS2 driver
        cmd = [
            'ros2', 'launch', 'my_sensor_package', 'my_sensor_launch.py',
            f'width:={self._config.rgb_width}',
            f'height:={self._config.rgb_height}',
            f'fps:={self._config.fps}'
        ]

        # Add device ID if specified
        if self._config.device_id:
            cmd.append(f'device_id:={self._config.device_id}')

        self._camera_process = subprocess.Popen(cmd)
        time.sleep(3)  # Wait for sensor to initialize
        self._is_running = True

    def stop(self) -> None:
        """Stop your sensor"""
        if self._camera_process:
            self._camera_process.terminate()
            self._camera_process.wait(timeout=5)
            self._camera_process = None
        self._is_running = False

    def get_sensor_name(self) -> str:
        """Return sensor name"""
        return "My Sensor"

    def get_ros_topics(self) -> Dict[str, str]:
        """
        Return ROS2 topic names published by your sensor

        Required keys: 'rgb', 'depth', 'camera_info'
        Optional keys: 'imu'
        """
        topics = {
            'rgb': '/my_sensor/rgb/image_raw',
            'depth': '/my_sensor/depth/image_raw',
            'camera_info': '/my_sensor/rgb/camera_info'
        }

        if self._config and self._config.enable_imu:
            topics['imu'] = '/my_sensor/imu/data'

        return topics


# Usage - just 2 lines!
if __name__ == "__main__":
    sensor = MySensor()
    run_slam(sensor, RTABMapSLAM())
