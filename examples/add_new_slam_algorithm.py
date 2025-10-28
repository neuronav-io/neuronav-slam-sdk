#!/usr/bin/env python3
"""
Example: Adding a New SLAM Algorithm

This template shows how to add a new SLAM algorithm to the SDK.
Follow this pattern to integrate ORB-SLAM3, Cartographer, etc.

Time to implement: < 30 minutes
"""

import subprocess
import time
from typing import Optional, Tuple, Dict
import numpy as np
from neuronav import SlamBase, SlamStatus, SlamConfig, run_slam, RealSenseSensor


class MySLAMAlgorithm(SlamBase):
    """
    Template for implementing a new SLAM algorithm

    Required methods to implement:
    - configure()
    - start()
    - stop()
    - reset()
    - get_current_pose()
    - get_current_map()
    - save_map()
    - load_map()
    - get_algorithm_name()
    - get_version()
    """

    def __init__(self):
        super().__init__()
        self._process: Optional[subprocess.Popen] = None
        self._sensor_topics: Optional[Dict[str, str]] = None

    def configure(self, config: SlamConfig) -> None:
        """Configure your SLAM algorithm"""
        self._config = config
        self._status = SlamStatus.INITIALIZING

    def set_sensor_topics(self, topics: Dict[str, str]) -> None:
        """
        Receive sensor topics from sensor adapter

        Topics dict contains: 'rgb', 'depth', 'camera_info', 'imu' (optional)
        """
        self._sensor_topics = topics

    def start(self) -> None:
        """Start your SLAM algorithm"""
        if not self._sensor_topics:
            raise RuntimeError("Sensor topics not set")

        # Example: Launch your SLAM node
        cmd = [
            'ros2', 'run', 'my_slam_package', 'my_slam_node',
            '--ros-args',
            '-r', f'image:={self._sensor_topics["rgb"]}',
            '-r', f'depth:={self._sensor_topics["depth"]}',
            '-r', f'camera_info:={self._sensor_topics["camera_info"]}'
        ]

        self._process = subprocess.Popen(cmd)
        time.sleep(2)
        self._status = SlamStatus.TRACKING

    def stop(self) -> None:
        """Stop your SLAM algorithm"""
        if self._process:
            self._process.terminate()
            self._process.wait(timeout=5)
            self._process = None
        self._status = SlamStatus.STOPPED

    def reset(self) -> None:
        """Reset/clear the map"""
        # Implement map reset logic
        pass

    def get_current_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Return current pose as (position, quaternion)

        Returns:
            position: np.array([x, y, z])
            quaternion: np.array([x, y, z, w])
        """
        # TODO: Subscribe to your pose topic and return latest pose
        return None

    def get_current_map(self) -> Optional[np.ndarray]:
        """Return current map as Nx3 point cloud"""
        # TODO: Extract and return map data
        return None

    def save_map(self, filepath: str) -> bool:
        """Save map to file"""
        try:
            # TODO: Call service or save map data
            return True
        except Exception:
            return False

    def load_map(self, filepath: str) -> bool:
        """Load map from file"""
        try:
            # TODO: Load map data
            return True
        except Exception:
            return False

    def get_algorithm_name(self) -> str:
        """Return algorithm name"""
        return "MySLAM"

    def get_version(self) -> str:
        """Return algorithm version"""
        return "1.0.0"


# Usage - just 2 lines!
if __name__ == "__main__":
    sensor = RealSenseSensor()
    run_slam(sensor, MySLAMAlgorithm())
