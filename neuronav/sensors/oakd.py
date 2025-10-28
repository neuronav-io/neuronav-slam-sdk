"""
OAK-D Pro sensor adapter
"""

import subprocess
import time
from typing import Dict, Optional
from .base import SensorBase, SensorConfig


class OAKDSensor(SensorBase):
    """OAK-D Pro sensor adapter"""

    def __init__(self):
        super().__init__()
        self._camera_process: Optional[subprocess.Popen] = None
        self._sync_process: Optional[subprocess.Popen] = None
        self._imu_process: Optional[subprocess.Popen] = None

    def configure(self, config: SensorConfig) -> None:
        """Configure OAK-D sensor"""
        self._config = config

    def start(self) -> None:
        """Start OAK-D camera"""
        if self._is_running:
            return

        # Determine resolution setting
        if self._config.rgb_height <= 400:
            resolution = '400p'
        elif self._config.rgb_height <= 480:
            resolution = '480p'
        elif self._config.rgb_height <= 720:
            resolution = '720p'
        else:
            resolution = '800p'

        # Launch OAK-D Pro camera driver
        camera_cmd = [
            'ros2', 'launch', 'depthai_examples', 'stereo_inertial_node.launch.py',
            'depth_aligned:=false',
            'enableRviz:=false',
            f'monoResolution:={resolution}'
        ]

        self._camera_process = subprocess.Popen(camera_cmd)
        time.sleep(3)  # Wait for camera to initialize

        # Launch RGBD sync node for RTAB-Map compatibility (exact sync as in working example)
        sync_cmd = [
            'ros2', 'run', 'rtabmap_sync', 'rgbd_sync',
            '--ros-args',
            '-p', 'frame_id:=oak-d-base-frame',
            '-p', 'subscribe_rgbd:=true',
            '-p', 'subscribe_odom_info:=true',
            '-p', 'approx_sync:=false',
            '-p', 'wait_imu_to_init:=true',
            '-r', 'rgb/image:=/right/image_rect',
            '-r', 'rgb/camera_info:=/right/camera_info',
            '-r', 'depth/image:=/stereo/depth'
        ]
        self._sync_process = subprocess.Popen(sync_cmd)
        time.sleep(1)

        # Launch IMU filter for quaternion computation
        imu_cmd = [
            'ros2', 'run', 'imu_filter_madgwick', 'imu_filter_madgwick_node',
            '--ros-args',
            '-p', 'use_mag:=false',
            '-p', 'world_frame:=enu',
            '-p', 'publish_tf:=false',
            '-r', 'imu/data_raw:=/imu'
        ]
        self._imu_process = subprocess.Popen(imu_cmd)
        time.sleep(1)

        self._is_running = True

    def stop(self) -> None:
        """Stop OAK-D sensor"""
        if self._imu_process:
            self._imu_process.terminate()
            self._imu_process.wait(timeout=5)
            self._imu_process = None

        if self._sync_process:
            self._sync_process.terminate()
            self._sync_process.wait(timeout=5)
            self._sync_process = None

        if self._camera_process:
            self._camera_process.terminate()
            self._camera_process.wait(timeout=5)
            self._camera_process = None

        self._is_running = False

    def get_sensor_name(self) -> str:
        """Get sensor name"""
        return "OAK-D Pro"

    def get_ros_topics(self) -> Dict[str, str]:
        """Get ROS2 topic names (after sync)"""
        topics = {
            'rgb': '/rgbd_image/rgb/image',
            'depth': '/rgbd_image/depth/image',
            'camera_info': '/rgbd_image/rgb/camera_info',
            'imu': '/imu/data'  # Always include IMU for OAK-D
        }
        return topics
