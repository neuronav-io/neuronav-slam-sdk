# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""
Intel RealSense sensor adapter with production-ready error handling.
"""

from typing import Dict, Optional
from .base import SensorBase, SensorConfig
from ..process_manager import ManagedProcess, ProcessManager
from ..logger import get_logger
from ..exceptions import (
    SensorInitializationError,
    SensorNotFoundError,
    SensorError
)


class RealSenseSensor(SensorBase):
    """
    Intel RealSense D435i/D455/D415 sensor adapter.

    Features:
    - Automatic depth alignment
    - IMU support with Madgwick filter
    - Health monitoring
    - Graceful shutdown
    - Production-ready error handling
    """

    def __init__(self):
        super().__init__()
        self._process_manager = ProcessManager()
        self._logger = get_logger("sensor.realsense")

    def configure(self, config: SensorConfig) -> None:
        """
        Configure RealSense sensor.

        Args:
            config: Validated sensor configuration

        Raises:
            ConfigurationError: If configuration is invalid
        """
        self._logger.info("Configuring Intel RealSense sensor")

        # Validation happens in SensorConfig.__post_init__
        self._config = config
        self._is_configured = True

        self._logger.info(
            f"RealSense configured: {config.rgb_width}x{config.rgb_height} @ {config.fps}fps, "
            f"IMU: {config.enable_imu}"
        )

    def start(self) -> None:
        """
        Start RealSense camera and IMU.

        Raises:
            SensorNotFoundError: If RealSense device not found
            SensorInitializationError: If sensor fails to start
            RuntimeError: If sensor not configured
        """
        if not self._is_configured:
            raise RuntimeError("Sensor must be configured before starting")

        if self._is_running:
            self._logger.warning("RealSense already running")
            return

        self._logger.info("Starting Intel RealSense camera driver")

        try:
            # Build camera command
            camera_cmd = [
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'camera_namespace:=',
                f'rgb_camera.profile:={self._config.rgb_width}x{self._config.rgb_height}x{self._config.fps}',
                'align_depth.enable:=true',
                'enable_sync:=true',
                'depth_module.emitter_enabled:=1'
            ]

            # Add IMU if enabled
            if self._config.enable_imu:
                camera_cmd.extend([
                    'enable_gyro:=true',
                    'enable_accel:=true',
                    'unite_imu_method:=2'
                ])

            # Add device ID if specified
            if self._config.device_id:
                camera_cmd.append(f'serial_no:={self._config.device_id}')
                self._logger.info(f"Using device ID: {self._config.device_id}")

            # Create and start camera process
            camera_process = ManagedProcess(
                name="realsense_camera",
                command=camera_cmd,
                startup_timeout=3.0,
                shutdown_timeout=5.0
            )
            self._process_manager.add_process(camera_process)
            camera_process.start()

            # Launch IMU filter if enabled
            if self._config.enable_imu:
                self._logger.info("Starting IMU filter")
                imu_cmd = [
                    'ros2', 'run', 'imu_filter_madgwick', 'imu_filter_madgwick_node',
                    '--ros-args',
                    '-p', 'use_mag:=false',
                    '-p', 'world_frame:=enu',
                    '-p', 'publish_tf:=false',
                    '-r', 'imu/data_raw:=/camera/imu'
                ]
                imu_process = ManagedProcess(
                    name="imu_filter",
                    command=imu_cmd,
                    startup_timeout=1.0,
                    shutdown_timeout=3.0
                )
                self._process_manager.add_process(imu_process)
                imu_process.start()

            self._is_running = True
            self._logger.info("RealSense camera started successfully")

        except FileNotFoundError as e:
            raise SensorNotFoundError(
                "Intel RealSense",
                self._config.device_id
            )
        except Exception as e:
            self._logger.error(f"Failed to start RealSense: {e}", exc_info=True)
            # Cleanup on failure
            try:
                self._process_manager.stop_all(force=True)
            except Exception:
                pass
            raise SensorInitializationError("Intel RealSense", str(e))

    def stop(self) -> None:
        """Stop RealSense sensor and cleanup resources."""
        if not self._is_running:
            self._logger.debug("RealSense not running, nothing to stop")
            return

        self._logger.info("Stopping Intel RealSense sensor")

        try:
            self._process_manager.stop_all()
            self._is_running = False
            self._logger.info("RealSense stopped successfully")
        except Exception as e:
            self._logger.error(f"Error stopping RealSense: {e}", exc_info=True)
            # Force kill as fallback
            try:
                self._process_manager.stop_all(force=True)
                self._is_running = False
            except Exception as e2:
                self._logger.critical(f"Failed to force stop RealSense: {e2}")

    def health_check(self) -> bool:
        """
        Check if RealSense is healthy.

        Returns:
            True if all processes are running and healthy
        """
        if not self._is_running:
            return False

        return self._process_manager.health_check_all()

    def get_sensor_name(self) -> str:
        """Get sensor name."""
        return "Intel RealSense"

    def get_ros_topics(self) -> Dict[str, str]:
        """
        Get ROS2 topic names published by RealSense.

        Returns:
            Dictionary of topic names
        """
        topics = {
            'rgb': '/camera/color/image_raw',
            'depth': '/camera/aligned_depth_to_color/image_raw',
            'camera_info': '/camera/color/camera_info'
        }
        if self._config and self._config.enable_imu:
            topics['imu'] = '/imu/data'
        return topics
