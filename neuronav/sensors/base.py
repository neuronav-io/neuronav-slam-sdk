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
Base classes for sensor adapters with production-ready error handling.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from ..exceptions import ConfigurationError


@dataclass
class SensorConfig:
    """
    Configuration for sensor devices with validation.

    Attributes:
        device_id: Serial number or device path (None for auto-detect)
        rgb_width: RGB image width in pixels (must be > 0)
        rgb_height: RGB image height in pixels (must be > 0)
        depth_width: Depth image width in pixels (must be > 0)
        depth_height: Depth image height in pixels (must be > 0)
        fps: Frames per second (must be > 0 and <= 120)
        enable_imu: Enable IMU data streaming
        enable_ir: Enable infrared streams
        custom_params: Additional sensor-specific parameters
    """
    # Device settings
    device_id: Optional[str] = None

    # Image settings
    rgb_width: int = 640
    rgb_height: int = 480
    depth_width: int = 640
    depth_height: int = 480
    fps: int = 30

    # Sensor features
    enable_imu: bool = True
    enable_ir: bool = False

    # Custom parameters
    custom_params: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """Validate configuration parameters."""
        self.validate()

    def validate(self):
        """
        Validate sensor configuration parameters.

        Raises:
            ConfigurationError: If any parameter is invalid
        """
        if self.rgb_width <= 0:
            raise ConfigurationError(
                "rgb_width", self.rgb_width,
                "Width must be greater than 0"
            )
        if self.rgb_height <= 0:
            raise ConfigurationError(
                "rgb_height", self.rgb_height,
                "Height must be greater than 0"
            )
        if self.depth_width <= 0:
            raise ConfigurationError(
                "depth_width", self.depth_width,
                "Width must be greater than 0"
            )
        if self.depth_height <= 0:
            raise ConfigurationError(
                "depth_height", self.depth_height,
                "Height must be greater than 0"
            )
        if self.fps <= 0 or self.fps > 120:
            raise ConfigurationError(
                "fps", self.fps,
                "FPS must be between 1 and 120"
            )

        # Validate reasonable resolution limits
        max_dimension = 4096  # 4K max
        if self.rgb_width > max_dimension or self.rgb_height > max_dimension:
            raise ConfigurationError(
                "resolution", f"{self.rgb_width}x{self.rgb_height}",
                f"Resolution exceeds maximum {max_dimension}x{max_dimension}"
            )
        if self.depth_width > max_dimension or self.depth_height > max_dimension:
            raise ConfigurationError(
                "resolution", f"{self.depth_width}x{self.depth_height}",
                f"Resolution exceeds maximum {max_dimension}x{max_dimension}"
            )


class SensorBase(ABC):
    """
    Abstract base class for sensor devices with production-ready features.

    Features:
    - Configuration validation
    - Safe startup/shutdown with error handling
    - Health monitoring
    - Context manager support for automatic cleanup
    - State tracking
    """

    def __init__(self):
        self._config: Optional[SensorConfig] = None
        self._is_running: bool = False
        self._is_configured: bool = False

    @abstractmethod
    def configure(self, config: SensorConfig) -> None:
        """
        Configure the sensor with validated parameters.

        Args:
            config: Sensor configuration (will be validated)

        Raises:
            ConfigurationError: If configuration is invalid
            SensorError: If sensor configuration fails
        """
        pass

    @abstractmethod
    def start(self) -> None:
        """
        Start the sensor and begin publishing data.

        Raises:
            SensorNotFoundError: If sensor device is not available
            SensorInitializationError: If sensor fails to start
            RuntimeError: If sensor not configured before starting
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the sensor and cleanup resources.

        This method should be safe to call multiple times and should
        handle errors gracefully to ensure cleanup succeeds.
        """
        pass

    @abstractmethod
    def get_sensor_name(self) -> str:
        """
        Get human-readable sensor name.

        Returns:
            Sensor name (e.g., "Intel RealSense D435i")
        """
        pass

    @abstractmethod
    def get_ros_topics(self) -> Dict[str, str]:
        """
        Get ROS2 topic names published by this sensor.

        Returns:
            Dictionary with keys: 'rgb', 'depth', 'camera_info', 'imu' (optional)
            Example: {'rgb': '/camera/color/image_raw', 'depth': '/camera/depth/image_raw', ...}
        """
        pass

    def health_check(self) -> bool:
        """
        Check if sensor is healthy and producing data.

        Returns:
            True if sensor is operating normally, False otherwise
        """
        return self._is_running

    @property
    def is_running(self) -> bool:
        """Check if sensor is currently running."""
        return self._is_running

    @property
    def is_configured(self) -> bool:
        """Check if sensor has been configured."""
        return self._is_configured

    def __enter__(self):
        """Context manager entry - allows 'with sensor:' syntax."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup happens."""
        try:
            self.stop()
        except Exception:
            # Suppress exceptions during cleanup to avoid masking original errors
            pass
        return False  # Don't suppress exceptions from the with block
