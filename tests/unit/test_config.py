# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for configuration validation."""

import pytest
from neuronav.sensors.base import SensorConfig
from neuronav.exceptions import ConfigurationError


def test_valid_sensor_config():
    """Test that valid configuration is accepted."""
    config = SensorConfig(
        rgb_width=640,
        rgb_height=480,
        depth_width=640,
        depth_height=480,
        fps=30,
        enable_imu=True
    )
    assert config.rgb_width == 640
    assert config.rgb_height == 480
    assert config.fps == 30
    assert config.enable_imu is True


def test_default_sensor_config():
    """Test default configuration values."""
    config = SensorConfig()
    assert config.rgb_width == 640
    assert config.rgb_height == 480
    assert config.fps == 30
    assert config.enable_imu is True


def test_invalid_width():
    """Test that invalid width raises ConfigurationError."""
    with pytest.raises(ConfigurationError) as exc_info:
        SensorConfig(rgb_width=0)
    assert "rgb_width" in str(exc_info.value)
    assert "greater than 0" in str(exc_info.value)


def test_invalid_height():
    """Test that invalid height raises ConfigurationError."""
    with pytest.raises(ConfigurationError) as exc_info:
        SensorConfig(rgb_height=-10)
    assert "rgb_height" in str(exc_info.value)


def test_invalid_fps_zero():
    """Test that zero FPS raises ConfigurationError."""
    with pytest.raises(ConfigurationError) as exc_info:
        SensorConfig(fps=0)
    assert "fps" in str(exc_info.value)


def test_invalid_fps_too_high():
    """Test that FPS above 120 raises ConfigurationError."""
    with pytest.raises(ConfigurationError) as exc_info:
        SensorConfig(fps=200)
    assert "fps" in str(exc_info.value)
    assert "between 1 and 120" in str(exc_info.value)


def test_resolution_too_large():
    """Test that excessive resolution raises ConfigurationError."""
    with pytest.raises(ConfigurationError) as exc_info:
        SensorConfig(rgb_width=10000, rgb_height=10000)
    assert "resolution" in str(exc_info.value).lower()
    assert "4096" in str(exc_info.value)


def test_custom_params():
    """Test custom parameters are stored correctly."""
    custom = {"my_param": "value", "another": 42}
    config = SensorConfig(custom_params=custom)
    assert config.custom_params == custom


def test_device_id():
    """Test device ID is stored correctly."""
    config = SensorConfig(device_id="123456")
    assert config.device_id == "123456"


def test_valid_high_resolution():
    """Test that 4K resolution is accepted."""
    config = SensorConfig(rgb_width=3840, rgb_height=2160)
    assert config.rgb_width == 3840
    assert config.rgb_height == 2160


def test_valid_high_fps():
    """Test that 120 FPS is accepted."""
    config = SensorConfig(fps=120)
    assert config.fps == 120


def test_valid_low_fps():
    """Test that 1 FPS is accepted."""
    config = SensorConfig(fps=1)
    assert config.fps == 1
