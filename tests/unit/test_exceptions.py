# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for exception classes."""

import pytest
from neuronav.exceptions import (
    NeuroNavError,
    SensorError,
    SensorNotFoundError,
    SensorInitializationError,
    SlamError,
    ConfigurationError,
    ProcessStartupError,
)


def test_base_exception():
    """Test base NeuroNavError."""
    err = NeuroNavError("test error")
    assert str(err) == "test error"
    assert err.message == "test error"
    assert err.details == {}


def test_exception_with_details():
    """Test exception with additional details."""
    details = {"sensor": "RealSense", "device_id": "123"}
    err = NeuroNavError("test error", details)
    assert err.message == "test error"
    assert err.details == details
    assert "sensor=RealSense" in str(err)
    assert "device_id=123" in str(err)


def test_sensor_not_found():
    """Test SensorNotFoundError."""
    err = SensorNotFoundError("RealSense", "123456")
    assert "RealSense" in str(err)
    assert err.details["sensor"] == "RealSense"
    assert err.details["device_id"] == "123456"


def test_sensor_initialization_error():
    """Test SensorInitializationError."""
    err = SensorInitializationError("OAK-D", "device not responding")
    assert "OAK-D" in str(err)
    assert "device not responding" in str(err)
    assert err.details["sensor"] == "OAK-D"
    assert err.details["reason"] == "device not responding"


def test_configuration_error():
    """Test ConfigurationError."""
    err = ConfigurationError("fps", -1, "must be positive")
    assert "fps" in str(err)
    assert "-1" in str(err)
    assert "must be positive" in str(err)


def test_process_startup_error():
    """Test ProcessStartupError."""
    err = ProcessStartupError("camera_node", "ros2 run pkg node", "command not found")
    assert "camera_node" in str(err)
    assert err.details["process"] == "camera_node"
    assert err.details["command"] == "ros2 run pkg node"
    assert err.details["reason"] == "command not found"


def test_exception_inheritance():
    """Test exception class hierarchy."""
    assert issubclass(SensorError, NeuroNavError)
    assert issubclass(SensorNotFoundError, SensorError)
    assert issubclass(SensorInitializationError, SensorError)
    assert issubclass(SlamError, NeuroNavError)
    assert issubclass(ConfigurationError, NeuroNavError)


def test_raise_and_catch():
    """Test raising and catching exceptions."""
    with pytest.raises(SensorNotFoundError) as exc_info:
        raise SensorNotFoundError("TestSensor")

    assert "TestSensor" in str(exc_info.value)
    assert isinstance(exc_info.value, SensorError)
    assert isinstance(exc_info.value, NeuroNavError)
