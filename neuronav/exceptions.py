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
Production-ready exception classes for NeuroNav SLAM SDK.

This module defines a hierarchy of exceptions that provide clear error
messages and enable proper error handling in production robotic systems.
"""

from typing import Optional


class NeuroNavError(Exception):
    """Base exception for all NeuroNav SDK errors."""

    def __init__(self, message: str, details: Optional[dict] = None):
        """
        Initialize NeuroNav error.

        Args:
            message: Human-readable error message
            details: Optional dictionary with additional error context
        """
        super().__init__(message)
        self.message = message
        self.details = details or {}

    def __str__(self):
        if self.details:
            details_str = ", ".join(f"{k}={v}" for k, v in self.details.items())
            return f"{self.message} ({details_str})"
        return self.message


class SensorError(NeuroNavError):
    """Base exception for sensor-related errors."""
    pass


class SensorNotFoundError(SensorError):
    """Raised when a sensor device cannot be found or accessed."""

    def __init__(self, sensor_name: str, device_id: Optional[str] = None):
        details = {"sensor": sensor_name}
        if device_id:
            details["device_id"] = device_id
        super().__init__(
            f"Sensor '{sensor_name}' not found or cannot be accessed",
            details
        )


class SensorInitializationError(SensorError):
    """Raised when sensor initialization fails."""

    def __init__(self, sensor_name: str, reason: str):
        super().__init__(
            f"Failed to initialize sensor '{sensor_name}': {reason}",
            {"sensor": sensor_name, "reason": reason}
        )


class SensorTimeoutError(SensorError):
    """Raised when sensor operations timeout."""

    def __init__(self, sensor_name: str, operation: str, timeout: float):
        super().__init__(
            f"Sensor '{sensor_name}' timeout during '{operation}' ({timeout}s)",
            {"sensor": sensor_name, "operation": operation, "timeout": timeout}
        )


class SlamError(NeuroNavError):
    """Base exception for SLAM algorithm errors."""
    pass


class SlamInitializationError(SlamError):
    """Raised when SLAM initialization fails."""

    def __init__(self, algorithm: str, reason: str):
        super().__init__(
            f"Failed to initialize SLAM algorithm '{algorithm}': {reason}",
            {"algorithm": algorithm, "reason": reason}
        )


class SlamTrackingLostError(SlamError):
    """Raised when SLAM tracking is lost and cannot be recovered."""

    def __init__(self, algorithm: str, last_pose: Optional[tuple] = None):
        details = {"algorithm": algorithm}
        if last_pose:
            details["last_pose"] = last_pose
        super().__init__(
            f"SLAM tracking lost for '{algorithm}'",
            details
        )


class SlamConfigurationError(SlamError):
    """Raised when SLAM configuration is invalid."""

    def __init__(self, parameter: str, value: any, reason: str):
        super().__init__(
            f"Invalid SLAM configuration: {parameter}={value} - {reason}",
            {"parameter": parameter, "value": value, "reason": reason}
        )


class ProcessError(NeuroNavError):
    """Base exception for process management errors."""
    pass


class ProcessStartupError(ProcessError):
    """Raised when a subprocess fails to start."""

    def __init__(self, process_name: str, command: str, reason: str):
        super().__init__(
            f"Failed to start process '{process_name}': {reason}",
            {"process": process_name, "command": command, "reason": reason}
        )


class ProcessTerminationError(ProcessError):
    """Raised when a subprocess fails to terminate gracefully."""

    def __init__(self, process_name: str, pid: Optional[int] = None):
        details = {"process": process_name}
        if pid:
            details["pid"] = pid
        super().__init__(
            f"Failed to terminate process '{process_name}' gracefully",
            details
        )


class ProcessHealthCheckError(ProcessError):
    """Raised when process health check fails."""

    def __init__(self, process_name: str, reason: str):
        super().__init__(
            f"Health check failed for process '{process_name}': {reason}",
            {"process": process_name, "reason": reason}
        )


class ConfigurationError(NeuroNavError):
    """Raised when configuration validation fails."""

    def __init__(self, parameter: str, value: any, reason: str):
        super().__init__(
            f"Invalid configuration: {parameter}={value} - {reason}",
            {"parameter": parameter, "value": value, "reason": reason}
        )


class ROSError(NeuroNavError):
    """Base exception for ROS2-related errors."""
    pass


class ROSTopicError(ROSError):
    """Raised when ROS2 topic operations fail."""

    def __init__(self, topic: str, operation: str, reason: str):
        super().__init__(
            f"ROS2 topic '{topic}' {operation} failed: {reason}",
            {"topic": topic, "operation": operation, "reason": reason}
        )


class ROSNodeError(ROSError):
    """Raised when ROS2 node operations fail."""

    def __init__(self, node: str, reason: str):
        super().__init__(
            f"ROS2 node '{node}' error: {reason}",
            {"node": node, "reason": reason}
        )


class VisualizationError(NeuroNavError):
    """Raised when visualization operations fail."""

    def __init__(self, reason: str, port: Optional[int] = None):
        details = {"reason": reason}
        if port:
            details["port"] = port
        super().__init__(f"Visualization error: {reason}", details)


class ResourceError(NeuroNavError):
    """Raised when system resource issues are detected."""

    def __init__(self, resource: str, reason: str):
        super().__init__(
            f"Resource error ({resource}): {reason}",
            {"resource": resource, "reason": reason}
        )
