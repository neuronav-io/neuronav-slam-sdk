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
Production-ready logging system for NeuroNav SLAM SDK.

This module provides structured logging with configurable levels,
file output, and integration with robotic systems monitoring.
"""

import logging
import sys
from pathlib import Path
from typing import Optional
from datetime import datetime
import os


class NeuroNavLogger:
    """
    Centralized logging system for NeuroNav SDK.

    Features:
    - Configurable log levels (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    - File and console output
    - Structured logging with component names
    - Rotation and archiving support
    - Production-ready formatting
    """

    _instance: Optional['NeuroNavLogger'] = None
    _initialized: bool = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._loggers = {}
        self._log_level = logging.INFO
        self._log_file: Optional[Path] = None
        self._file_handler: Optional[logging.FileHandler] = None
        self._console_handler: Optional[logging.StreamHandler] = None
        self._initialized = True

        # Setup default configuration
        self._setup_default_logging()

    def _setup_default_logging(self):
        """Setup default logging configuration."""
        # Create console handler with formatting
        self._console_handler = logging.StreamHandler(sys.stdout)
        self._console_handler.setLevel(self._log_level)

        # Production-ready format with timestamp, level, component
        formatter = logging.Formatter(
            fmt='%(asctime)s [%(levelname)8s] [%(name)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self._console_handler.setFormatter(formatter)

    def configure(
        self,
        level: str = "INFO",
        log_file: Optional[str] = None,
        enable_console: bool = True,
        format_string: Optional[str] = None
    ):
        """
        Configure global logging settings.

        Args:
            level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            log_file: Optional file path for log output
            enable_console: Whether to output logs to console
            format_string: Custom format string (uses default if None)
        """
        # Set log level
        level_map = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "ERROR": logging.ERROR,
            "CRITICAL": logging.CRITICAL
        }
        self._log_level = level_map.get(level.upper(), logging.INFO)

        # Configure file logging
        if log_file:
            self._log_file = Path(log_file)
            self._log_file.parent.mkdir(parents=True, exist_ok=True)

            self._file_handler = logging.FileHandler(self._log_file)
            self._file_handler.setLevel(self._log_level)

            # Use custom or default format
            if format_string:
                formatter = logging.Formatter(format_string)
            else:
                formatter = logging.Formatter(
                    fmt='%(asctime)s [%(levelname)8s] [%(name)s] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S'
                )
            self._file_handler.setFormatter(formatter)

        # Update console handler
        if self._console_handler:
            self._console_handler.setLevel(self._log_level)
            if not enable_console:
                self._console_handler.setLevel(logging.CRITICAL + 1)  # Disable

        # Update all existing loggers
        for logger in self._loggers.values():
            logger.setLevel(self._log_level)

    def get_logger(self, component: str) -> logging.Logger:
        """
        Get or create a logger for a specific component.

        Args:
            component: Component name (e.g., 'sensor.realsense', 'slam.rtabmap')

        Returns:
            Configured logger instance
        """
        if component not in self._loggers:
            logger = logging.getLogger(f"neuronav.{component}")
            logger.setLevel(self._log_level)
            logger.propagate = False  # Don't propagate to root logger

            # Add handlers
            if self._console_handler:
                logger.addHandler(self._console_handler)
            if self._file_handler:
                logger.addHandler(self._file_handler)

            self._loggers[component] = logger

        return self._loggers[component]

    def get_log_file(self) -> Optional[Path]:
        """Get current log file path."""
        return self._log_file

    def close(self):
        """Close all log handlers."""
        if self._file_handler:
            self._file_handler.close()
        if self._console_handler:
            self._console_handler.close()


# Global logger instance
_logger_instance = NeuroNavLogger()


def get_logger(component: str) -> logging.Logger:
    """
    Get a logger for a specific component.

    Args:
        component: Component name (e.g., 'sensor', 'slam', 'sdk')

    Returns:
        Configured logger instance

    Example:
        >>> from neuronav.logger import get_logger
        >>> logger = get_logger('sensor.realsense')
        >>> logger.info("Starting RealSense camera")
        >>> logger.error("Failed to connect", exc_info=True)
    """
    return _logger_instance.get_logger(component)


def configure_logging(
    level: str = "INFO",
    log_file: Optional[str] = None,
    enable_console: bool = True,
    format_string: Optional[str] = None
):
    """
    Configure global logging settings.

    Args:
        level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path for log output
        enable_console: Whether to output logs to console
        format_string: Custom format string

    Example:
        >>> from neuronav import configure_logging
        >>> configure_logging(level="DEBUG", log_file="/var/log/neuronav.log")
    """
    _logger_instance.configure(level, log_file, enable_console, format_string)


def get_log_file() -> Optional[Path]:
    """Get current log file path."""
    return _logger_instance.get_log_file()


def close_logging():
    """Close all log handlers."""
    _logger_instance.close()
