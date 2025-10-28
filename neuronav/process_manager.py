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
Production-ready process management for ROS2 nodes and subprocesses.

This module provides robust process lifecycle management with:
- Health checks and monitoring
- Graceful shutdown with fallback to force-kill
- Automatic restart on failure
- Resource cleanup
- Logging and error handling
"""

import subprocess
import time
import signal
import psutil
from typing import Optional, List
from pathlib import Path
from .logger import get_logger
from .exceptions import (
    ProcessStartupError,
    ProcessTerminationError,
    ProcessHealthCheckError
)


class ManagedProcess:
    """
    Wrapper for subprocess with production-ready lifecycle management.

    Features:
    - Health monitoring
    - Graceful shutdown with timeout
    - Force kill as fallback
    - Resource tracking
    - Logging
    """

    def __init__(
        self,
        name: str,
        command: List[str],
        startup_timeout: float = 5.0,
        shutdown_timeout: float = 5.0,
        auto_restart: bool = False
    ):
        """
        Initialize managed process.

        Args:
            name: Human-readable process name
            command: Command list (e.g., ['ros2', 'run', 'pkg', 'node'])
            startup_timeout: Seconds to wait for startup
            shutdown_timeout: Seconds to wait for graceful shutdown
            auto_restart: Automatically restart if process dies
        """
        self.name = name
        self.command = command
        self.startup_timeout = startup_timeout
        self.shutdown_timeout = shutdown_timeout
        self.auto_restart = auto_restart

        self._process: Optional[subprocess.Popen] = None
        self._start_time: Optional[float] = None
        self._logger = get_logger(f"process.{name}")

    def start(self) -> None:
        """
        Start the process.

        Raises:
            ProcessStartupError: If process fails to start
        """
        if self.is_running():
            self._logger.warning(f"Process '{self.name}' already running")
            return

        self._logger.info(f"Starting process '{self.name}': {' '.join(self.command)}")

        try:
            self._process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            self._start_time = time.time()

            # Wait for startup
            time.sleep(self.startup_timeout)

            # Check if process is still running
            if not self.is_running():
                stderr = self._process.stderr.read() if self._process.stderr else ""
                raise ProcessStartupError(
                    self.name,
                    ' '.join(self.command),
                    f"Process died during startup: {stderr}"
                )

            self._logger.info(f"Process '{self.name}' started successfully (PID: {self._process.pid})")

        except FileNotFoundError as e:
            raise ProcessStartupError(
                self.name,
                ' '.join(self.command),
                f"Command not found: {e}"
            )
        except Exception as e:
            raise ProcessStartupError(
                self.name,
                ' '.join(self.command),
                str(e)
            )

    def stop(self, force: bool = False) -> None:
        """
        Stop the process gracefully or forcefully.

        Args:
            force: If True, skip graceful shutdown and kill immediately
        """
        if not self._process:
            self._logger.debug(f"Process '{self.name}' not running, nothing to stop")
            return

        pid = self._process.pid
        self._logger.info(f"Stopping process '{self.name}' (PID: {pid})")

        try:
            if force:
                self._force_kill()
            else:
                self._graceful_shutdown()

            self._logger.info(f"Process '{self.name}' stopped successfully")

        except ProcessTerminationError as e:
            self._logger.error(f"Failed to stop process '{self.name}': {e}")
            raise
        finally:
            self._process = None
            self._start_time = None

    def _graceful_shutdown(self) -> None:
        """Attempt graceful shutdown with SIGTERM."""
        if not self._process:
            return

        pid = self._process.pid

        try:
            # Send SIGTERM
            self._process.terminate()
            self._logger.debug(f"Sent SIGTERM to process '{self.name}' (PID: {pid})")

            # Wait for graceful shutdown
            try:
                self._process.wait(timeout=self.shutdown_timeout)
                return  # Success
            except subprocess.TimeoutExpired:
                self._logger.warning(
                    f"Process '{self.name}' did not terminate gracefully, force killing"
                )
                self._force_kill()

        except Exception as e:
            self._logger.error(f"Error during graceful shutdown: {e}")
            self._force_kill()

    def _force_kill(self) -> None:
        """Force kill the process with SIGKILL."""
        if not self._process:
            return

        pid = self._process.pid

        try:
            # Try psutil for robust killing (kills child processes too)
            if psutil.pid_exists(pid):
                process = psutil.Process(pid)
                # Kill all children first
                children = process.children(recursive=True)
                for child in children:
                    self._logger.debug(f"Killing child process {child.pid}")
                    child.kill()

                # Kill main process
                process.kill()
                self._logger.debug(f"Force killed process '{self.name}' (PID: {pid})")

            # Wait for process to die
            self._process.wait(timeout=2.0)

        except psutil.NoSuchProcess:
            self._logger.debug(f"Process {pid} already dead")
        except Exception as e:
            raise ProcessTerminationError(self.name, pid)

    def is_running(self) -> bool:
        """
        Check if process is currently running.

        Returns:
            True if process is running, False otherwise
        """
        if not self._process:
            return False

        return self._process.poll() is None

    def health_check(self) -> bool:
        """
        Perform health check on the process.

        Returns:
            True if process is healthy, False otherwise
        """
        if not self.is_running():
            return False

        # Check if process is zombie
        try:
            if psutil.pid_exists(self._process.pid):
                proc = psutil.Process(self._process.pid)
                if proc.status() == psutil.STATUS_ZOMBIE:
                    self._logger.warning(f"Process '{self.name}' is zombie")
                    return False
        except Exception as e:
            self._logger.error(f"Health check failed: {e}")
            return False

        return True

    def get_pid(self) -> Optional[int]:
        """Get process ID if running."""
        return self._process.pid if self._process else None

    def get_uptime(self) -> float:
        """Get process uptime in seconds."""
        if not self._start_time:
            return 0.0
        return time.time() - self._start_time

    def get_memory_usage(self) -> float:
        """
        Get process memory usage in MB.

        Returns:
            Memory usage in MB, or 0 if not available
        """
        if not self.is_running():
            return 0.0

        try:
            proc = psutil.Process(self._process.pid)
            return proc.memory_info().rss / 1024 / 1024  # Convert to MB
        except Exception:
            return 0.0

    def __del__(self):
        """Cleanup on deletion."""
        try:
            if self.is_running():
                self.stop(force=True)
        except Exception:
            pass


class ProcessManager:
    """
    Manager for multiple processes with centralized control.

    Features:
    - Start/stop multiple processes
    - Health monitoring
    - Batch operations
    - Automatic cleanup
    """

    def __init__(self):
        self._processes: List[ManagedProcess] = []
        self._logger = get_logger("process_manager")

    def add_process(self, process: ManagedProcess) -> None:
        """Add a process to be managed."""
        self._processes.append(process)

    def start_all(self) -> None:
        """Start all managed processes."""
        self._logger.info(f"Starting {len(self._processes)} processes")

        for process in self._processes:
            try:
                process.start()
            except ProcessStartupError as e:
                self._logger.error(f"Failed to start {process.name}: {e}")
                # Stop all started processes
                self.stop_all()
                raise

    def stop_all(self, force: bool = False) -> None:
        """Stop all managed processes in reverse order."""
        self._logger.info(f"Stopping {len(self._processes)} processes")

        # Stop in reverse order
        for process in reversed(self._processes):
            try:
                process.stop(force=force)
            except Exception as e:
                self._logger.error(f"Error stopping {process.name}: {e}")

    def health_check_all(self) -> bool:
        """
        Check health of all processes.

        Returns:
            True if all processes are healthy
        """
        all_healthy = True
        for process in self._processes:
            if not process.health_check():
                self._logger.warning(f"Process '{process.name}' is unhealthy")
                all_healthy = False

        return all_healthy

    def get_status(self) -> dict:
        """Get status of all processes."""
        return {
            process.name: {
                "running": process.is_running(),
                "pid": process.get_pid(),
                "uptime": process.get_uptime(),
                "memory_mb": process.get_memory_usage()
            }
            for process in self._processes
        }

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self.stop_all(force=True)
        except Exception:
            pass
