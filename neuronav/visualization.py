"""
Foxglove Bridge visualization for SLAM data
"""

import subprocess
from typing import Optional


class FoxgloveVisualizer:
    """Launch Foxglove Bridge for web-based visualization"""

    def __init__(self, port: int = 8765):
        """
        Initialize Foxglove visualizer

        Args:
            port: WebSocket port for Foxglove Bridge (default: 8765)
        """
        self.port = port
        self._process: Optional[subprocess.Popen] = None

    def start(self):
        """Start Foxglove Bridge"""
        cmd = [
            'ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml',
            f'port:={self.port}'
        ]

        self._process = subprocess.Popen(cmd)

        print(f"🦊 Foxglove Bridge started!")
        print(f"   WebSocket: ws://localhost:{self.port}")
        print(f"   Open Foxglove Studio: https://foxglove.dev/studio")
        print(f"   Connect to: ws://localhost:{self.port}")

    def stop(self):
        """Stop Foxglove Bridge"""
        if self._process:
            self._process.terminate()
            self._process.wait(timeout=5)
            self._process = None
        print("🛑 Foxglove Bridge stopped")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


def visualize(port: int = 8765):
    """
    Start Foxglove Bridge for SLAM visualization

    Args:
        port: WebSocket port (default: 8765)

    Example:
        >>> from neuronav import visualize
        >>> visualize()  # Starts on port 8765

    Then open Foxglove Studio and connect to ws://localhost:8765
    """
    viz = FoxgloveVisualizer(port)
    viz.start()

    try:
        import time
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Stopping Foxglove Bridge...")
        viz.stop()


__all__ = ['FoxgloveVisualizer', 'visualize']
