# Contributing to NeuroNav SLAM SDK

Thank you for your interest in contributing to NeuroNav SLAM SDK! This document provides guidelines for contributing to the project.

## Code of Conduct

We expect all contributors to be respectful and professional. Please be kind and courteous when interacting with others.

## Architecture Overview

NeuroNav SLAM SDK provides a **hardware-agnostic abstraction layer** over ROS2-based SLAM systems.

**Key Design Principles:**

1. **2-Line API** - Minimal code to run SLAM: `sensor = RealSenseSensor(); run_slam(sensor, RTABMapSLAM())`
2. **Adapter Pattern** - Sensors and algorithms are adapters that wrap ROS2 nodes
3. **Process Management** - `ManagedProcess` handles ROS2 node lifecycle with health checks
4. **Configuration-First** - All settings validated through dataclasses (`SensorConfig`, `SlamConfig`)
5. **Production-Ready** - Comprehensive error handling, logging, and graceful shutdown

**Architecture Layers:**

```
┌─────────────────────────────────────┐
│     User Application (2 lines)      │
├─────────────────────────────────────┤
│  run_slam() orchestration (sdk.py)  │
├──────────────┬──────────────────────┤
│ SensorBase   │  SlamBase            │← Abstract base classes
│ (sensors/)   │  (slam_base.py)      │
├──────────────┼──────────────────────┤
│ RealSense    │  RTABMapSLAM         │← Algo. implementations
│ OAK-D        │  (future: ORBSLAM3)  │
├──────────────┴──────────────────────┤
│   ManagedProcess (ROS2 lifecycle)   │
├─────────────────────────────────────┤
│   ROS2 Nodes (camera drivers, SLAM) │
└─────────────────────────────────────┘
```

**Data Flow:**

1. User calls `run_slam(sensor, slam_algorithm)`
2. SDK configures sensor and starts ROS2 camera node via `ManagedProcess`
3. Sensor publishes RGB/Depth/IMU to ROS2 topics
4. SDK configures SLAM and starts SLAM node, subscribing to sensor topics
5. SLAM processes data and publishes pose/map
6. Optional: `FoxgloveVisualizer` bridges data to WebSocket for visualization

## Getting Started

### Development Setup

1. **Clone the repository:**
```bash
git clone https://github.com/neuronav/neuronav-slam-sdk.git
cd neuronav-slam-sdk
```

2. **Install ROS2 Humble** (if not already installed):
```bash
# Follow: https://docs.ros.org/en/humble/Installation.html
```

3. **Install development dependencies:**
```bash
pip install -e ".[dev]"
```

4. **Install camera drivers** (choose one):
```bash
# RealSense
sudo apt install ros-humble-realsense2-camera

# OAK-D
sudo apt install ros-humble-depthai-ros
```

5. **Install RTAB-Map:**
```bash
sudo apt install ros-humble-rtabmap-ros
```

6. **Install pre-commit hooks** (optional but recommended):
```bash
pre-commit install
```

## Development Workflow

### Branch Naming

- `feature/description` - New features
- `fix/description` - Bug fixes
- `docs/description` - Documentation changes
- `refactor/description` - Code refactoring
- `test/description` - Test additions or modifications

### Commit Messages

Follow conventional commits format:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `refactor`: Code refactoring
- `test`: Test additions/modifications
- `chore`: Build process or auxiliary tool changes
- `perf`: Performance improvements

Examples:
```
feat(sensor): add ZED camera support
fix(process): handle zombie processes in health check
docs(readme): update installation instructions
```

### Code Style

- Follow PEP 8 style guidelines
- Use type hints for function parameters and return values
- Maximum line length: 100 characters
- Use docstrings for all public functions, classes, and modules

Format code with Black:
```bash
black neuronav/
```

Lint code with flake8:
```bash
flake8 neuronav/
```

Type check with mypy:
```bash
mypy neuronav/
```

### Testing

Run tests before submitting PR:

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=neuronav --cov-report=html

# Run specific test file
pytest tests/test_sensors.py

# Run with verbose output
pytest -v
```

### Adding New Features

1. **Sensors**: Extend `SensorBase` class in `neuronav/sensors/`
   - Implement all abstract methods
   - Add comprehensive error handling
   - Include health check implementation
   - Add integration tests

2. **SLAM Algorithms**: Extend `SlamBase` class in `neuronav/algorithms/`
   - Implement all abstract methods
   - Handle tracking loss gracefully
   - Add map save/load functionality
   - Include integration tests

3. **Documentation**: Update relevant documentation
   - Add docstrings to all public APIs
   - Update README.md if user-facing
   - Add examples if applicable

### Pull Request Process

1. Create a feature branch from `main`
2. Make your changes with clear commit messages
3. Add/update tests for your changes
4. Update documentation as needed
5. Ensure all tests pass: `pytest`
6. Ensure code is formatted: `black neuronav/`
7. Submit pull request with description of changes

PR Requirements:
- [ ] All tests pass
- [ ] Code is formatted with Black
- [ ] Type hints are added
- [ ] Docstrings are complete
- [ ] Documentation is updated
- [ ] No breaking changes (or clearly marked if unavoidable)

### Review Process

- At least one maintainer review required
- CI checks must pass
- Address all review comments
- Squash commits before merge (if requested)

## Project Structure

```
neuronav-slam-sdk/
├── neuronav/                      # Main SDK package
│   ├── __init__.py                # Public API exports
│   ├── __version__.py             # Version information
│   ├── sdk.py                     # Core run_slam() function
│   ├── sensors/                   # Sensor adapters
│   │   ├── __init__.py
│   │   ├── base.py                # SensorBase, SensorConfig
│   │   ├── realsense.py           # RealSenseSensor (D435i, D455, D415)
│   │   └── oakd.py                # OAKDSensor (OAK-D Pro/W)
│   ├── algorithms/                # SLAM algorithm implementations
│   │   ├── __init__.py
│   │   └── rtabmap_slam.py        # RTABMapSLAM wrapper
│   ├── slam_base.py               # SlamBase, SlamConfig, SlamStatus
│   ├── process_manager.py         # ManagedProcess for ROS2 nodes
│   ├── logger.py                  # Logging utilities
│   ├── exceptions.py              # Custom exception hierarchy
│   └── visualization.py           # FoxgloveVisualizer
├── examples/                      # Usage examples
│   ├── simple_realsense.py
│   ├── simple_oakd.py
│   ├── with_visualization.py
│   ├── custom_config.py
│   └── add_new_sensor.py          # Template for new sensors
├── docker/                        # Docker setup
│   ├── Dockerfile
│   ├── docker_build.sh
│   ├── docker_run.sh
│   └── docker-compose.yml
├── tests/                         # Test suite (pytest)
├── setup.py                       # Package installation
├── README.md                      # Quick start guide
├── CONTRIBUTING.md                # This file
├── ACKNOWLEDGMENTS.md             # Third-party credits
├── THIRD-PARTY-NOTICES.md         # License notices
└── LICENSE                        # Apache 2.0 license
```

## Adding Support for New Hardware

### Adding a New Sensor

See [examples/add_new_sensor.py](examples/add_new_sensor.py) for a complete template.

**Step-by-step process:**

1. **Create a new file** in `neuronav/sensors/` (e.g., `zed.py`)

2. **Extend `SensorBase`** class from `neuronav.sensors.base`:
   ```python
   from .base import SensorBase, SensorConfig
   from ..process_manager import ManagedProcess
   from ..logger import get_logger
   from ..exceptions import SensorError, SensorInitializationError

   class ZEDSensor(SensorBase):
       def __init__(self):
           super().__init__()
           self._process = None
           self._logger = get_logger(__name__)
   ```

3. **Implement all abstract methods:**
   - `configure(config: SensorConfig)` - Validate and store config
   - `start()` - Launch ROS2 node using `ManagedProcess`
   - `stop()` - Cleanup and terminate processes
   - `get_sensor_name()` - Return human-readable name
   - `get_ros_topics()` - Return dict of ROS2 topic names

4. **Use `ManagedProcess` for ROS2 nodes:**
   ```python
   self._process = ManagedProcess(
       name="zed_camera",
       command=["ros2", "launch", "zed_wrapper", "zed.launch.py"],
       env=os.environ.copy(),
       logger=self._logger
   )
   self._process.start()
   ```

5. **Add comprehensive error handling:**
   - Raise `ConfigurationError` for invalid configs
   - Raise `SensorNotFoundError` if hardware not detected
   - Raise `SensorInitializationError` if startup fails
   - Use try/except in `stop()` for graceful cleanup

6. **Export in `neuronav/sensors/__init__.py`:**
   ```python
   from .zed import ZEDSensor
   __all__ = [..., "ZEDSensor"]
   ```

7. **Add integration tests** in `tests/test_sensors.py`

8. **Update documentation:**
   - Add to README.md supported hardware list
   - Update ACKNOWLEDGMENTS.md with hardware/SDK credits
   - Add example in `examples/simple_zed.py`

### Adding a New SLAM Algorithm

**Step-by-step process:**

1. **Create a new file** in `neuronav/algorithms/` (e.g., `orbslam3.py`)

2. **Extend `SlamBase`** class from `neuronav.slam_base`:
   ```python
   from ..slam_base import SlamBase, SlamConfig, SlamStatus
   from ..process_manager import ManagedProcess
   from ..logger import get_logger
   from ..exceptions import SlamError, SlamInitializationError

   class ORBSLAM3(SlamBase):
       def __init__(self):
           super().__init__()
           self._process = None
           self._logger = get_logger(__name__)
           self.status = SlamStatus.IDLE
   ```

3. **Implement all abstract methods:**
   - `configure(config: SlamConfig)` - Validate and store config
   - `start()` - Launch SLAM nodes using `ManagedProcess`
   - `stop()` - Cleanup and save map if needed
   - `get_algorithm_name()` - Return human-readable name
   - `get_version()` - Return version string
   - `get_current_pose()` - Return current robot pose
   - `save_map(path)` - Save map to file
   - `load_map(path)` - Load existing map

4. **Manage ROS2 topics:**
   ```python
   def set_sensor_topics(self, topics: Dict[str, str]):
       """Configure which sensor topics to subscribe to"""
       self._rgb_topic = topics.get('rgb')
       self._depth_topic = topics.get('depth')
       self._camera_info_topic = topics.get('camera_info')
   ```

5. **Track SLAM status** using `SlamStatus` enum:
   - `IDLE` - Not started
   - `INITIALIZING` - Starting up
   - `TRACKING` - Actively tracking
   - `LOST` - Lost tracking
   - `SHUTDOWN` - Shutting down

6. **Export in `neuronav/algorithms/__init__.py`:**
   ```python
   from .orbslam3 import ORBSLAM3
   __all__ = [..., "ORBSLAM3"]
   ```

7. **Add tests** in `tests/test_algorithms.py`

8. **Update documentation:**
   - Add to README.md supported algorithms list
   - Update ACKNOWLEDGMENTS.md with algorithm credits
   - Add example in `examples/simple_orbslam3.py`

## Testing Guidelines

### Unit Tests
- Test individual components in isolation
- Mock external dependencies (ROS2, hardware)
- Fast execution (< 1s per test)

### Integration Tests
- Test component interactions
- Use test fixtures for ROS2 environment
- Moderate execution time (< 10s per test)

### System Tests
- Test full SLAM pipeline
- Require actual hardware or recorded datasets
- Longer execution time acceptable

### Test Structure
```python
import pytest
from neuronav import RealSenseSensor, SensorConfig
from neuronav.exceptions import ConfigurationError

def test_sensor_configuration():
    """Test sensor configuration validation."""
    sensor = RealSenseSensor()
    config = SensorConfig(rgb_width=640, rgb_height=480)
    sensor.configure(config)
    assert sensor.is_configured

def test_invalid_configuration():
    """Test that invalid configuration raises error."""
    with pytest.raises(ConfigurationError):
        SensorConfig(fps=-1)
```

## Documentation Guidelines

### Docstring Format

Use Google-style docstrings:

```python
def function_name(param1: str, param2: int) -> bool:
    """
    Short description of function.

    Longer description if needed, explaining behavior,
    edge cases, and important details.

    Args:
        param1: Description of param1
        param2: Description of param2

    Returns:
        Description of return value

    Raises:
        ValueError: Description of when this is raised
        RuntimeError: Description of when this is raised

    Example:
        >>> result = function_name("test", 42)
        >>> print(result)
        True
    """
```

## Release Process

Maintainers only:

1. Update version in `setup.py` and `neuronav/__version__.py`
2. Update `CHANGELOG.md`
3. Create release tag: `git tag -a v1.2.3 -m "Release v1.2.3"`
4. Push tag: `git push origin v1.2.3`
5. GitHub Actions will build and publish to PyPI

## Getting Help

- **Documentation**: [docs.neuronav.io](https://docs.neuronav.io)
- **Issues**: For bugs or feature requests
- **Discussions**: For questions and general discussion
- Check existing issues before creating new ones
- Provide minimal reproducible examples for bugs

## License

By contributing, you agree that your contributions will be licensed under Apache 2.0 license.

See [LICENSE](LICENSE) for details.

## Recognition

Contributors will be recognized in:
- CHANGELOG.md for their contributions
- README.md contributors section
- Release notes

## Key Technologies

- **RTAB-Map** (BSD-3) - Copyright (c) 2010-2025, Mathieu Labbé - IntRoLab - Université de Sherbrooke
- **ROS2 Humble**, **Intel RealSense SDK** (Apache-2.0), **Luxonis DepthAI** (MIT), **Foxglove Bridge** (MPL-2.0)

See [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md) and [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md) for details.

## Questions?

- **Documentation**: [docs.neuronav.io](https://docs.neuronav.io)
- **Issues**: [GitHub Issues](https://github.com/neuronav/neuronav-slam-sdk/issues)
- **Discussions**: For questions and general discussion

Thank you for contributing to NeuroNav SLAM SDK!
