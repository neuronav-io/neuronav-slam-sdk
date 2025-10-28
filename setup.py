# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

from setuptools import setup, find_packages
from pathlib import Path

# Read version from __version__.py
version_file = Path(__file__).parent / "neuronav" / "__version__.py"
version_dict = {}
exec(version_file.read_text(), version_dict)
VERSION = version_dict["__version__"]

# Read long description from README
readme_file = Path(__file__).parent / "README.md"
with open(readme_file, "r", encoding="utf-8") as fh:
    long_description = fh.read()

# Core dependencies
install_requires = [
    "pyyaml>=6.0",
    "numpy>=1.20.0,<2.0",
    "opencv-python>=4.5.0",
    "psutil>=5.8.0",  # For process management
]

# Development dependencies
dev_requires = [
    "pytest>=7.0",
    "pytest-cov>=3.0",
    "pytest-timeout>=2.1",
    "pytest-mock>=3.6",
    "black>=22.0",
    "flake8>=4.0",
    "mypy>=0.950",
    "isort>=5.10",
    "pre-commit>=2.15",
    "bandit>=1.7",  # Security linting
    "pip-audit>=2.0",  # Dependency vulnerability scanning
]

# Optional sensor-specific dependencies
realsense_requires = [
    "pyrealsense2>=2.50.0",
]

# Documentation dependencies
docs_requires = [
    "sphinx>=4.0",
    "sphinx-rtd-theme>=1.0",
    "sphinx-autodoc-typehints>=1.12",
]

setup(
    name="neuronav-slam-sdk",
    version=VERSION,
    author="NeuroNav Team",
    author_email="eldaniz@neuronav.io",
    description="Production-ready SLAM SDK for robotics with 2-line API. Built on RTAB-Map, supports Intel RealSense & Luxonis OAK-D.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/neuronav/neuronav-slam-sdk",
    project_urls={
        "Documentation": "https://docs.neuronav.io",
        "Bug Tracker": "https://github.com/neuronav/neuronav-slam-sdk/issues",
        "Source Code": "https://github.com/neuronav/neuronav-slam-sdk",
        "Changelog": "https://github.com/neuronav/neuronav-slam-sdk/blob/main/CHANGELOG.md",
        "Acknowledgments": "https://github.com/neuronav/neuronav-slam-sdk/blob/main/ACKNOWLEDGMENTS.md",
    },
    packages=find_packages(exclude=["tests", "tests.*", "examples", "docs"]),
    include_package_data=True,
    classifiers=[
        # Development status
        "Development Status :: 4 - Beta",

        # Intended audience
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Manufacturing",

        # Topics
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Image Recognition",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Hardware :: Universal Serial Bus (USB) :: Camera",

        # License
        "License :: OSI Approved :: Apache Software License",

        # Python versions
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3 :: Only",

        # OS
        "Operating System :: POSIX :: Linux",

        # Other
        "Environment :: Console",
        "Natural Language :: English",
        "Typing :: Typed",
    ],
    keywords=[
        "slam", "robotics", "ros2", "rtabmap", "realsense", "oakd",
        "computer-vision", "mapping", "localization", "3d-reconstruction",
        "visual-slam", "depth-camera", "sensor-fusion"
    ],
    python_requires=">=3.8",
    install_requires=install_requires,
    extras_require={
        "dev": dev_requires,
        "realsense": realsense_requires,
        "docs": docs_requires,
        "all": dev_requires + realsense_requires + docs_requires,
    },
    entry_points={
        "console_scripts": [
            "neuronav=neuronav.cli:cli",
        ],
    },
    package_data={
        "neuronav": [
            "py.typed",  # PEP 561 marker for type hints
        ],
    },
    # Production metadata
    zip_safe=False,  # Don't install as zip for better debugging
    platforms=["Linux"],

    # Security
    license="Apache-2.0",
    license_files=["LICENSE"],
)
