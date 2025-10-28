# Copyright 2025 NeuroNav Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# SPDX-License-Identifier: Apache-2.0

"""Version information for NeuroNav SLAM SDK."""

__version__ = "0.1.0"
__version_info__ = tuple(int(x) for x in __version__.split("."))

# Semantic versioning
MAJOR = __version_info__[0]
MINOR = __version_info__[1]
PATCH = __version_info__[2] if len(__version_info__) > 2 else 0

# Release information
__author__ = "NeuroNav Team"
__license__ = "Apache-2.0"
__copyright__ = "Copyright 2025 NeuroNav Team"
