#!/usr/bin/env python3
"""
Simple OAK-D Pro SLAM Example

This demonstrates the 2-line SDK usage for running SLAM
with an OAK-D Pro camera and RTAB-Map algorithm.
"""

from neuronav import OAKDSensor, RTABMapSLAM, run_slam

# Line 1: Choose your sensor
sensor = OAKDSensor()

# Line 2: Run SLAM with your chosen algorithm
run_slam(sensor, RTABMapSLAM())
