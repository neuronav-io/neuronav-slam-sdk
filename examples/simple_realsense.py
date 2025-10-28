#!/usr/bin/env python3
"""
Simple RealSense SLAM Example

This demonstrates the 2-line SDK usage for running SLAM
with a RealSense camera and RTAB-Map algorithm.
"""

from neuronav import RealSenseSensor, RTABMapSLAM, run_slam

# Line 1: Choose your sensor
sensor = RealSenseSensor()

# Line 2: Run SLAM with your chosen algorithm
run_slam(sensor, RTABMapSLAM())
