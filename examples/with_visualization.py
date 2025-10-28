from neuronav import OAKDSensor, RTABMapSLAM, run_slam

# Define your sensor
sensor = OAKDSensor()

# Run SLAM with visualization
run_slam(sensor, RTABMapSLAM(), visualize=True)
