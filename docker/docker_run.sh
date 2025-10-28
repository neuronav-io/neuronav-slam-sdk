#!/bin/bash

echo "Starting NeuroNav RTAB-Map Docker container..."
echo "============================================="

# Allow X11 connections
xhost +local:docker 2>/dev/null || true

# Create workspace directory if it doesn't exist
mkdir -p workspace

# Run the Docker container
docker run -it --rm \
    --name neuronav_rtabmap \
    --privileged \
    --network host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID=0 \
    --env XDG_RUNTIME_DIR=/tmp/runtime-root \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --env MESA_GL_VERSION_OVERRIDE=4.5 \
    --add-host=host.docker.internal:host-gateway \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume /dev:/dev \
    --volume $(pwd)/workspace:/root/workspace \
    --volume neuronav_rtabmap_data:/root/.ros \
    --device /dev/dri:/dev/dri \
    --device /dev/bus/usb:/dev/bus/usb \
    neuronav/rtabmap:latest

# Revoke X11 permissions
xhost -local:docker 2>/dev/null || true
