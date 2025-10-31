#!/bin/bash

echo "Building NeuroNav RTAB-Map Docker image..."
echo "========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Go to the parent directory (project root)
cd "$SCRIPT_DIR/.."

# Build the Docker image with Dockerfile from docker/ folder
docker build -f docker/Dockerfile -t neuronav/rtabmap:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo ""
    echo "To run the container, use:"
    echo "  ./docker/docker_run.sh"  # Updated path
    echo ""
    echo "Or with docker-compose:"
    echo "  docker-compose -f docker/docker-compose.yml up"  # Updated path
else
    echo ""
    echo "Build failed! Please check the error messages above."
    exit 1
fi
