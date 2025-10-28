#!/bin/bash

echo "Building NeuroNav RTAB-Map Docker image..."
echo "========================================="

# Build the Docker image
docker build -t neuronav/rtabmap:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo ""
    echo "To run the container, use:"
    echo "  ./docker_run.sh"
    echo ""
    echo "Or with docker-compose:"
    echo "  docker-compose up"
else
    echo ""
    echo "Build failed! Please check the error messages above."
    exit 1
fi
