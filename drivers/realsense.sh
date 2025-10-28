#!/bin/bash
set -e

echo "🔧 Updating and installing dependencies..."
apt-get update && apt-get install -y \
    git cmake build-essential \
    libusb-1.0-0-dev libssl-dev pkg-config \
    libglfw3-dev libgl1-mesa-dev libgtk-3-dev \
    libudev-dev libjpeg-dev curl wget \
    python3-pip

echo "📦 Cloning and building librealsense v2.56.1 (no DKMS)..."
cd /root
rm -rf librealsense
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.54.1
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DFORCE_LIBUVC=true
make -j$(nproc)
make install
ldconfig

echo "📦 Setting up ROS 2 workspace and realsense2_camera wrapper v2.56.1..."
source /opt/ros/humble/setup.bash
mkdir -p /ws/src
cd /ws/src
rm -rf realsense-ros
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout 4.54.1

cd /ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

echo "✅ Sourcing environment setup..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /ws/install/setup.bash" >> ~/.bashrc

echo "✅ RealSense SDK v2.56.1 and ROS2 wrapper installed successfully."
