#!/bin/bash
set -e

echo "🔧 Updating and installing dependencies..."
apt-get update && apt-get install -y \
    git cmake build-essential \
    libusb-1.0-0-dev libssl-dev pkg-config \
    libglfw3-dev libgl1-mesa-dev libgtk-3-dev \
    libudev-dev libjpeg-dev curl wget \
    python3-pip python3-dev \
    libopencv-dev \
    usbutils

echo "📦 Installing DepthAI Core library..."
cd /root
rm -rf depthai-core
git clone --recursive https://github.com/luxonis/depthai-core.git
cd depthai-core
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j$(nproc)
make install
ldconfig

echo "📦 Installing DepthAI Python SDK..."
pip3 install --upgrade pip
pip3 install depthai opencv-python numpy

echo "📦 Setting up ROS 2 workspace and depthai-ros..."
source /opt/ros/humble/setup.bash
mkdir -p /ws_oak/src
cd /ws_oak/src
rm -rf depthai-ros
git clone --recursive https://github.com/luxonis/depthai-ros.git
cd depthai-ros
git checkout humble

cd /ws_oak
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo "🔧 Setting up udev rules for OAK-D Pro..."
mkdir -p /etc/udev/rules.d/
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/80-movidius.rules
# Skip udevadm in Docker build environment
if [ -S /run/udev/control ]; then
    udevadm control --reload-rules && udevadm trigger
fi

echo "✅ Sourcing environment setup..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /ws_oak/install/setup.bash" >> ~/.bashrc

echo "✅ DepthAI SDK and ROS2 depthai-ros installed successfully for OAK-D Pro."
echo ""
echo "📌 Note: To use OAK-D Pro, you may need to:"
echo "   1. Reconnect the camera if it's already plugged in"
echo "   2. Run 'source /ws_oak/install/setup.bash' in your terminal"
echo "   3. Launch with: ros2 launch depthai_ros_driver camera.launch.py"
