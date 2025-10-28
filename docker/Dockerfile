# Base image with ROS2 Humble
FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    zip \
    unzip \
    tar \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    software-properties-common \
    lsb-release \
    gnupg2 \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y \
build-essential cmake curl unzip git pkg-config \
&& rm -rf /var/lib/apt/lists/*

# run local bash script to install realsense2 dependencies
COPY realsense.sh /tmp/install_realsense_dependencies.sh
RUN chmod +x /tmp/install_realsense_dependencies.sh && \
    /tmp/install_realsense_dependencies.sh

# run local bash script to install OAK-D Pro dependencies
COPY oakd_pro.sh /tmp/install_oakd_pro_dependencies.sh
RUN chmod +x /tmp/install_oakd_pro_dependencies.sh && \
    /tmp/install_oakd_pro_dependencies.sh


# Install ROS2 packages for RTAB-Map and RealSense
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rtabmap \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-msgs \
    ros-${ROS_DISTRO}-rtabmap-launch \
    ros-${ROS_DISTRO}-rtabmap-viz \
    ros-${ROS_DISTRO}-rtabmap-examples \
    # install rclpy packages
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-image-proc \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-message-filters \
    && rm -rf /var/lib/apt/lists/*

# Install Nav2 and Foxglove Bridge packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-amcl \
    ros-${ROS_DISTRO}-nav2-behavior-tree \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-core \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-dwb-controller \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-behaviors \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-nav2-smac-planner \
    ros-${ROS_DISTRO}-nav2-theta-star-planner \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-waypoint-follower \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-collision-monitor \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    ros-${ROS_DISTRO}-nav2-rotation-shim-controller \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools and libraries
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libflann-dev \
    libvtk9-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libglu1-mesa-dev \
    libgl1-mesa-dev \
    libglfw3-dev \
    libglew-dev \
    mesa-utils \
    libxi-dev \
    libxmu-dev \
    libglu1-mesa \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install \
    pyyaml>=6.0 \
    numpy>=1.20.0 \
    opencv-python>=4.5.0 \
    pyrealsense2>=2.50.0 \
    transforms3d \
    scipy \
    matplotlib

# Create workspace
RUN mkdir -p /root/neuronav_ws/src
WORKDIR /root/neuronav_ws

# Copy the neuronav package
COPY . /root/neuronav_ws/src/neuronav_rtabmap

RUN pip3 install --upgrade "numpy<2"

# Install neuronav package
RUN cd /root/neuronav_ws/src/neuronav_rtabmap && \
    pip3 install -e .

# Setup entrypoint
RUN echo '#!/bin/bash\n\
    # Create XDG_RUNTIME_DIR if it doesn't exist\n\
    if [ -z "$XDG_RUNTIME_DIR" ]; then\n\
    export XDG_RUNTIME_DIR=/tmp/runtime-root\n\
    mkdir -p $XDG_RUNTIME_DIR\n\
    chmod 700 $XDG_RUNTIME_DIR\n\
    fi\n\
    source /opt/ros/${ROS_DISTRO}/setup.bash\n\
    source /root/neuronav_ws/install/setup.bash 2>/dev/null || true\n\
    source /ws/install/setup.bash 2>/dev/null || true\n\
    source /ws_oak/install/setup.bash 2>/dev/null || true\n\
    exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Force numpy 1.x for cv_bridge compatibility (must be after all pip installs)
RUN pip3 install --force-reinstall "numpy<2"

# Set up udev rules for RealSense (will need to be run with privileged mode)
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/99-realsense-libusb.rules

# Expose ROS2 default discovery port
EXPOSE 11311

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
