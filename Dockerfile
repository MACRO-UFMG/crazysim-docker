# ROS2 desktop full base image with additional linux utils
FROM osrf/ros:humble-desktop-full AS ros2-base
ENV DEBIAN_FRONTEND noninteractive

# Install essential CLI tools and networking utilities for debugging
RUN apt-get update && apt-get install -y \
    git x11vnc wget unzip xvfb icewm tree dos2unix vim \
    net-tools iputils-ping iproute2 iptables tcpdump nano tmux && \
    rm -rf /var/lib/apt/lists/*

# Docker image for crazysim
FROM ros2-base AS crazysim
ENV DEBIAN_FRONTEND noninteractive

# Add Environment Variables here so 'ros2' is always in the PATH
ENV PATH="/opt/ros/humble/bin:${PATH}"
ENV ROS_DISTRO=humble

# Install build dependencies and GUI-related libraries for Gazebo/Qt
RUN apt-get update && apt-get install -y \
    cmake build-essential lsb-release curl gnupg usbutils \
    libqt5x11extras5 libxcb-xinerama0 libxcb-cursor0 python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies. 
# NOTE: nicegui is currently pinned to 1.4.22 to maintain compatibility with 
# the current CrazySim UI code. Future updates should target nicegui 2.x.
RUN pip3 install --upgrade pip && \
    pip3 install Jinja2 rowan "nicegui==1.4.22" transforms3d

# Ensure 'python' command points to 'python3' for legacy script compatibility
RUN ln -sf /usr/bin/python3 /usr/bin/python

# Install Gazebo Garden
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null 

RUN apt-get update && apt-get install -y gz-garden && rm -rf /var/lib/apt/lists/*

# Clone CrazySim - IMPORTANT: As CrazySim evolves, this specific commit hash 
# (7dbacd9) should be updated to leverage newer features and bug fixes.
RUN git clone https://github.com/gtfactslab/CrazySim.git && \
    cd CrazySim && \
    git checkout 7dbacd9d8280379c65ffb71180e4f134d201a202 && \
    git submodule update --init --recursive

WORKDIR /CrazySim

# Install crazyflie-lib-python (cflib). 
# NOTE: Since cflib now supports the CrazySim udpdriver implementation, 
# this local 'pip install -e .' could be replaced with 'pip install cflib' 
# in future iterations to simplify the build process.
RUN cd crazyflie-lib-python && \
    SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 pip install -e .

# Build the crazyflie firmware for Software-In-The-Loop (SITL)
RUN mkdir -p crazyflie-firmware/sitl_make/build && \
    cd crazyflie-firmware/sitl_make/build && \
    cmake .. && make all

WORKDIR /
# Install crazyflie-clients-python for ground control interface
RUN git clone https://github.com/llanesc/crazyflie-clients-python && \
    cd crazyflie-clients-python && pip3 install -e .

WORKDIR /CrazySim/crazyswarm2_ws/src
RUN git clone --recursive https://github.com/IMRCLab/motion_capture_tracking.git

SHELL ["/bin/bash", "-c"]

WORKDIR /CrazySim/crazyswarm2_ws
# Install ROS dependencies via rosdep and build the workspace
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# CRITICAL FIX: Reinstalling NumPy < 2.0.0
# While cflib and newer python packages may pull NumPy 2.x by default, 
# ROS Humble's core libraries and the current nicegui/matplotlib integration 
# in this project are not yet compatible with the NumPy 2.0 ABI. 
# This force-reinstall ensures the "AttributeError: _ARRAY_API not found" is avoided.
RUN pip3 install "numpy<2.0.0" --force-reinstall

WORKDIR /CrazySim/crazyswarm2_ws

# Automate workspace sourcing for interactive bash sessions (docker exec)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /CrazySim/crazyswarm2_ws/install/setup.bash ]; then source /CrazySim/crazyswarm2_ws/install/setup.bash; fi" >> ~/.bashrc

WORKDIR /CrazySim/crazyflie-firmware

# Keep container alive
CMD ["tail", "-f", "/dev/null"]