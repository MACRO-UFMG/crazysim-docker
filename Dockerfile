# ROS2 desktop full base image with additional linux utils
FROM osrf/ros:humble-desktop-full AS ros2-base
ENV DEBIAN_FRONTEND noninteractive
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

RUN apt-get update && apt-get install -y \
    cmake build-essential lsb-release curl gnupg usbutils \
    libqt5x11extras5 libxcb-xinerama0 libxcb-cursor0 python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && pip3 install Jinja2
RUN ln -sf /usr/bin/python3 /usr/bin/python

# Install Gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null 

RUN apt-get update && apt-get install -y gz-garden && rm -rf /var/lib/apt/lists/*

# Clone CrazySim
RUN git clone https://github.com/gtfactslab/CrazySim.git --recursive
WORKDIR /CrazySim
RUN apt remove -y python3-packaging python3-numpy

RUN cd crazyflie-lib-python && \
    SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 pip install -e .

RUN mkdir -p crazyflie-firmware/sitl_make/build && \
    cd crazyflie-firmware/sitl_make/build && \
    cmake .. && make all

WORKDIR /
# Install cfclient
RUN git clone https://github.com/llanesc/crazyflie-clients-python && \
    cd crazyflie-clients-python && pip3 install -e .

# --- ROS FIXES START HERE ---
SHELL ["/bin/bash", "-c"]

# 1. Install missing CLI tools and rosdep
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros2cli \
    ros-humble-ros2topic \
    ros-humble-ros2launch \
    ros-humble-ros2service \
    ros-humble-ament-cmake \
    ros-humble-builtin-interfaces \
    ros-humble-rosidl-default-generators \
    ros-humble-joy \
    ros-humble-tf-transformations \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# 2. Initialize rosdep
RUN rosdep init || true && rosdep update

WORKDIR /CrazySim/crazyswarm2_ws/src
RUN git clone --recursive https://github.com/IMRCLab/motion_capture_tracking.git

WORKDIR /CrazySim/crazyswarm2_ws
# 3. Ensure apt-get update runs in the same layer as rosdep install
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --force-reinstall "numpy<2.0"
RUN pip3 install "rowan"
RUN pip3 install "nicegui==1.4.22"

# 4. Automate sourcing for 'docker exec'
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /CrazySim/crazyswarm2_ws/install/setup.bash ]; then source /CrazySim/crazyswarm2_ws/install/setup.bash; fi" >> ~/.bashrc

WORKDIR /CrazySim/crazyflie-firmware
CMD ["bash", "tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh", "-n", "4", "-m", "crazyflie"]