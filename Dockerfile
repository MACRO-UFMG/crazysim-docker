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

RUN pip3 install --upgrade pip && \
    pip3 install Jinja2 && \
    pip3 install rowan && \
    pip3 install "nicegui==1.4.22" && \
    pip3 install transforms3d
RUN ln -sf /usr/bin/python3 /usr/bin/python

# Install Gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null 

RUN apt-get update && apt-get install -y gz-garden && rm -rf /var/lib/apt/lists/*

# Clone CrazySim
RUN git clone https://github.com/gtfactslab/CrazySim.git && \
    cd CrazySim && \
    git checkout 7dbacd9d8280379c65ffb71180e4f134d201a202 && \
    git submodule update --init --recursive

WORKDIR /CrazySim

RUN cd crazyflie-lib-python && \
    SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 pip install -e .

RUN mkdir -p crazyflie-firmware/sitl_make/build && \
    cd crazyflie-firmware/sitl_make/build && \
    cmake .. && make all

WORKDIR /
# Install cfclient
RUN git clone https://github.com/llanesc/crazyflie-clients-python && \
    cd crazyflie-clients-python && pip3 install -e .

WORKDIR /CrazySim/crazyswarm2_ws/src
RUN git clone --recursive https://github.com/IMRCLab/motion_capture_tracking.git

# --- ROS FIXES START HERE ---
SHELL ["/bin/bash", "-c"]

WORKDIR /CrazySim/crazyswarm2_ws
# 3. Ensure apt-get update runs in the same layer as rosdep install
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install "numpy<2.0.0" --force-reinstall

WORKDIR /CrazySim/crazyswarm2_ws

# 4. Automate sourcing for 'docker exec'
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /CrazySim/crazyswarm2_ws/install/setup.bash ]; then source /CrazySim/crazyswarm2_ws/install/setup.bash; fi" >> ~/.bashrc

WORKDIR /CrazySim/crazyflie-firmware

CMD ["tail", "-f", "/dev/null"]