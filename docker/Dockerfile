# Multi-stage Dockerfile for Autonomous Drone Simulation
FROM ubuntu:22.04 AS base

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# Set timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    apt-utils \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    git \
    vim \
    nano \
    build-essential \
    cmake \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# Stage: ROS2 Humble Installation
# ============================================
FROM base AS ros2-base

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ============================================
# Stage: ArduPilot SITL Installation
# ============================================
FROM ros2-base AS ardupilot-base

# Install ArduPilot dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    libxml2-dev \
    libxslt1-dev \
    ccache \
    gawk \
    git \
    make \
    wget \
    libexpat1-dev \
    g++ \
    gcc \
    gdb \
    valgrind \
    && rm -rf /var/lib/apt/lists/*

# Clone ArduPilot
WORKDIR /opt
RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
WORKDIR /opt/ardupilot

# Checkout stable version
RUN git checkout Copter-4.3

# Install ArduPilot prerequisites
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduPilot SITL
RUN ./waf configure --board sitl
RUN ./waf copter

# Add ArduPilot to PATH
ENV PATH="/opt/ardupilot/Tools/autotest:${PATH}"
ENV PATH="/opt/ardupilot/build/sitl/bin:${PATH}"

# ============================================
# Stage: AirSim Installation
# ============================================
FROM ardupilot-base AS airsim-base

# Install AirSim dependencies
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone and build AirSim
WORKDIR /opt
RUN git clone https://github.com/Microsoft/AirSim.git
WORKDIR /opt/AirSim

# Build AirSim
RUN ./setup.sh
RUN ./build.sh

# Build AirSim Python API
WORKDIR /opt/AirSim/PythonClient
RUN pip3 install -e .

# Set AirSim environment variables
ENV AIRSIM_ROOT=/opt/AirSim

# ============================================
# Stage: Python ML Environment
# ============================================
FROM airsim-base AS ml-base

# Install Python ML dependencies
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip setuptools wheel

# Install PyTorch (CPU version for minimal setup, can be changed to GPU)
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Copy requirements file
COPY requirements.txt /tmp/requirements.txt

# Install Python packages
RUN pip3 install -r /tmp/requirements.txt

# ============================================
# Stage: Final Development Environment
# ============================================
FROM ml-base AS final

# Install additional utilities
RUN apt-get update && apt-get install -y \
    tmux \
    htop \
    tree \
    net-tools \
    iputils-ping \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Copy project files
COPY . /workspace/

# Install drone_gym package in development mode
RUN pip3 install -e /workspace/

# Setup environment variables
ENV PYTHONPATH="/workspace:${PYTHONPATH}"
ENV MAVLINK20=1

# Create data directories
RUN mkdir -p /workspace/data/{logs,recordings,checkpoints}

# Expose ports
# MAVLink: 14550 (UDP), 14551 (UDP)
# AirSim: 41451 (API), 8080 (Dashboard)
# TensorBoard: 6006
EXPOSE 14550/udp 14551/udp 41451 8080 6006

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
