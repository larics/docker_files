# Use ROS 2 Humble base image
FROM ros:humble

#ARG BASE_IMAGE_NAME=lmark1/uav_ros_stack_source
#ARG BASE_IMAGE_TAG=focal

#FROM $BASE_IMAGE_NAME:$BASE_IMAGE_TAG

ARG ROS2_DISTRO=humble

# Install necessary tools and dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    git \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-teleop-twist-keyboard \
    libasio-dev \
    libtinyxml2-dev \
    nano \
    vim \
    can-utils \
    iproute2 \
    net-tools \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone
#RUN ip link set can0 up type can bitrate 500000

RUN apt-get update

# --------------- SSH ---------------
# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client and git
RUN apt-get install openssh-client git
# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts


# Set up a workspace
WORKDIR /root/scout_ws/src

# Clone the scout ROS 2 repository
RUN git clone --single-branch --branch humble https://github.com/westonrobot/scout_ros2.git
RUN git clone https://github.com/westonrobot/ugv_sdk.git
RUN apt-get install -y ros-humble-tf2-geometry-msgs
# Sensor fusion
RUN --mount=type=ssh git clone --branch master git@github.com:larics/sensor_fusion.git
# Go to workspace and build it
WORKDIR /root/scout_ws/

# Install dependencies with rosdep
RUN apt-get update 
RUN rosdep update 
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh
#RUN colcon build

# Source the workspace on container startup
RUN echo "source /opt/ros/humble/setup.sh" >> /root/.bashrc
RUN echo "source /root/scout_ws/install/setup.bash" >> /root/.bashrc

# Build the workspace
#RUN /bin/bash -c '/opt/ros/humble/setup.sh && colcon build'

# Set environment variables for ROS 2
ENV ROS_DOMAIN_ID=0

# ===================== ssh cloning ===================
ENV DOCKER_BUILDKIT=1
RUN sudo apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root
RUN --mount=type=ssh id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts 

RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
RUN echo "source /root/scout_ws/install/setup.bash" >> ~/.bashrc
WORKDIR /root/scout_ws/
# Run the container as an interactive shell
CMD ["bash"]
