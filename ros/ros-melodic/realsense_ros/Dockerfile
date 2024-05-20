FROM ubuntu:bionic

# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=catkin_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=melodic


# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install ROS
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/ros.sh | bash

# Install General ROS things
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/general.sh | bash


# Install Realsense specific dependencies
RUN apt-get update && \
    apt-get install -y \
        openssh-server \
        libmysqlclient-dev \
        ros-$ROS_DISTRO-realsense2-camera \
        ros-$ROS_DISTRO-rgbd-launch \
        ros-$ROS_DISTRO-realsense2-description \
        libglew-dev \
        libglfw3 \
        libglfw3-dev \
        nano


# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client and git
RUN apt-get install openssh-client git

# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install workspace
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/workspace_setup.sh | bash -s $CATKIN_WORKSPACE

# Set realsesnse rules
RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules >> /etc/udev/rules.d/99-realsense-libusb.rules
