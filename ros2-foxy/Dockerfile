# syntax=docker/dockerfile:1.3
FROM ubuntu:focal

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

RUN apt-get update

# Install necessary packages for ROS and Gazebo
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    lsb-release \
    locales \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    tmuxinator \
    wget \
    ranger \
    htop \
    libgl1-mesa-glx \
    libgl1-mesa-dri 


RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=${LC_ALL} LANG=${LANG}

# Verify locale settings 
RUN locale 

RUN apt-get install -y software-properties-common

# Add repository
RUN add-apt-repository universe

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# INSTALL DEPENDENCIES
RUN apt update && apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# install Cyclone DDS dependencies
RUN apt install --no-install-recommends -y \
  libcunit1-dev

# install ros2 in root 
RUN mkdir -p /root/ros2_foxy/src
WORKDIR /root/ros2_foxy
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos ./src

# Update DDS and use rosdep 
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

# Add developer and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

WORKDIR /home/developer
RUN mkdir -p ros2_ws/src
WORKDIR /home/developer/src
RUN sudo apt install python3-colcon-common-extensions

#RUN git clone https://github.com/ros2/examples src/examples -b foxy
#RUN bash -c "source /root/ros2_foxy/install/setup.sh; colcon build --symlink-install"

CMD ["bash"]
