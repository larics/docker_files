# syntax=docker/dockerfile:1.3
FROM ubuntu:focal
LABEL maintainer filip.zoric@fer.hr

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'
#ROSCONSOLE_FORMAT=[${severity}] [${time}] [${node}] [${file}] [${thread}] [${function}]: ${message}


# Mitigate interactive prompt for choosing keyboard type
COPY ./keyboard /etc/default/keyboard

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
    imagemagick \
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    wget \
    libncursesw5-dev


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update

# Install ROS
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-ros-control \ 
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-rosserial 

# Instal RQT debug/devel stuff
RUN apt-get install -y \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller 


# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python3-wstool \ 
    python3-catkin-pkg \
    python3-catkin-tools \
    python3-rosdep \
    clang-format-10 \
    libeigen3-dev \
    libtf-conversions-dev

# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer 
WORKDIR /home/developer

RUN sudo rosdep init
RUN rosdep update 
 
# Init ROS workspace 
RUN mkdir -p bebop_ws/src 
WORKDIR /home/developer/bebop_ws/src
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"; 
WORKDIR /home/developer/bebop_ws


# ================== plotJuggler =====================
ARG plotjuggler
WORKDIR /home/developer/bebop_ws/src
RUN if "$plotjuggler"; then git clone https://github.com/PlotJuggler/plotjuggler_msgs.git; fi
RUN if "$plotjuggler"; then git clone https://github.com/facontidavide/PlotJuggler.git; fi
RUN if "$plotjuggler"; then git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git; fi
WORKDIR /home/developer/bebop_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# ===================== IDE =========================
# CLion installation
ARG ide
WORKDIR /home/developer
ENV version 2020.3.2
RUN if "$ide"; then wget -O /home/developer/clion.tar.gz "https://download.jetbrains.com/cpp/CLion-2020.3.2.tar.gz"; fi
RUN if "$ide"; then mkdir -p /home/developer/clion; fi
RUN if "$ide"; then tar -xvf /home/developer/clion.tar.gz && rm -rf /home/developer/clion.tar.gz; fi
RUN if "$ide"; then echo 'alias clion=/home/developer/clion-$version/bin/clion.sh' >> /home/developer/.bashrc; fi
RUN if "$ide"; then rm -rf /home/developer/clion.tar.gz; fi

# =====================================================

# Build initial ROS workspace
WORKDIR /home/developer/bebop_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin build"  

# ===================== ssh cloning ===================
ENV DOCKER_BUILDKIT=1
RUN sudo apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root
RUN --mount=type=ssh id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts 

WORKDIR /home/developer/bebop_ws/src
RUN --mount=type=ssh,id=default git clone git@github.com:larics/parrot_arsdk.git
RUN --mount=type=ssh,id=default git clone git@github.com:antonellabarisic/parrot_arsdk.git
USER developer
RUN bash -c "/opt/ros/noetic/setup.bash; catkin build"


CMD ["bash"]

