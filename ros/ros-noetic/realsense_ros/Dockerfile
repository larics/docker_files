FROM ubuntu:focal

# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=catkin_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=noetic

# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    build-essential \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    lsb-release \
    tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install General ROS things
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt update && \
    apt-get install -y \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rviz

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
WORKDIR /root
RUN mkdir -p catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git 
WORKDIR /root/catkin_ws/src/realsense-ros/
RUN git checkout ros1-legacy
WORKDIR /root/catkin_ws 
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release"

# Set realsesnse rules
RUN mkdir -p /etc/udev/rules.d
RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules >> /etc/udev/rules.d/99-realsense-libusb.rules

# Source workspaces for the realsense
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc 
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc 

CMD ["bash"] 
