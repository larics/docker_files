FROM ubuntu:jammy AS ros2
LABEL maintainer filip.zoric@fer.hr

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO humble
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
# Not sure this is same in ROS and ROS 2 
# ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

# Mitigate interactive prompt for choosing keyboard type
COPY ./to_copy/keyboard /etc/default/keyboard

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages for ROS and Gazebo
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
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
    libgl1-mesa-dri \
    libeigen-stl-containers-dev \
    dbus-x11 \
    gnome-terminal

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Install ROS2
RUN apt-get update && \
    apt install -y \
    ros-${ROS2_DISTRO}-desktop-full \
    ros-${ROS2_DISTRO}-test-msgs \
    ros-${ROS2_DISTRO}-generate-parameter-library

# Install ROS2 tools
RUN apt install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool 

# Colcon setup according to moveit tutorial 
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Setup PX4-SITL --> Gazebo Harmonic included in installation
WORKDIR /root
RUN mkdir sitl
WORKDIR /root/sitl 
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash -c "bash ./PX4-Autopilot/Tools/setup/ubuntu.sh"
WORKDIR /root/sitl/PX4-Autopilot 
RUN make px4_sitl 

# Install necessary packages
RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools 

# Install ros-gzharmonic bridge for ROS2 Image bridge
RUN apt-get update && \
    apt install -y \
    ros-${ROS2_DISTRO}-ros-gzharmonic

# XRCE-DDS agent
WORKDIR /root/sitl 
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git 
WORKDIR /root/sitl/Micro-XRCE-DDS-Agent
RUN mkdir build
WORKDIR /root/sitl/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make 
RUN make install 
RUN ldconfig /usr/local/lib 

RUN pip install --user -U setuptools==68.1.0

RUN apt-get install tmuxinator

# UAV_WS
RUN apt install ros-dev-tools
WORKDIR /root
RUN mkdir -p uav_ws/src
WORKDIR /root/uav_ws/src
RUN git clone --single-branch --branch main https://github.com/larics/px4_ros_com.git
RUN git clone --single-branch --branch release/1.15 https://github.com/PX4/px4_msgs.git
WORKDIR /root/uav_ws
RUN bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"
RUN echo "source /opt/ros/humble >> ~/.bashrc" 
RUN echo "source /root/uav_ws/install/local_setup.bash" >> ~/.bashrc
RUN echo "ROS_DOMAIN_ID=20" >> ~/.bashrc 

CMD ["bash"]
