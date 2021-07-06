FROM ubuntu:bionic
LABEL maintainer filip.zoric@fer.hr

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO melodic
ENV TZ=Europe/Zagreb 
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute 

# Setup timezone 
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone 

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
    
RUN apt-get update && \
    apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/*

# Add sources and keys for Gazebo installation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages
RUN apt-get install -q -y \
    gazebo9-common \
    libgazebo9-dev

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
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
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-dev \ 
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-ros-control \ 
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-mavlink \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-hector-gazebo-plugins

# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python-wstool \ 
    python-catkin-pkg \
    python-catkin-tools \
    python-rosdep \
    protobuf-compiler \
    clang-format-10 \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
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
RUN mkdir -p catkin_ws/src 
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin init" 
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin config --extend /opt/ros/melodic" 
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release" 
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build"
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


RUN sudo apt-get install -y -q \
    ros-${ROS_DISTRO}-rqt-gui-py \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs

# Clone rotors simulator
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/larics/rotors_simulator.git --branch larics_melodic_master --single-branch \
    && cd rotors_simulator \
    && git checkout tags/summer_semester21 \
    && cd ..

# Clone mav_comm
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/larics/mav_comm.git --branch larics_master --single-branch \
    && cd mav_comm \
    && git checkout tags/summer_semester21 \
    && cd ..

WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/catkin/catkin_simple.git

# Clone mmuav gazebo
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/larics/mmuav_gazebo.git \
    && cd mmuav_gazebo \
    && git checkout tags/summer_semester21 \
    && cd .. 

# Demand moveit-ros-planning
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/larics/aerial_manipulators.git --branch devel --single-branch 
RUN git clone https://github.com/suiauthon/impedance_control.git

WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build" 


CMD ["bash"]

