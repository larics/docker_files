# syntax=docker/dockerfile:1.3
FROM ubuntu:focal

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO noetic
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

# Mitigate interactive prompt for choosing keyboard type
COPY ./to_copy/keyboard /etc/default/keyboard

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
    rfkill

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "test1" 
RUN apt-get update

# Install ROS
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-planners-chomp \
    ros-${ROS_DISTRO}-moveit-planners-ompl \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-ompl \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-image-common \
    ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-canopen-motor-node \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-panda-moveit-config 

# Update due to some weird build error
RUN apt-get update
# Instal RQT debug/devel stuff
RUN apt-get install -y \
    ros-${ROS_DISTRO}-rqt-common-plugins

# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python3-wstool \ 
    python3-catkin-pkg \
    python3-catkin-tools \
    python3-rosdep \
    clang-format-10 \
    libeigen3-dev \
    libtf-conversions-dev \
    python3-pip libglib2.0-dev
    

RUN pip3 install \
    bluepy \
    bleak \
    quaternion \
    ruamel.yaml 
    
# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Install scipy
RUN sudo apt-get install -y python3-scipy

USER developer 
WORKDIR /home/developer

# Set up ROS
RUN sudo rosdep init
RUN rosdep update 

# Init ROS workspace 
RUN mkdir -p catkin_ws/src 
WORKDIR /home/developer/catkin_ws/src
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"; 

# Build initial ROS workspace
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install libfranka
RUN sudo apt install ros-noetic-libfranka ros-noetic-franka-ros -y
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/frankaemika/franka_ros.git
RUN git clone https://github.com/larics/for_franka_ros.git
RUN git clone https://github.com/ros-planning/panda_moveit_config.git

# Setup hostname
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

# Build workspace
WORKDIR /home/developer/catkin_ws/
RUN bash -c "source ~/.bashrc; catkin build"

RUN echo "source /home/developer/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Copy configuration files
COPY --chown=developer:developer ./to_copy/aliases /home/developer/.bash_aliases
COPY --chown=developer:developer ./to_copy/nanorc /home/developer/.nanorc
COPY --chown=developer:developer ./to_copy/tmux /home/developer/.tmux.conf
COPY --chown=developer:developer ./to_copy/ranger /home/developer/.config/ranger/rc.conf

CMD ["bash"]

