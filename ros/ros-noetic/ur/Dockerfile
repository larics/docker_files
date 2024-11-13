# syntax=docker/dockerfile:1.3
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

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

# Add sources and keys for Gazebo installation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages
RUN apt-get install -q -y \
    gazebo11 \
    libgazebo11-dev

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
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-dev \ 
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-ros-control \ 
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-collada-urdf \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-rosserial \
    ros-${ROS_DISTRO}-serial \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-dynamixel-workbench-controllers \
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ur-robot-driver


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
RUN mkdir -p ur_ws/src 
WORKDIR /home/developer/ur_ws/src
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"; 

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/developer/ur_ws/devel/setup.bash" >> ~/.bashrc 


# Install ROS packages from src
WORKDIR /home/developer/ur_ws/src
RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot 
WORKDIR /home/developer/ur_ws
RUN bash -c "rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}" \
    && bash -c "source /opt/ros/noetic/setup.bash; catkin build"


# ===================== ssh cloning ===================
ENV DOCKER_BUILDKIT=1
RUN sudo apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root
RUN --mount=type=ssh,id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts 


USER developer
CMD ["bash"]