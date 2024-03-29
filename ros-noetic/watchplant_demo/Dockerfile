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
    iproute2 \
    iputils-ping \
    sshpass

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update

# Install ROS
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-roslint \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-plotjuggler-ros

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
    python3-pip \
    clang-format-10 \
    libeigen3-dev \
    libtf-conversions-dev

# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN adduser developer dialout

USER developer 
WORKDIR /home/developer

# Instal Python packages
RUN python3 -m pip install pyserial

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
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source ~/.ros_setup" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "echo Hello! If this is your first time using this container, please run: [bash utils/raspi_docker_setup.sh]" >> ~/.bashrc
RUN echo "echo Otherwise, please run [tmuxinator start -p launch/connect_remote.yml] to start WatchPlant demo." >> ~/.bashrc

# Clone packages
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/mkrizmancic/watchplant_demo.git
RUN bash -c "source ~/.bashrc; catkin build"

# Copy configuration files
COPY --chown=developer:developer ./to_copy/aliases /home/developer/.bash_aliases
COPY --chown=developer:developer ./to_copy/nanorc /home/developer/.nanorc
COPY --chown=developer:developer ./to_copy/tmux /home/developer/.tmux.conf
COPY --chown=developer:developer ./to_copy/ranger /home/developer/.config/ranger/rc.conf
COPY --chown=developer:developer ./to_copy/ros_setup /home/developer/.ros_setup

WORKDIR /home/developer/catkin_ws/src/watchplant_demo

CMD ["bash"]

