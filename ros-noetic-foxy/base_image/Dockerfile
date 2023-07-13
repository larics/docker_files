FROM ubuntu:focal

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy
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
    libgl1-mesa-dri

# Prepare for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update

# Install ROS
RUN apt-get install -y \
    ros-${ROS1_DISTRO}-desktop

# Instal RQT debug/devel stuff
RUN apt-get install -y \
    ros-${ROS1_DISTRO}-rqt-common-plugins

# Install catkin tools
RUN apt-get install -y --no-install-recommends \
    python3-wstool \ 
    python3-catkin-pkg \
    python3-catkin-tools \
    python3-rosdep \
    clang-format-10 \
    libeigen3-dev \
    libtf-conversions-dev \
    python3-pip libglib2.0-dev

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Install ROS2
RUN apt install -y \
    ros-${ROS2_DISTRO}-desktop 
    
# Install ROS2 tools
RUN apt install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions

# Set up ROS
RUN sudo rosdep init
RUN rosdep update 

# Init ROS workspace 
RUN mkdir -p ros1_ws/src 
WORKDIR /root/ros1_ws/src
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"; 

# Build initial ROS workspace
WORKDIR /root/ros1_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# Init ROS2 workspace
RUN mkdir -p ros2_ws/src
WORKDIR /root/ros2_ws/src

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf
COPY ./to_copy/build_bridge.sh /root/ros2_ws/build_bridge.sh
COPY ./to_copy/run_bridge.sh /root/ros2_ws/run_bridge.sh

ENV ROS1_INSTALL=/opt/ros/$ROS1_DISTRO
ENV ROS2_INSTALL=/opt/ros/$ROS2_DISTRO
ENV ROS1_WS=/root/ros1_ws
ENV ROS2_WS=/root/ros2_ws

# Build ros1_bridge
RUN git clone https://github.com/ros2/ros1_bridge.git --branch $ROS2_DISTRO --single-branch
WORKDIR /root/ros2_ws
RUN bash build_bridge.sh

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "export ROS1_INSTALL=$ROS1_INSTALL" >> ~/.bashrc
RUN echo "export ROS2_INSTALL=$ROS2_INSTALL" >> ~/.bashrc
RUN echo "export ROS1_WS=$ROS1_WS" >> ~/.bashrc
RUN echo "export ROS2_WS=$ROS2_WS" >> ~/.bashrc

RUN echo "" >> ~/.bashrc
RUN echo "alias source_ros1='source $ROS1_INSTALL/setup.bash'" >> ~/.bashrc
RUN echo "alias source_ros2='source $ROS2_INSTALL/setup.bash'" >> ~/.bashrc
RUN echo "alias source_ros1ws='source $ROS1_WS/devel/setup.bash'" >> ~/.bashrc
RUN echo "alias source_ros2ws='source $ROS2_WS/install/setup.bash'" >> ~/.bashrc

CMD ["bash"]
