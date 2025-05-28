FROM ubuntu:jammy AS ros2

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
    can-utils \
    cmake \
    curl \
    ethtool \    
    git \
    gnupg \
    iproute2 \
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
    libeigen-stl-containers-dev 

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Install ROS2
RUN apt-get update && \
    apt install -y \
    ros-${ROS2_DISTRO}-desktop-full \
    ros-${ROS2_DISTRO}-test-msgs \
    ros-${ROS2_DISTRO}-generate-parameter-library \
    ros-${ROS2_DISTRO}-joint-state-publisher-gui 

# Install ROS2 tools
RUN apt install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool 

# MoveIt 2 ws
WORKDIR /root
ENV COLCON_WS /root/ws_moveit2
RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}/src

# Download moveit2 source code
RUN git clone https://github.com/ros-planning/moveit2.git -b ${ROS2_DISTRO}
RUN git clone https://github.com/moveit/py_binding_tools.git -b ros2
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_${ROS2_DISTRO}.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN sudo rosdep init 
RUN rosdep update
RUN rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS2_DISTRO} -y
#RUN git clone https://github.com/moveit/moveit_task_constructor.git -b ${ROS2_DISTRO}

# Install middleware 
RUN sudo apt-get update
RUN sudo apt install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc


# Build MoveIt!
WORKDIR ${COLCON_WS}
RUN sudo apt install -y \
    ros-${ROS2_DISTRO}-ament-cmake-python \
    ros-${ROS2_DISTRO}-rviz-visual-tools \
    ros-${ROS2_DISTRO}-vision-msgs 
RUN bash -c "source /opt/ros/humble/setup.sh; source ~/.bashrc; MAKEFLAGS="-j${nproc-1}" colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"
WORKDIR ${COLCON_WS}/src
RUN git clone https://github.com/moveit/moveit2_tutorials.git -b ${ROS2_DISTRO}
RUN git clone https://github.com/moveit/moveit_visual_tools.git -b ros2
RUN git clone https://github.com/PickNikRobotics/graph_msgs.git -b ros2
RUN git clone https://github.com/moveit/moveit_task_constructor.git -b ${ROS2_DISTRO}
#RUN git clone https://github.com/moveit/moveit_resources.git -b ros2
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/humble/setup.sh; source ~/.bashrc; MAKEFLAGS="-j${nproc-1}" colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

# Install ROS2 tools
RUN apt install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-pip 

# Colcon setup according to moveit tutorial 
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Moveit2! binary installation 
RUN apt-get install -y \
#    ros-${ROS2_DISTRO}-moveit \
    ros-${ROS2_DISTRO}-controller-manager \
    ros-${ROS2_DISTRO}-joint-trajectory-controller \
    ros-${ROS2_DISTRO}-joint-state-broadcaster

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Piper installation 
WORKDIR /root
RUN mkdir -p piper_ws/src
WORKDIR /root/piper_ws/src
RUN git clone --single-branch --branch humble https://github.com/larics/piper_ros.git
RUN pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools==68.1.0 numpy==1.26.4
RUN pip3 install piper_sdk 
WORKDIR /root/piper_ws
RUN bash -c "source /opt/ros/humble/setup.bash; colcon build" 
RUN echo "source /piper_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"] 
