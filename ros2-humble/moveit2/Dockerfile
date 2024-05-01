FROM ubuntu:jammy as ros2

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO humble
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
# Not sure this is same in ROS and ROS 2
#ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

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
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool 

# Colcon setup according to moveit tutorial 
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN  colcon mixin update default

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Init ROS2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws
RUN colcon build
WORKDIR /root/ros2_ws/src

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

# Install gazebo_harmonic
FROM ros2 as ign_gazebo

# Gazebo installation 
RUN apt-get update
RUN apt-get install ros-${ROS2_DISTRO}-ros-gz -y

# ros2_gz --> go through this!
RUN apt-get install -y \
    ros-${ROS2_DISTRO}-ros-ign-bridge \
    ros-${ROS2_DISTRO}-vision-msgs \
    ros-${ROS2_DISTRO}-joint-state-publisher \
    ros-${ROS2_DISTRO}-ros2-control \
    ros-${ROS2_DISTRO}-ros2-controllers \
    ros-${ROS2_DISTRO}-hardware-interface \
    ros-${ROS2_DISTRO}-controller-interface \ 
    ros-${ROS2_DISTRO}-controller-manager \
    libignition-transport11-dev \
    libignition-gazebo6-dev 

# ROS 2 ws
WORKDIR /root/ros2_ws
RUN rosdep init
RUN rosdep update
RUN rosdep install -r --from-paths src -i -y --rosdistro ${ROS2_DISTRO}

# MoveIt 2 ws
WORKDIR /root/
ENV COLCON_WS /root/ws_moveit2/
RUn mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}/src

# Download moveit2 source code
RUN git clone https://github.com/ros-planning/moveit2.git -b ${ROS2_DISTRO}
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_${ROS2_DISTRO}.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS2_DISTRO} -y
RUN git clone https://github.com/moveit/moveit_task_constructor.git -b ${ROS2_DISTRO}

# Install middleware 
RUN sudo apt install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build MoveIt!
WORKDIR ${COLCON_WS}
RUN sudo apt install ros-${ROS2_DISTRO}-ament-cmake-python
RUN sudo apt-get install ros-${ROS2_DISTRO}-rviz-visual-tools 
RUN bash -c "source /opt/ros/humble/setup.sh; colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"
WORKDIR ${COLCON_WS}/src
RUN git clone https://github.com/moveit/moveit2_tutorials.git -b ${ROS2_DISTRO}
RUN git clone https://github.com/moveit/moveit_visual_tools.git -b ros2
RUN git clone https://github.com/PickNikRobotics/graph_msgs.git -b ros2
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/humble/setup.sh; colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc


CMD ["bash"]
