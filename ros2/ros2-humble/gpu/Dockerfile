FROM nvidia/cuda:12.4.0-runtime-ubuntu22.04

# Setup environment
ENV LANG=C.UTF-8
ENV LC_AL=C.UTF-8
ENV ROS2_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
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
    cuda-toolkit \
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
    libeigen-stl-containers-dev 

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Fix sources bug
RUN rm /etc/apt/sources.list.d/cuda-ubuntu2204-x86_64.list

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
    python3-vcstool \
    python3-pip

# Colcon setup according to moveit tutorial 
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Update repos
RUN apt-get update -y

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc

# Install middleware 
RUN sudo apt install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

WORKDIR /root/
RUN mkdir -p /root/ros2_ws/src

# Build ros2_ws
WORKDIR /root/ros2_ws/
RUN bash -c "source /opt/ros/humble/setup.bash; sudo rosdep init; rosdep update && rosdep install --ignore-src --from-paths src -y"
RUN bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"
COPY ./to_copy/ros2_aliases /tmp/
RUN cat /tmp/ros2_aliases >> ~/.bashrc 
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Add example for mediapipe
# RUN pip3 install mediapipe
# RUN python3 -m pip install 'tensorflow[and-cuda]'
# RUN pip3 install tensorflow_hub

# Optionally - add mp_ros_wrapper and hpe_ros_msgs

CMD ["bash"]

