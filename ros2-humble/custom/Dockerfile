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

# Fix for apt-get update? 

# Mitigate interactive prompt for choosing keyboard type
COPY ./to_copy/keyboard /etc/default/keyboard

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

#RUN apt remove libappstream4
#RUN apt-get update

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
    python3-colcon-common-extensions

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Add developer and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Init ROS2 workspace
RUN mkdir -p /home/developer/ros2_ws/src
WORKDIR /home/developer/ros2_ws
RUN colcon build
WORKDIR /home/developer/ros2_ws/src

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/developer/ros2_ws/install/local_setup.bash" >> ~/.bashrc

# Install gazebo_harmonic
FROM ros2 as ign_gazebo

# Gazebo installation 
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y gz-harmonic

# ros2_gz
RUN sudo apt-get install -y \
    ros-${ROS2_DISTRO}-ros-ign-bridge \
    ros-${ROS2_DISTRO}-vision-msgs \
    libignition-transport11-dev \
    libignition-gazebo6-dev 

WORKDIR /home/developer/ros2_ws/src
RUN git clone https://github.com/gazebosim/ros_gz.git -b ${ROS2_DISTRO}
WORKDIR /home/developer/ros2_ws
RUN sudo rosdep init
RUN rosdep update
RUN rosdep install -r --from-paths src -i -y --rosdistro ${ROS2_DISTRO}

CMD ["bash"]
