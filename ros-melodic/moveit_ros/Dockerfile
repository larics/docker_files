FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04
LABEL maintainer filip.zoric@fer.hr

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO melodic
ENV TZ=Europe/Zagreb

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
    imagemagick \
    locate \
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    wget \ 
    mesa-utils \
    libncursesw5-dev

# Add sources and keys for Gazebo installation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages
RUN apt-get install -q -y \
    gazebo9 \
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
    ros-${ROS_DISTRO}-gazebo-dev \ 
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-msgs \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-controller-interface \ 
    ros-${ROS_DISTRO}-ros-controllers 

# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python-wstool \ 
    python-catkin-pkg \
    python-catkin-tools \
    python-rosdep \
    clang-format-10


# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer 
WORKDIR /home/developer

RUN sudo rosdep init
RUN rosdep update 

RUN echo 'GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib' >> /home/developer/.bashrc
 
# Install additional GAZEBO_PLUGINS (libhwi_switch_gazebo_ros_control) --> Add libhwi controller for switching between position and vel>
RUN git clone --single-branch kinetic_dev https://github.com/ipa320/cob_gazebo_plugins
WORKDIR /home/developer/cob_gazebo_plugins/cob_gazebo_plugins
RUN mkdir build && cd build && cmake ../ && make -j${nproc}
WORKDIR /home/developer/cob_gazebo_plugins/cob_gazebo_ros_control
RUN mkdir build && cd build && cmake ../ && make -j${nproc}
RUN echo 'export GAZEBO_PLUGIN_PATH=/home/developer/cob_gazebo_plugins/cob_gazebo_ros_control:$GAZEBO_PLUGIN_PATH'  >> ~/.bashrc

# Init ROS workspace
RUN mkdir -p ~/ws_moveit/src
WORKDIR /home/developer/ws_moveit/src
RUN git clone -b melodic-devel https://github.com/ros-planning/moveit_tutorials.git 
RUN git clone -b melodic-devel https://github.com/ros-planning/panda_moveit_config.git 

WORKDIR /home/developer/ws_moveit 
RUN catkin config --extend /opt/ros/melodic 
RUN catkin build 
RUN echo 'source /opt/ros/melodic/setup.bash' >> /home/developer/.bashrc

CMD ["bash"]
