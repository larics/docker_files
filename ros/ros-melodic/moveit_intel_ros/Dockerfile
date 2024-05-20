FROM ubuntu:bionic 
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
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    wget \
    libncursesw5-dev 

# Install mesa libraries in the image 
RUN apt-get update && apt-get install -q -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \ 
    && rm -rf /var/lib/apt/lists/*


# Add sources and keys for Gazebo installation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages
RUN apt-get install -q -y \
    gazebo11 \
    libgazebo11-dev

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
    ros-${ROS_DISTRO}-gazebo11-plugins \
    ros-${ROS_DISTRO}-gazebo11-ros \
    ros-${ROS_DISTRO}-gazebo11-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo11-ros-control \
    ros-${ROS_DISTRO}-gazebo11-dev \ 
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-ros-control \ 
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-collada-urdf \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-tf-conversions 

# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python-wstool \ 
    python-catkin-pkg \
    python-catkin-tools \
    python-rosdep \
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
RUN mkdir -p moveit_ws/src 
WORKDIR /home/developer/moveit_ws/src
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace"; 
WORKDIR /home/developer/moveit_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; wstool init src" \
    && bash -c "wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall" \
    && bash -c "wstool update -t src" \
    && bash -c "rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}" \
    && bash -c "source /opt/ros/melodic/setup.bash; catkin build" 
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


# Add Cartesian planning -> Descartes planning capability
WORKDIR /home/developer/moveit_ws/src
RUN git clone https://github.com/PickNikRobotics/descartes_capability.git
WORKDIR /home/developer/moveit_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; wstool merge -t src src/descartes_capability/descartes_capability.rosinstall" \
    && bash -c "wstool update -t src" \
    && bash -c "rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}" \
    && bash -c "source /opt/ros/melodic/setup.bash; catkin build" 

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/developer/moveit_ws/devel/setup.bash" >> ~/.bashrc 

# Fails with fatal: could not read Username for https://github.com: No such file or directory
#WORKDIR /home/developer/moveit_ws/src
#RUN git clone -b melodic-dev --single-branch https://github.com/larics/schunk_lwa4p_ros.git


# Install OpenRAVE for FastIK plugin 
# Package denpendencies 
# RUN mkdir openrave-repos 
# WORKDIR /home/developer/openrave-repos
# RUN git clone https://github.com/crigroup/openrave-installation
# To install openrave follow this instructions: https://github.com/crigroup/openrave-installation 

# Currently is TracIK used for inverse kinematics 
# WORKDIR /home/developer
# RUN mkdir repos


# CLion installation 
ARG ide
WORKDIR /home/developer
ENV version 2020.3.2
RUN if "$ide"; then wget -O /home/developer/clion.tar.gz "https://download.jetbrains.com/cpp/CLion-2020.3.2.tar.gz"; fi
RUN if "$ide"; then mkdir -p /home/developer/clion; fi
RUN if "$ide"; then tar -xvf /home/developer/clion.tar.gz && rm -rf /home/developer/clion.tar.gz; fi
RUN if "$ide"; then echo 'alias clion=/home/developer/clion-$version/bin/clion.sh' >> /home/developer/.bashrc; fi
RUN if "$ide"; then rm -rf /home/developer/clion.tar.gz; fi

CMD ["bash"]


