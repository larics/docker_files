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
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \ 
    wget 

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
    ros-${ROS_DISTRO}-gazebo-dev 

# Install ROS python packages
RUN apt-get install -y \
    python-rosdep  \
    python-rosinstall \
    python-vcstool \
    python-rosinstall-generator 

# Install catkin tools 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y --no-install-recommends python-catkin-tools

# Install glog 
RUN git clone https://github.com/google/glog.git
RUN cd glog \ 
    && cmake -H. -Bbuild -G "Unix Makefiles" \
    && cmake --build build \
    && cmake --build build --target install 

# Install Eigen3 
RUN apt-get install -y --no-install-recommends \
    libeigen3-dev

RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

USER developer
WORKDIR /home/developer

# Init ROS workspace 
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/melodic/setup.bash; catkin_init_workspace ~/catkin_ws/src" \
    && bash -c ". /opt/ros/melodic/setup.bash; cd catkin_ws; catkin build" 

# Clone ethz-repos for uav simulation
WORKDIR /home/developer/catkin_ws/src 
RUN git clone -b master --single-branch https://github.com/ethz-asl/rotors_simulator.git \
    && cd rotors_simulator \
    && git checkout fe64a7157160781061c11b54678bb5845afaa6d7 

RUN git clone -b master --single-branch https://github.com/ethz-asl/mav_comm.git \
    && cd mav_comm \
    && git checkout 46e4d40a13a37e768e438600ee58c824dffa570d 

RUN git clone https://fzoric:ML2yPHJyMzwb7vfdSRFn@bitbucket.org/fzoric/uav_msgs.git

RUN git clone https://fzoric:ML2yPHJyMzwb7vfdSRFn@bitbucket.org/fzoric/uav_ros_control.git


# Build catkin workspace
WORKDIR /home/developer/catkin_ws
RUN bash -c ". /opt/ros/melodic/setup.bash; cd catkin_ws; catkin build" 

# Setup bashrc environment 
RUN echo '. /usr/share/gazebo/setup.sh' >> /home/developer/.bashrc \
    && echo '. /home/developer/catkin_ws/devel/setup.bash' >> /home/developer/.bashrc \
    && echo 'export ROS_MASTER_URI=http://$(hostname --ip-address):11311' >> /home/developer/.bashrc \ 
    && echo 'export ROS_HOSTNAME=$(hostname --ip-address)' >> /home/developer/.bashrc 

# Pycharm installation 
WORKDIR /home/developer
ENV version 2020.2.3
RUN wget -O /home/developer/pycharm.tar.gz "https://download.jetbrains.com/python/pycharm-community-$version.tar.gz"
RUN mkdir -p /home/developer/pycharm
RUN tar -xvf /home/developer/pycharm.tar.gz && rm -rf /home/developer/pycharm.tar.gz
RUN echo 'alias pycharm=/home/developer/pycharm-community-$version/bin/pycharm.sh' >> /home/developer/.bashrc

CMD ["tmux"]
