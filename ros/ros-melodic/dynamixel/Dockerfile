FROM ros:melodic-robot-bionic
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

RUN echo "Update"
RUN apt-get update 

# Install ROS
ENV ROS_DISTRO melodic
RUN apt-get update
RUN apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-teleop-twist-joy \ 
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-dynamixel-workbench-controllers \
    ros-${ROS_DISTRO}-dynamixel-workbench-operators \
    ros-${ROS_DISTRO}-dynamixel-workbench-toolbox \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-rqt-service-caller \
    ros-${ROS_DISTRO}-serial 


# Install ROS python packages 
RUN apt-get install -y \
    python-rosdep \
    python-rosinstall \
    python-vcstool \
    python-rosinstall-generator 

# Install catkin tools 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
    python-catkin-tools 

# Add user
RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

# Basic catkin setup
WORKDIR /home/developer
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/melodic/setup.bash; catkin_init_workspace /home/developer/catkin_ws/src" \
    && bash -c ". /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make" \
    && echo '. /home/developer/catkin_ws/devel/setup.bash' >> /home/developer/.bashrc \
    && echo 'export ROS_MASTER_URI=http://$(hostname --ip-address):11311' >> /home/developer/.bashrc \
    && echo 'export ROS_HOSTNAME=$(hostname --ip-address)' >> /home/developer/.bashrc 

# Add repo for handling closing and opening of dynamixels for separator 
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/goranvasilj/separator_end_effector.git
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin_make"


CMD ["tmux"]
