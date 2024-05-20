FROM ubuntu:xenial

LABEL maintainer filip.zoric@fer.hr 

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8 

RUN apt-get update && apt-get install -q -y \
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


# Add sources for ROS installation 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS
ENV ROS_DISTRO kinetic
RUN apt-get update
RUN apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-dynamixel-controllers 

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
RUN apt-get install -y --no-install-recommends python-catkin-tools

# Add user 
RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

# Create catkin_ws 
WORKDIR /home/developer
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/kinetic/setup.bash; catkin_init_workspace /home/developer/catkin_ws/src" \
    && bash -c ". /opt/ros/kinetic/setup.bash; cd catkin_ws; catkin_make" \
    && echo '. /home/developer/catkin_ws/devel/setup.bash' >> /home/developer/.bashrc \
    && echo 'export ROS_MASTER_URI=http://$(hostname --ip-address):11311' >> /home/developer/.bashrc \
    && echo 'export ROS_HOSTNAME=$(hostname --ip-address)' >> /home/developer/.bashrc 

# Install PyCharm for development purposes 
#WORKDIR /home/developer 
#ENV version 2020.2.3
#RUN wget -O /home/developer/pycharm.tar.gz "https://download.jetbrains.com/python/pycharm-community-$version.tar.gz"
#RUN tar -xvf /home/developer/pycharm.tar.gz && rm -rf /home/developer/pycharm.tar.gz 
#RUN echo 'alias pycharm=/home/developer/pycharm-community-$version/bin/pycharm.sh' >> /home/developer/.bashrc 

CMD ["tmux"]
