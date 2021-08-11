FROM pytorch/pytorch:1.7.0-cuda11.0-cudnn8-devel

LABEL maintainer filip.zoric@fer.hr

# Set arguments 
ARG USER=developer 
ARG UID=1000
ARG GID=1000 
ARG PW=developer
ARG TZ=Europe/Zagreb
ARG ROS_DISTRO=melodic

# Setup timezone (fix interactive package installation) 
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone


# Install necessary software packages 
RUN apt-get update && \
    apt-get install -y --no-install-recommends \ 
    build-essential \
    feh \ 
    git \
    nano \
    python \
    python-pip \
    python3 \
    python3-pip \
    software-properties-common \ 
    sudo \ 
    wget 


# Add ROS keys
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
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-ros-control \ 
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rqt-gui 



# Add developer user / remove neccessity to type password for sudo command
RUN adduser --disabled-password \
    --gecos '' ${USER}
RUN adduser ${USER} sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers 

# Install packages neccessary for training and testing neural networks
RUN conda install -c anaconda \
                     scipy \
                     configargparse \
                     cython

RUN conda install -c conda-forge \
                     progress \
                     json_tricks \
                     tensorboardx \
                     easydict \
                     opencv \
                     pycocotools

USER developer 
WORKDIR /home/developer

RUN sudo apt-get install -y python-catkin-tools

# Init ROS workspace 
RUN mkdir -p catkin_ws/src 
WORKDIR /home/developer/catkin_ws/src
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace" 
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build"

CMD ["bash"]

