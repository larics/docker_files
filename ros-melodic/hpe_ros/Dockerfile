# syntax=docker/dockerfile:1.3
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
    locate \
    nano \
    python \
    python-pip \
    python3 \
    python3-pip \
    software-properties-common \ 
    sudo \ 
    wget 

# ================= ros-install =====================

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
    ros-${ROS_DISTRO}-image-common \
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
RUN pip install rospkg 


# ==================== catkin build ==================

RUN mkdir -p catkin_ws/src 
WORKDIR /home/developer/catkin_ws/src
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace" 
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin config -DPYTHON_EXECUTABLE="/opt/conda/bin/python3.8 -DPYTHON_INCLUDE_DIR=/opt/conda/include/python3.8 -DPYTHON_LIBRARY=/opt/conda/lib/libpython3.8.so -DSETUPTOOLS_DEB_LAYOUT=OFF"
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build"

RUN echo "source /home/developer/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# ===================== ssh cloning ===================

ENV DOCKER_BUILDKIT=1
RUN sudo apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root
RUN --mount=type=ssh,id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts 

WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/fzoric8/hpe_ros_package.git
RUN git clone https://github.com/ros-drivers/usb_cam.git

#RUN bash -c "/opt/ros/melodic/setup.bash; catkin build"
# ================= TODO: ============================
#
# set up Torch path in hpe_ros_package automatically
# how to download weights automatically 
#
# =====================================================


CMD ["bash"]


