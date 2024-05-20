FROM nvidia/cuda:11.3.0-cudnn8-runtime-ubuntu18.04

LABEL maintainer filip.zoric@fer.hr

ARG USER=developer
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

# Instal Boost & OpenCV
RUN apt-get install -y \
    libboost-all-dev 

# Add developer user 
RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

WORKDIR /home/developer 

# Install OpenCV from sources
RUN mkdir opencv_build 
WORKDIR opencv_build  

#https://answers.opencv.org/question/5090/why-opencv-building-is-so-slow-with-cuda/

RUN git clone -b 3.4 https://github.com/opencv/opencv.git
RUN git clone -b 3.4 https://github.com/opencv/opencv_contrib.git
WORKDIR /home/developer/opencv_build/

# Install qt, prerequisite for correct OpenCV buildy
RUN sudo apt-get install qt5-default -y
WORKDIR /home/developer/opencv_build/build
RUN cmake ../opencv/ -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DOPENCV_GENERATE_PKGCONFIG=ON \
    -DFORCE_VTK=ON \
    -DWITH_TBB=ON \
    -DWITH_V4L=ON \
    -DWITH_QT=ON \ 
    -DWITH_OPENGL=ON \ 
    -DWITH_CUBLAS=ON \
    -DCUDA_NVCC_FLAGS="-D_FORCE_INLINES" \
    -DWITH_GDAL=ON \
    -DWITH_XINE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
    && make -j${nproc-1} 
# Final OpenCV install with 
RUN sudo make install

# Create ROS workspace 
RUN mkdir -p catkin_ws/src
WORKDIR bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace" 

WORKDIR /home/developer/catkin_ws/src
RUN git clone --recursive -b melodic https://github.com/leggedrobotics/darknet_ros.git

RUN sudo apt-get install -y \
    ros-melodic-catkin \
    ros-melodic-nodelet \
    python-catkin-tools \
    libx11-dev

# Call catkin build for ROS workspace build
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build -DCMAKE_BUILD_TYPE=Release"

# Add entrypoint script for sourcing ROS 
CMD ["bash"]
