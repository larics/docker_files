FROM ubuntu:xenial 

LABEL maintainer filip.zoric@fer.hr

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV QT_MAJOR_VERSION 5.12
ENV QT_MINOR_VERSION 5.12.10

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
    ros-${ROS_DISTRO}-teleop-twist-joy

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

RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd

WORKDIR /home/developer
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/kinetic/setup.bash; catkin_init_workspace /home/developer/catkin_ws/src" \
    && bash -c ". /opt/ros/kinetic/setup.bash; cd catkin_ws; catkin_make" \
    && echo '. /home/developer/catkin_ws/devel/setup.bash' >> /home/developer/.bashrc \
    && echo 'export ROS_MASTER_URI=http://$(hostname --ip-address):11311' >> /home/developer/.bashrc \
    && echo 'export ROS_HOSTNAME=$(hostname --ip-address)' >> /home/developer/.bashrc 


# Download QT 
#RUN wget http://download.qt.io/official_releases/qt/${QT_MAJOR_VERSION}/${QT_MINOR_VERSION}/qt-opensource-linux-x64-${QT_MINOR_VERSION}.run 
#RUN pwd
#RUN chmod +x ./qt-opensource-linux-x64-${QT_MINOR_VERSION}.run 
#COPY qt-installer-noninteractive.qs ./
#RUN ls
#RUN ./qt-opensource-linux-x64-${QT_MINOR_VERSION}.run -platform minimal --script qt-installer-noninteractive.qs

# Download QT and install from sources to have good verson 
RUN wget download.qt.io/official_releases/qt/5.15/5.15.2/single/qt-everywhere-src-5.15.2.tar.xz
RUN tar -xvf qt-everywhere-src-5.15.2.tar.xz

COPY configuration.sh /home/developer/qt-everywhere-src-5.15.2 

WORKDIR /home/developer/qt-everywhere-src-5.15.2
RUN ./configuration.sh \
    && make -j${nproc} \
    && make install -j${nproc}

RUN echo 'export PATH=/home/Qt/bin:$PATH' >> /home/developer/.bashc

RUN rm -rf /home/developer/qt-everywhere-src-5.15.2.tar.xz
RUN rm -rf /home/developer/qt-everywhere-src-5.15.2

WORKDIR /home/developer
RUN mkdir code 
WORKDIR /home/developer/code 

ENV BUILD_DIR_SUFFIX buildGCC_x64

RUN git clone https://0a028a24f2f6da7f08027dcc1f49634fe1a4f922:x-outh-basic@github.com/larics/CaptoGloveAPI.git
RUN mkdir CaptoGloveAPI-${BUILD_DIR_SUFFIX}

# Create catkin workspace 


CMD ["bash"]
