FROM nvidia/opengl:1.0-glvnd-runtime

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
    build-essential \
    bc \
    cmake \
    curl \
    imagemagick \
    libboost-all-dev \
    lsb-release \
    sudo \
    nano \
    net-tools \
    tmux \ 
    wget 

# Add sources and keys for ROS installation 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# Add sources and keys for Gazebo installation 
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages 
RUN apt-get install -q -y \
    gazebo11 \
    libgazebo11-dev 

# Install ROS 
RUN apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-gazebo11 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-pose-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-map-server \
    && rm -rf /var/lib/apt/lists/*

# Install ROS python packages
RUN apt-get install -y \
    python-rosdep  \
    python-rosinstall \
    python-vcstool \
    python-rosinstall-generator 

RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

USER developer
WORKDIR /home/developer

# Init ROS workspace 
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/melodic/setup.bash; catkin_init_workspace ~/catkin_ws/src" \
    && bash -c ". /opt/ros/melodic/setup.bash; cd ~/catkin-ws; catkin_make -DCMAKE_TYPE=Release"

# Setup bashrc environment 
RUN echo '. /usr/share/gazebo/setup.sh' >> /home/developer/.bashrc \
    && echo ". /home/developer/catkin_ws/devel/setup.bash" >> /home/developer/.bashrc \
    && echo "export ROS_MASTER_URI=http://$(hostname --ip-address):11311" >> /home/ddeveloper/.bashrc \ 
    && echo "export ROS_HOSTNAME="$(hostname --ip-address)" >> /home/developer/.bashrc

CMD ["tmux"]
