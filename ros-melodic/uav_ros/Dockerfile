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
    gitk \
    geographiclib-doc \
    geographiclib-tools \
    imagemagick \
    lsb-release \
    libboost-dev \
    libgeographic-dev \
    libgeographic17 \
    node-geographiclib \
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
    ros-${ROS_DISTRO}-gazebo-dev \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-mavlink \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-dynamic-edt-3d \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-hector-gazebo-plugins 

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
RUN apt-get install -y --no-install-recommends \
    python-catkin-tools \
    protobuf-compiler \
    libgoogle-glog-dev \
    libopencv-dev \
    python-pip

# Install neccessary pip packages
RUN pip install \
    pymavlink \
    mavproxy

WORKDIR /opt/ros/${ROS_DISTRO}/lib/mavros/
RUN bash -c "./install_geographiclib_datasets.sh"     
 
RUN apt-get install  -y ros-${ROS_DISTRO}-rqt-gui-py
    
# Install Eigen3 
RUN apt-get install -y --no-install-recommends \
    libeigen3-dev

RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

# Setup sudo config for prereqs ardupilot script
ADD /sudoers.txt /etc/sudoers
RUN chmod 400 /etc/sudoers 

USER developer
WORKDIR /home/developer

# Setup Ardupilot 
RUN git clone https://github.com/ArduPilot/ardupilot.git 
WORKDIR /home/developer/ardupilot 
RUN git submodule update --init --recursive 

ARG DEBIAN_FRONTEND=noninteractive 
RUN sudo usermod -G users developer 
ENV SKIP_AT_EXT_ENV=1 SKIP_API_GRAPHIC_ENV=1 SKP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1

WORKDIR /home/developer/ardupilot/Tools/environment_install 
RUN bash -c "./install-prereqs-ubuntu.sh -y"; exit 0
RUN echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc 
RUN echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc 

# Init ROS workspace
WORKDIR /home/developer  
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/melodic/setup.bash; catkin_init_workspace ~/catkin_ws/src" \
    && bash -c ". /opt/ros/melodic/setup.bash; cd catkin_ws; catkin build" 

# Clone ethz-repos for uav simulation - rotors simulator
WORKDIR /home/developer/catkin_ws/src 
RUN git clone https://github.com/larics/rotors_simulator.git \ 
    && cd rotors_simulator \
    && git checkout larics_melodic_master 

# Mav comm
WORKDIR /home/developer/catkin_ws/src 
RUN git clone https://github.com/larics/mav_comm.git \
    && cd mav_comm \
    && git checkout larics_master 

# Ardupilot gazebo
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/larics/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && git checkout larics-master 

# ROS control 
WORKDIR /home/developer/catkin_ws/src 
RUN git clone https://github.com/larics/uav_ros_control.git \
    && cd uav_ros_control \
    && git checkout master-v3 

# Build catkin workspace
WORKDIR /home/developer/catkin_ws
RUN bash -c ". /opt/ros/melodic/setup.bash; cd catkin_ws; catkin build"  

# Setup bashrc environment 
RUN echo '. /usr/share/gazebo/setup.sh' >> /home/developer/.bashrc \
    && echo '. /home/developer/catkin_ws/devel/setup.bash' >> /home/developer/.bashrc \
    && echo 'export ROS_MASTER_URI=http://$(hostname --ip-address):11311' >> /home/developer/.bashrc \ 
    && echo 'export ROS_HOSTNAME=$(hostname --ip-address)' >> /home/developer/.bashrc \ 
    && echo 'source ~/catkin_ws/src/ardupilot_gazebo/scripts/shell_scripts.sh' >> ~/.bashrc \ 
    && echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc \
    && echo 'export GAZEBO_MODEL_PATH=/home/developer/catkin_ws/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc \
    && echo 'export GAZEBO_PLUGIN_PATH=/home/developer/catkin_ws/build/ardupilot_gazebo:${GAZEBO_PLUGIN_PATH}' >> ~/.bashrc \
    && echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc

# Pycharm installation 
#WORKDIR /home/developer
#ENV version 2020.2.3
#RUN wget -O /home/developer/pycharm.tar.gz "https://download.jetbrains.com/python/pycharm-community-$version.tar.gz"
#RUN mkdir -p /home/developer/pycharm
#RUN tar -xvf /home/developer/pycharm.tar.gz && rm -rf /home/developer/pycharm.tar.gz
#RUN echo 'alias pycharm=/home/developer/pycharm-community-$version/bin/pycharm.sh' >> /home/developer/.bashrc

CMD ["tmux"]
