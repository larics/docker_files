FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 
LABEL maintainer filip.zoric@fer.hr

RUN apt-get update && apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    imagemagick \
    locate \
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \ 
    wget \
    mesa-utils 

# Add sources for Gazebo installation 
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN apt-get update 

# Install gazebo packages 
RUN apt-get install -q -y \
    gazebo9 \ 
    libgazebo9-dev 

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update 

RUN apt-get install -y \ 
    python-catkin-tools 

# Install ROS
RUN apt-get install -y \
    ros-kinetic-ros-core \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-joint-state-publisher \
    ros-kinetic-xacro \
    ros-kinetic-image-transport-plugins \ 
    ros-kinetic-teleop-twist-joy \
    ros-kinetic-map-server \
    ros-kinetic-octomap-ros \
    ros-kinetic-octomap-msgs \
    ros-kinetic-gazebo9-plugins \ 
    ros-kinetic-gazebo9-ros \
    ros-kinetic-gazebo9-ros-pkgs \
    ros-kinetic-gazebo9-ros-control \
    ros-kinetic-gazebo9-dev \
    ros-kinetic-catkin \
    ros-kinetic-moveit \
    ros-kinetic-moveit-msgs \
    ros-kinetic-cob-srvs \ 
    ros-kinetic-control-toolbox \
    ros-kinetic-controller-interface \
    ros-kinetic-costmap-2d \ 
    ros-kinetic-ros-control \
    ros-kinetic-joint-state-controller \
    ros-kinetic-position-controllers \
    ros-kinetic-velocity-controllers \
    ros-kinetic-effort-controllers \
    ros-kinetic-controller-manager \ 
    ros-kinetic-ros-controllers 

# Add user and add it to sudo group 
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers 

USER developer
WORKDIR /home/developer 

RUN sudo rosdep init 
RUN rosdep update 

# Add default path for Gazebo plugins
ENV GAZEBO_PLUGIN_PATH /opt/ros/kinetic/lib

# Install additional GAZEBO_PLUGINS (libhwi_switch_gazebo_ros_control) --> Add libhwi controller for switching between position and velocity control 
RUN git clone --single-branch kinetic_dev https://github.com/ipa320/cob_gazebo_plugins
WORKDIR /home/developer/cob_gazebo_plugins/cob_gazebo_plugins
RUN mkdir build && cd build && cmake ../ && make -j${nproc}
WORKDIR /home/developer/cob_gazebo_plugins/cob_gazebo_ros_control 
RUN mkdir build && cd build && cmake ../ && make -j${nproc}
RUN echo 'export GAZEBO_PLUGIN_PATH=/home/developer/cob_gazebo_plugins/cob_gazebo_ros_control:$GAZEBO_PLUGIN_PATH'  >> ~/.bashrc 

# Init ROS workspace 
RUN mkdir -p ~/ws_moveit/src 
WORKDIR /home/developer/ws_moveit/src
RUN git clone -b kinetic-devel https://github.com/ros-planning/moveit_tutorials.git
RUN git clone -b kinetic-devel https://github.com/ros-planning/panda_moveit_config.git
RUN rosdep install -y --from-paths . --ignore-src --rosdistro kinetic 
WORKDIR /home/developer/ws_moveit 

RUN catkin config --extend /opt/ros/kinetic 
RUN catkin build 
RUN echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc 

WORKDIR /home/developer/ws_moveit/src 
RUN git clone --branch kinetic_dev https://github.com/ipa320/cob_control.git 
RUN git clone https://github.com/h3ct0r/schunk_lwa4p_gazebo_sim.git 

# Add libhwi controller for switching between position and velocity control 

WORKDIR /home/developer/ws_moveit
RUN catkin build 

CMD ["bash"]


