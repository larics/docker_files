FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04
LABEL maintainer filip.zoric@fer.hr & filip.karaj@gmail.com

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO noetic
ENV TZ=Europe/Zagreb 
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute 

# Setup timezone 
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
    wget \
    libncursesw5-dev \
    swig

# Add sources and keys for Gazebo installation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages
RUN apt-get install -q -y \
    gazebo11-common \
    libgazebo11-dev
    
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
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
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-dev \ 
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-ros-control \ 
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-mavlink \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-rqt-gui \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-hector-gazebo-plugins
    
# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python3-wstool \ 
    python3-catkin-pkg \
    python3-catkin-tools \
    python3-rosdep \
    protobuf-compiler \
    clang-format-10 \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libtf-conversions-dev 
    
# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Install python3 packeges
RUN mkdir tools
WORKDIR /home/developer/tools
RUN apt-get update && apt-get install python3-pip -y
RUN apt-get install libasound-dev -y 
COPY pa_stable_candidate_v190700_rc2_20210331.tgz .
RUN tar zxvf pa_stable_candidate_v190700_rc2_20210331.tgz
WORKDIR /home/developer/tools/portaudio
RUN ./configure 
RUN make 
RUN make install
RUN ldconfig

RUN pip3 install pyaudio \
    && pip3 install pyttsx3 \
    && python3 -m pip install --upgrade pip setuptools wheel \
    && pip3 install --upgrade pocketsphinx \
    && apt-get install gstreamer1.0-pocketsphinx -y

USER developer 
WORKDIR /home/developer

RUN sudo rosdep init
RUN rosdep update

# Init ROS workspace 
RUN mkdir -p catkin_ws/src 
WORKDIR /home/developer/catkin_ws/src
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace" 
#RUN bash -c "source /opt/ros/noetic/setup.bash; catkin config --extend /opt/ros/noetic" 
#RUN bash -c "source /opt/ros/noetic/setup.bash; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release" 
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

WORKDIR /home/developer/catkin_ws/src
RUN echo "REPEAT"

# Clone turtlebot3 & pocketsphinx
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel \
	&& git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel \
	&& git clone https://github.com/mikeferguson/pocketsphinx.git \
	&& cd ..

WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_make" 


CMD ["bash"]

