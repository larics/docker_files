FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04
LABEL maintainer filip.zoric@fer.hr

# TODO: 
# - Add moveit source build

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS_DISTRO melodic
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
    libncursesw5-dev

# Add sources and keys for Gazebo installation
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update

# Install gazebo packages
RUN apt-get install -q -y \
    gazebo9-common \
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
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-dev \ 
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-rqt-launchtree \
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
    python-wstool \ 
    python-catkin-pkg \
    python-catkin-tools \
    python-rosdep \
    protobuf-compiler \
    clang-format-10 \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libtf-conversions-dev 

RUN apt-get update
# Add pulseaudio stuff and corresponding entrypoint
RUN apt-get install -y --no-install-recommends \
    alsa-utils \
    pulseaudio-utils \
    pulseaudio \
    portaudio19-dev \
    libpulse0 \
    libasound-dev \
    swig \
    python-pip \
    gstreamer1.0-pocketsphinx 

#COPY entrypoint.sh /opt/bin/entrypoint.sh
RUN adduser root pulse-access
# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN usermod -a -G audio developer
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer
WORKDIR /home/developer

# Install python3 packeges
RUN mkdir tools
WORKDIR /home/developer/tools
COPY pa_stable_candidate_v190700_rc2_20210331.tgz .
RUN tar zxvf pa_stable_candidate_v190700_rc2_20210331.tgz
WORKDIR /home/developer/tools/portaudio
RUN ./configure 
RUN make 
RUN sudo make install
RUN sudo ldconfig

RUN pip install pyaudio \
    && pip install pyttsx3 swiglpk \
    && python -m pip install --upgrade pip setuptools wheel \
    && pip install --upgrade pocketsphinx 

WORKDIR /home/developer

RUN sudo rosdep init
RUN rosdep update 

# Init ROS workspace 
RUN mkdir -p catkin_ws/src 
WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin init" 
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin config --extend /opt/ros/melodic" 
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release" 
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build"
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /home/developer/catkin_ws/devel/setup.bash" >> ~/.bashrc


RUN sudo apt-get install -y -q \
    ros-${ROS_DISTRO}-rqt-gui-py \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs

WORKDIR /home/developer/catkin_ws/src

# Clone rotors simulator 
RUN git clone https://github.com/larics/rotors_simulator.git --branch larics_melodic_master --single-branch \
    && cd rotors_simulator \
    && git checkout tags/summer_semester21 \
    && cd ..

# Clone mav_comm
RUN git clone https://github.com/larics/mav_comm.git --branch larics_master --single-branch \
    && cd mav_comm \
    && git checkout tags/summer_semester21 \
    && cd ..

RUN git clone https://github.com/catkin/catkin_simple.git

# Clone mmuav_gazebo 
RUN git clone https://github.com/larics/mmuav_gazebo.git \
    && cd mmuav_gazebo \
    && git checkout tags/summer_semester21 \
    && cd .. 

########## Multimaster if needed ###################
# RUN sudo add-apt-repository ppa:roehling/grpc
# RUN sudo apt-get update
# RUN sudo apt install python-grpcio python-grpc-tools
# RUN git clone https://github.com/fkie/multimaster_fkie.git

# Demand moveit-ros-planning
RUN git clone https://github.com/larics/aerial_manipulators.git --branch devel --single-branch 
RUN git clone https://github.com/suiauthon/impedance_control.git

WORKDIR /home/developer/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build" 

# Copy pulse-client (enable server/client arch)
COPY pulse-client.conf /etc/pulse/client.conf

# Setup root user (neccessary for audio setup) 
USER root

# Copy entrypoint script
COPY entrypoint.sh /root/entrypoint.sh 

# Run entrypoint script
ENTRYPOINT /root/entrypoint.sh
