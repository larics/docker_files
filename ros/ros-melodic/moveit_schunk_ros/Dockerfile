FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04
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
    imagemagick \
    lsb-release \
    libboost-dev \
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
    ros-${ROS_DISTRO}-gazebo-dev


# Install catkin tools
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get install -y --no-install-recommends \
    python-wstool \ 
    python-catkin-pkg \
    python-catkin-tools \
    python-rosdep \
    clang-format-10

# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer 
WORKDIR /home/developer

RUN sudo rosdep init
RUN rosdep update 
 
# Init ROS workspace 
RUN mkdir -p moveit_ws/src 
WORKDIR /home/developer/moveit_ws
RUN bash -c "source /opt/ros/melodic/setup.bash; wstool init src" \
    && bash -c "wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall" \
    && bash -c "wstool update -t src" \
    && bash -c "rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}"

# Build moveit from source
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release 
RUN catkin build 

WORKDIR /home/developer

# Install neccessary ROS package for Schunk simulation 
RUN apt-get install -y no-install-recommends \
    ros-melodic-cob-srvs \
    ros-melodic-cob-control \
    ros-melodic-costmap-2d \
    ros-melodic-moveit-msgs \ 
    ros-melodic-tf-conversions
        

RUN mkdir -p catkin_ws/src
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/h3ct0r/schunk_lwa4p_gazebo_sim.git
RUN git clone -b melodic_dev https://github.com/ipa320/cob_control.git 
WORKDIR /home/developer/catkin_ws
RUN catkin_make

# Source all ROS packages
RUN echo 'source /opt/ros/melodic/setup.bash' >> /home/developer/.bashrc
RUN echo 'source /home/developer/moveit_ws/devel/setup.bash' >> /home/developer/.bashrc
RUN echo 'source /home/developer/catkin_ws/devel/setup.bash' >> /home/developer/.bashrc

WOKRDIR /home/developer
CMD ["bash"]


