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
    imagemagick \
    lsb-release \
    sudo \
    nano \
    net-tools \
    tmux \ 
    wget 

#RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd 

# Setup sudo config for prereqs ardupilot script
#ADD /sudoers.txt /etc/sudoers
#RUN chmod 400 /etc/sudoers 

WORKDIR /home/developer
#RUN git clone https://github.com/fzoric8/uav_ros_simulation.git 
COPY /uav_ros_simulation  /home/developer/uav_ros_simulation
WORKDIR /home/developer/uav_ros_simulation/installation
RUN bash -c "./install_ros_and_gazebo.sh" 
RUN bash -c "./install_ardupilot_and_source.sh"
RUN bash -c "./setup_workspace.sh" 
WORKDIR /root/uav_ws
RUN bash -c "catkin build" 

CMD ["bash"]

