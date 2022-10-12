# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

USER root

ARG DEBIAN_FRONTEND=noninteractive

# Tools useful during development
RUN apt-get update -qq \
 && apt-get install --no-install-recommends -y -qq \
        build-essential \
        atop \
        cmake \
        cppcheck \
        expect \
        gdb \
        git \
        gnutls-bin \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
        libfcl-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        software-properties-common \
        vim \
        net-tools \
        iputils-ping \
        xvfb \
        curl \
 && apt-get clean -qq

# set up ros2 repo
RUN /bin/sh -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# install ignition fortress
RUN apt-get update \
  && apt-get install -y ignition-fortress \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

# install rosdep
RUN apt-get update \
  && apt install -y python3-rosdep \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq \
  && rosdep init \
  && rosdep update

# install ROS2
RUN apt-get update \
  && apt-get install -y ros-galactic-ros-base \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

# install colcon
RUN apt-get -qq update && apt-get -q -y install \
  python3-vcstool \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Set developer as the main used
ENV USERNAME=developer

# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Prepare the colcon workspace
RUN mkdir -p catkin_ws/src

# clone ros_ign bridge
RUN cd /home/$USERNAME/catkin_ws/src \
    && git clone https://github.com/osrf/ros_ign.git -b galactic

WORKDIR /home/$USERNAME/catkin_ws/src
RUN git clone -b galactic-devel --single-branch https://github.com/ros/ros_tutorials.git

ENV IGNITION_VERSION fortress

WORKDIR /home/$USERNAME/catkin_ws
RUN sudo apt-get update \
    && rosdep update \
    && rosdep install -r --from-paths src -i -y --rosdistro galactic \
    && sudo rm -rf /var/lib/apt/lists/* \
    && sudo apt-get clean -qq

WORKDIR /home/$USERNAME/catkin_ws
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash; colcon build --merge-install'

RUN /bin/sh -c 'echo ". /opt/ros/galactic/setup.bash" >> ~/.bashrc' \
    && /bin/sh -c 'echo ". ~/catkin_ws/install/setup.sh" >> ~/.bashrc'

CMD ["/bin/bash"]
