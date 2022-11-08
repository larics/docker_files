# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install --no-install-recommends -y \
    tzdata \
    sudo \
    wget \
    gnupg2 \
    lsb-release \
 && rm -rf /var/lib/apt/lists/* \
 && apt-get -qq clean

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get -qq update && apt-get -q -y install tzdata \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

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
        openssh-server \
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

# Dependencies for glvnd and X11.
RUN apt install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
-rw------- 1 developer developer  104 Jun 17 08:13 .Xauthority
  && rm -rf /var/lib/apt/lists/*

# Open Ports
# ssh
EXPOSE 1234
# jupyter notebook
EXPOSE 8080

# Set up SSH forwarding.
RUN echo "X11UseLocalhost no" >> /etc/ssh/sshd_config
ADD xorg.conf /usr/share/X11/xorg.conf.d/xorg.conf
RUN service ssh start
CMD ["/usr/sbin/sshd","-D"]

# set up ros2 repo
RUN /bin/sh -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# Set up repo to install Ignition
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'

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

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME


# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Prepare the colcon workspace
RUN mkdir -p mbzirc_ws/src

# clone ros_ign bridge
RUN cd /home/$USERNAME/mbzirc_ws/src \
 && git clone https://github.com/osrf/ros_ign.git -b galactic

WORKDIR /home/$USERNAME/mbzirc_ws

COPY . src/mbzirc

ENV IGNITION_VERSION fortress

RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -r --from-paths src -i -y --rosdistro galactic \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt-get clean -qq

# Be sure that mbzirc_ros is present in the compilation by calling info
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash \
#  && colcon info mbzirc_ros \
  && colcon build --merge-install'
# TODO(jrivero): implement build arg to clean build/ log/ while releasing
# && rm -fr build/ log/

# Install neccessary ROS packages
RUN sudo apt-get update 
RUN sudo apt-get install -y \
    ros-galactic-mavros-msgs \ 
    ros-galactic-joy* \
    ros-galactic-ament-cmake-pycodestyle
   

#Add rviz
# WORKDIR /home/developer
# RUN mkdir -p rviz_ws/src
# WORKDIR /home/developer/rviz_ws/src
# RUN git clone -b galactic --single-branch https://github.com/ros2/rviz.git
# RUN git clone https://github.com/ros2/geometry2.git
# Fix for issue of overloading cancel
# WORKDIR /home/developer/rviz_ws/src/geometry2
# RUN git checkout 0.22.0 
# WORKDIR /home/developer/rviz_ws 
# RUN bash -c "source /opt/ros/galactic/setup.bash; colcon build --merge-install" # tf2_py fails during build

RUN /bin/sh -c 'echo ". /opt/ros/galactic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/rviz_ws/install/setup.sh" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/mbzirc_ws/install/setup.sh" >> ~/.bashrc'

# Copy entry point script, and set the entrypoint
# COPY mbzirc_sim/run_sim.bash ./
# ENTRYPOINT ["./run_sim.bash"]

CMD ["/bin/bash"]
