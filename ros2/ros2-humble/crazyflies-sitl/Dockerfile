# Ubuntu 22.04
FROM ubuntu:jammy

ENV LANG=C.UTF-8
ENV LC_AL=C.UTF-8
ENV ROS2_DISTRO=humble
ENV GZ_RELEASE=garden
ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ARG INSTALL_BRIDGE=false
ARG HOME=/root


# Tools useful during development
RUN apt-get update \
 && apt install -y \
        build-essential \
        atop \
        clang \
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
        libspdlog-dev \
        libusb-dev \
        libboost-thread-dev \
        libboost-system-dev \
        libboost-filesystem-dev \
        libboost-regex-dev \
        libboost-program-options-dev \
        libconsole-bridge-dev \
        libpoco-dev \
        libtinyxml2-dev \
        liblz4-dev \
        libbz2-dev \
        uuid-dev \
        liblog4cxx-dev \
        libgpgme-dev \
        libgtest-dev \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        python3-nose \
        python3-pycryptodome \
        python3-defusedxml \
        python3-mock \
        python3-netifaces \
        python3-gnupg \
        python3-psutil \
        vim \
        tmux \
        tmuxinator \
        nano \
        net-tools \
        iputils-ping \
        xvfb \
        curl\
        jq\
        wget\
        ranger \
        htop \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        rfkill \
        sudo \
        usbutils \
        software-properties-common \
 && add-apt-repository universe \
 && apt-get clean -qq

RUN apt-get update \
 && apt install -y \
        python3-flake8 \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-docstrings \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest \
        python3-pytest-cov \
        python3-pytest-repeat \
        python3-pytest-rerunfailures


# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client


ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN --mount=type=ssh id=default \
    mkdir -p -m 0600 ~/.ssh/ && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts

COPY to_copy /tmp/to_copy
COPY install_ros.sh /tmp/to_copy

RUN if [ "$INSTALL_BRIDGE" = "true" ]; then \
      echo "Installing ROS Noetic";\
      mkdir $HOME/.ros_setup && \
      cp /tmp/to_copy/ros_comm.patch $HOME/.ros_setup && \
      cp /tmp/to_copy/rosconsole.patch $HOME/.ros_setup && \
      cp /tmp/to_copy/CMakeLists.txt $HOME/.ros_setup && \
      cp /tmp/to_copy/install_ros.sh $HOME/.ros_setup && \
      rm -r /tmp/to_copy/ && \
      cd $HOME/.ros_setup && sudo ./install_ros.sh; \
    else \
      echo "Skipping the installation of ROS Noetic"; \
    fi

# install ROS2 humble
RUN apt update && sudo apt install locales \
  &&  locale-gen en_US en_US.UTF-8 \
  &&  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8

# set up ros2 repo
RUN /bin/sh -c 'sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

RUN apt update && sudo apt install -y \
   python3-rosdep \
   ros-dev-tools \
   libboost-program-options-dev \
   libusb-1.0-0-dev \ 
   python-is-python3

RUN apt-get upgrade -y && sudo apt install ros-${ROS2_DISTRO}-desktop -y

RUN apt-get install -y python3-colcon-common-extensions

#install Gazebo Garden
RUN /bin/sh -c ' curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'
RUN  apt-get update \
  && apt-get install -y gz-${GZ_RELEASE}

RUN pip3 install rowan transforms3d nicegui
RUN apt install -y ros-${ROS2_DISTRO}-tf-transformations \
                    ros-${ROS2_DISTRO}-nav2-map-server \
                    ros-${ROS2_DISTRO}-motion-capture-tracking \
                    ros-${ROS2_DISTRO}-nav2-lifecycle-manager\
                    ros-${ROS2_DISTRO}-rosbridge-suite\
                    ros-${ROS2_DISTRO}-rosbag2-storage-mcap

#Add mcap to bag things in ROS2
RUN VERSION="releases/mcap-cli/v0.0.50" && \
    RELEASE_URL=$(curl -s https://api.github.com/repos/foxglove/mcap/releases | jq -r --arg VERSION "$VERSION" '.[] | select(.tag_name == $VERSION) | .assets[0].browser_download_url') && \
    echo "Downloading release $VERSION from: $RELEASE_URL" && \
    curl -L -o /bin/mcap "$RELEASE_URL" && \
    cd /bin && chmod +x mcap



#installing CrazySim
WORKDIR $HOME
RUN  git clone https://github.com/gtfactslab/CrazySim.git --recursive \
    && cd $HOME/CrazySim/crazyflie-lib-python \
    &&  pip install -e .

RUN pip install Jinja2
RUN cd $HOME/CrazySim/crazyflie-firmware \
    &&  mkdir -p sitl_make/build && cd sitl_make/build \
    &&  cmake .. \
    &&  make all


#install other ROS2 ws packages
RUN mkdir -p $HOME/ros2_ws/src \
  && cd $HOME/ros2_ws/src \
  && git clone https://github.com/IMRCLab/crazyswarm2 --recursive

#Adapting crazyflies.yaml for simulation
RUN rm $HOME/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml
COPY to_copy/crazyflies.yaml $HOME/ros2_ws/src/crazyswarm2/crazyflie/config/

# Add alias for sourcing for ros2 and ros2 workspace
RUN echo "alias ros2_ws='source $HOME/ros2_ws/install/setup.bash'" >> $HOME/.bashrc
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> $HOME/.bashrc

#ros1 bridge setup
RUN if [ "$INSTALL_BRIDGE" = "true" ]; then \
      cd $HOME/ros2_ws/src && \
      sudo git clone https://github.com/ros2/ros1_bridge && \
      cd $HOME/ros2_ws && \
      echo "SOURCING ROS" && \
      bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --merge-install --packages-select crazyflie_interfaces" && \
      bash -c "source $HOME/ros_noetic/catkin_ws/devel/setup.bash;source $HOME/ros2_ws/install/setup.bash;source /opt/ros/${ROS2_DISTRO}/setup.bash;colcon build --merge-install --packages-select ros1_bridge --cmake-force-configure" && \
      echo "alias source_ros='source $HOME/ros_noetic/catkin_ws/devel/setup.bash'" >> $HOME/.bashrc; \
    else \
      echo "Skipping installation of ros1_bridge";\
    fi


RUN echo "export PATH='$HOME/.local/bin:$PATH'" >> $HOME/.bashrc

# setup ros2 environment variables
RUN echo "export ROS_LOCALHOST_ONLY=1" >> $HOME/.bashrc
RUN echo "export ROS_DOMAIN_ID=$(shuf -i 1-101 -n 1)" >> $HOME/.bashrc


WORKDIR $HOME/ros2_ws

# Final build of ROS2 ws
RUN if [ "$INSTALL_BRIDGE" = "true" ]; then \
      bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash;source $HOME/ros2_ws/install/setup.bash;colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-skip ros1_bridge crazyflie_interfaces";\
    else \
      bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash;source $HOME/ros2_ws/install/setup.bash;colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" ;\
      echo "ros2_ws" >> $HOME/.bashrc && \
      echo "source_ros2" >> $HOME/.bashrc;\
    fi

WORKDIR $HOME
RUN mkdir $HOME/startup
COPY to_copy/session.yml $HOME/startup
COPY to_copy/session_bridge.yml $HOME/startup
COPY to_copy/start.sh $HOME/startup
COPY to_copy/_setup.sh $HOME/startup
COPY to_copy/aliases $HOME/.bash_aliases
COPY to_copy/nanorc $HOME/.nanorc
COPY to_copy/tmux $HOME/.tmux.conf
COPY to_copy/ranger $HOME/.config/ranger/rc.conf

USER root
WORKDIR $HOME
CMD ["/bin/bash"]
