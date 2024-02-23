# Ubuntu 22.04 
FROM ubuntu:jammy

ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO iron
ARG DEBIAN_FRONTEND=noninteractive

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
        nano \
        net-tools \
        iputils-ping \
        xvfb \
        curl\
        wget\
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
# install ssh client and git
RUN sudo apt-get install openssh-client git


RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV DOCKER_BUILDKIT=1
RUN sudo apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root

RUN --mount=type=ssh id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts

USER developer 

#install ROS Noetic
RUN mkdir /home/developer/.ros_setup
COPY to_copy/ros_comm.patch /home/developer/.ros_setup
COPY to_copy/rosconsole.patch /home/developer/.ros_setup
COPY to_copy/CMakeLists.txt /home/developer/.ros_setup
COPY install_ros.sh /home/developer/.ros_setup
RUN cd /home/developer/.ros_setup && sudo ./install_ros.sh

# install ROS2 Iron
RUN sudo apt update && sudo apt install locales \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8 

# set up ros2 repo
RUN /bin/sh -c 'sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'


RUN sudo apt update && sudo apt install -y \
   python3-rosdep \
   ros-dev-tools

RUN  sudo apt-get update && sudo apt-get upgrade -y && sudo apt install ros-${ROS2_DISTRO}-desktop -y

RUN sudo apt-get install -y python3-colcon-common-extensions

#ros1 bridge setup
RUN mkdir -p /home/developer/ros2_ws/src \
  && cd /home/developer/ros2_ws/src \
  && git clone https://github.com/ros2/ros1_bridge \
  && git clone https://github.com/IMRCLab/crazyswarm2 --recursive \
  && git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git \
  && cd /home/developer/ros2_ws \
  && echo "SOURCING ROS" \
  && bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --merge-install --packages-select crazyflie_interfaces motion_capture_tracking_interfaces" \
  && bash -c "source /home/developer/ros_noetic/catkin_ws/devel/setup.bash;source /home/developer/ros2_ws/install/setup.bash;source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --merge-install --packages-select ros1_bridge --cmake-force-configure"


# Add sourcing for bridge
RUN echo "alias ros2_ws='source /home/developer/ros2_ws/install/setup.bash'" >> /home/developer/.bashrc

# Add separate aliases for noetic and ROS2_DISTRO
RUN echo "alias source_ros='source /home/developer/ros_noetic/catkin_ws/devel/setup.bash'" >> /home/developer/.bashrc
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> /home/developer/.bashrc
RUN echo "export PATH='/home/developer/.local/bin:$PATH'" >> /home/developer/.bashrc


#crazyflies setup
RUN pip3 install rowan cflib transforms3d nicegui
RUN pip3 install --upgrade pip 

RUN sudo apt-get update && sudo apt install -y \
  libusb-1.0-0-dev \
  libxcb-xinerama0 \
  libxcb-cursor0 \
  ros-iron-tf-transformations \
  make gcc-arm-none-eabi \
  swig3.0 \
  && sudo ln -s /usr/bin/swig3.0 /usr/bin/swig

RUN cd /home/developer && git clone https://github.com/bitcraze/crazyflie-clients-python \
&& cd crazyflie-clients-python \
&& pip3 install -e .

RUN cd /home/developer/ros2_ws \
  && bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash;colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON  --packages-skip ros1_bridge crazyflie_interfaces motion_capture_tracking_interfaces"

RUN cd /home/developer && git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git \
 && cd crazyflie-firmware && git submodule init && git submodule update \
 && make cf2_defconfig \
 && make -j 12 \
 && make bindings_python \
 && cd build \
 && python3 setup.py install --user \
 && export PYTHONPATH=/home/developer/crazyflie-firmware/build:$PYTHONPATH

# Set developer as the main used
ENV USERNAME=developer

# Add user and add it to sudo group
RUN sudo usermod -a -G plugdev $USERNAME
COPY to_copy/99-bitcraze.rules /etc/udev/rules.d/.

# Commands below run as the developer user
USER $USERNAME

WORKDIR /home/$USERNAME
#RUN /bin/bash -c 'source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --merge-install'

# RUN /bin/sh -c 'echo ". /opt/ros/${ROS2_DISTRO}/setup.bash" >> /home/developer/.bashrc' \
#     && /bin/sh -c 'echo ". /home/developer/ros2_ws/install/setup.sh" >> /home/developer/.bashrc'

CMD ["/bin/bash"]
