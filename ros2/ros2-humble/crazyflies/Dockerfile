# Ubuntu 22.04 
FROM ubuntu:jammy

ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO humble
ENV GZ_RELEASE garden
ARG DEBIAN_FRONTEND=noninteractive
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


#RUN adduser --disabled-password --gecos '' developer 
#RUN adduser developer sudo 
#RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root

RUN --mount=type=ssh id=default \
    mkdir -p -m 0600 ~/.ssh/ && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts

#USER developer 

#install ROS Noetic
RUN mkdir $HOME/.ros_setup
COPY to_copy/ros_comm.patch $HOME/.ros_setup
COPY to_copy/rosconsole.patch $HOME/.ros_setup
COPY to_copy/CMakeLists.txt $HOME/.ros_setup
COPY install_ros.sh $HOME/.ros_setup
RUN cd $HOME/.ros_setup && sudo ./install_ros.sh

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
   ros-dev-tools

RUN apt-get update &&  apt-get upgrade -y && sudo apt install ros-${ROS2_DISTRO}-desktop -y

RUN apt-get install -y python3-colcon-common-extensions

#install Gazebo Garden
RUN /bin/sh -c ' curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'
RUN  apt-get update \
  && apt-get install -y gz-${GZ_RELEASE}

RUN apt-get install python-is-python3

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



#ros1 bridge setup
RUN cd $HOME/CrazySim/ros2_ws/src \
  && sudo git clone https://github.com/ros2/ros1_bridge \
#  && git clone https://github.com/IMRCLab/crazyswarm2 --recursive \
#  && git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git \
   && cd $HOME/CrazySim/ros2_ws  \
   && echo "SOURCING ROS" \
   && bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --merge-install --packages-select crazyflie_interfaces motion_capture_tracking_interfaces" \
  && bash -c "source $HOME/ros_noetic/catkin_ws/devel/setup.bash;source $HOME/CrazySim/ros2_ws/install/setup.bash;source /opt/ros/${ROS2_DISTRO}/setup.bash;colcon build --merge-install --packages-select ros1_bridge --cmake-force-configure"


# Add sourcing for bridge
RUN echo "alias ros2_ws='source $HOME/CrazySim/ros2_ws/install/setup.bash'" >> $HOME/.bashrc

# Add separate aliases for noetic and ROS2_DISTRO
RUN echo "alias source_ros='source $HOME/ros_noetic/catkin_ws/devel/setup.bash'" >> $HOME/.bashrc
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> $HOME/.bashrc
RUN echo "export PATH='$HOME/.local/bin:$PATH'" >> $HOME/.bashrc

#MPC CaseStudy + Acados
RUN apt install libboost-program-options-dev libusb-1.0-0-dev
RUN pip3 install rowan transforms3d
RUN apt install -y ros-${ROS2_DISTRO}-tf-transformations

WORKDIR $HOME


RUN git clone https://github.com/acados/acados.git
#RUN sudo chown -R developer $HOME/acados

RUN cd acados \
    && git submodule update --recursive --init \
    && mkdir -p build \
    && cd build \
    && cmake -DACADOS_WITH_QPOASES=ON .. \
    && make install -j4 \
    && pip install casadi \
    && pip install -e $HOME/acados/interfaces/acados_template

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/acados/lib"" >> $HOME/.bashrc
RUN echo "export ACADOS_SOURCE_DIR="$HOME/acados"" >> $HOME/.bashrc

WORKDIR $HOME/CrazySim/ros2_ws 
RUN bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash;source $HOME/CrazySim/ros2_ws/install/setup.bash;colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-skip ros1_bridge crazyflie_interfaces motion_capture_tracking_interfaces"

WORKDIR $HOME

RUN mkdir $HOME/startup
COPY to_copy/session.yml $HOME/startup
COPY to_copy/start.sh $HOME/startup
COPY to_copy/mpc_case_study_setup.sh $HOME/startup
COPY to_copy/dottmux.conf $HOME
COPY to_copy/install.sh $HOME
RUN ./install.sh



# #crazyflies setup
# RUN pip3 install rowan cflib transforms3d nicegui
# RUN pip3 install --upgrade pip 

# RUN sudo apt-get update && sudo apt install -y \
#   libusb-1.0-0-dev \
#   libxcb-xinerama0 \
#   libxcb-cursor0 \
#   ros-iron-tf-transformations \
#   make gcc-arm-none-eabi \
#   swig3.0 \
#   && sudo ln -s /usr/bin/swig3.0 /usr/bin/swig

# RUN cd $HOME && git clone https://github.com/bitcraze/crazyflie-clients-python \
# && cd crazyflie-clients-python \
# && pip3 install -e .

# RUN cd $HOME/ros2_ws \
#   && bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash;colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON  --packages-skip ros1_bridge crazyflie_interfaces motion_capture_tracking_interfaces"

# RUN cd $HOME && git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git \
#  && cd crazyflie-firmware && git submodule init && git submodule update \
#  && make cf2_defconfig \
#  && make -j 12 \
#  && make bindings_python \
#  && cd build \
#  && python3 setup.py install --user \
#  && export PYTHONPATH=$HOME/crazyflie-firmware/build:$PYTHONPATH

# Set developer as the main used
#ENV USERNAME=developer

# Add user and add it to sudo group
#RUN sudo usermod -a -G plugdev $USERNAME
COPY to_copy/99-bitcraze.rules /etc/udev/rules.d/.

# Commands below run as the developer user
USER root

WORKDIR $HOME
#RUN /bin/bash -c 'source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --merge-install'

# RUN /bin/sh -c 'echo ". /opt/ros/${ROS2_DISTRO}/setup.bash" >> $HOME/.bashrc' \
#     && /bin/sh -c 'echo ". $HOME/ros2_ws/install/setup.sh" >> $HOME/.bashrc'

CMD ["/bin/bash"]
