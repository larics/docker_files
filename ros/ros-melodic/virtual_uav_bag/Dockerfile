FROM ubuntu:bionic

# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=catkin_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=melodic


# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install ROS
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/ros.sh | bash

# Install General ROS things
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/general.sh | bash

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client and git
RUN apt-get install openssh-client git

# download public key for github.com
#RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install workspace
RUN curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/workspace_setup.sh | bash -s $CATKIN_WORKSPACE

# Install requirements
RUN apt install ros-melodic-rosbridge* -y && \
    pip install Inject==3.5.4 paho-mqtt

# Get 
WORKDIR /root
RUN wget --content-disposition https://ferhr-my.sharepoint.com/:u:/g/personal/aivanovic_fer_hr/EWSfkFOi_kRKgIMZGwBWnJ4BN32_ociWPKxNXF1SMtv4Ig?download=1
RUN wget --content-disposition https://ferhr-my.sharepoint.com/:u:/g/personal/aivanovic_fer_hr/ESXBekRTdolNv_OGEhIJ-gIBjpSTBKDnaE366v3JbF_VHw?download=1
RUN wget --content-disposition https://ferhr-my.sharepoint.com/:u:/g/personal/aivanovic_fer_hr/EViCtB1qeDlCqSn0FfCAksoBzZQYPjCAn8UFGZPZez-Gog?download=1
# Install dependencies for zbar. I am lazy so I just install and remove
# the package
RUN apt install ros-melodic-zbar-ros -y && \
    apt remove ros-melodic-zbar-ros -y

# Get web server packages
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/larics/async_web_server_cpp.git --branch noetic-devel && \
    git clone https://github.com/larics/web_video_server.git && \
    git clone https://github.com/larics/zbar_ros.git --branch melodic-devel

# Get mqtt stuff
ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/larics/mqtt_bridge.git && cd mqtt_bridge && git checkout 0.1.7
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/larics/uav_ros_bridge && \
    git clone https://github.com/larics/uav_ros_msgs && \
    catkin build

#COPY entrypoint.sh /root/entrypoint.sh
#RUN ["chmod", "+x", "/root/entrypoint.sh"]
#ENTRYPOINT ["/root/entrypoint.sh"]

# Environment variable for uav namespace
ENV UAV_NAMESPACE red
COPY entrypoint.sh /root/entrypoint.sh
WORKDIR /root
CMD ["/bin/bash","-c","./entrypoint.sh"]