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

WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/larics/mqtt_bridge.git && cd mqtt_bridge && git checkout 0.1.7
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/larics/uav_ros_bridge && catkin build

WORKDIR /root
RUN wget --content-disposition https://ferhr-my.sharepoint.com/:u:/g/personal/aivanovic_fer_hr/EWSfkFOi_kRKgIMZGwBWnJ4BN32_ociWPKxNXF1SMtv4Ig?download=1

#COPY entrypoint.sh /root/entrypoint.sh
#RUN ["chmod", "+x", "/root/entrypoint.sh"]
#ENTRYPOINT ["/root/entrypoint.sh"]

COPY entrypoint.sh .
CMD ["/bin/bash","-c","./entrypoint.sh"]