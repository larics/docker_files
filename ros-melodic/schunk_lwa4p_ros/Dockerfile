# Inheriting ROS melodic image with ROS/Gazebo, can be found in moveit_ros folder
FROM moveit_melodic_img:latest 

RUN sudo apt-get update

RUN sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-canopen-chain-node \ 
    ros-${ROS_DISTRO}-canopen-motor-node \
    ros-${ROS_DISTRO}-ros-canopen \ 
    ros-${ROS_DISTRO}-openzen-sensor 

WORKDIR /home/developer/moveit_ws/src

# Download driver for schunk lwa4p
RUN git clone -b melodic-devel --single-branch https://github.com/fzoric8/schunk_robots.git 

# Download driver for IMUs
RUN git clone --recurse-submodules https://bitbucket.org/lpresearch/openzenros.git

# Donwload custom ROS package for IMUs
RUN git clone https://github.com/fzoric8/openzen_acore_imu.git

# Add privileges to serial ports that are used for IMUs --> Needs to be executed everytime when plugged/unpluged
RUN sudo chmod 777 /dev/ttyUSB0 
RUN sudo chmod 777 /dev/ttyUSB1
RUN sudo chmod 777 /dev/ttyUSB2

# Download driver for tool
WORKDIR /home/developer/moveit_ws/src
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build" 


CMD ["bash"]
