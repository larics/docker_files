FROM fzoric8/uav_ros_img:latest

ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=melodic

RUN apt-get install -y \
    ros-${ROS_DISTRO}-mav-msgs \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface 

WORKDIR $CATKIN_WORKSPACE/src
RUN --mount=type=ssh git clone git@github.com:larics/ardupilot_gazebo -b fzoric/devel 
RUN --mount=type=ssh git clone git@github.com:fzoric8/uam_ros_pkg.git
WORKDIR $CATKIN_WORKSPACE
RUN bash -c "source /opt/ros/melodic/setup.bash; source ~/.bashrc;  catkin build" 

CMD ["bash"]

