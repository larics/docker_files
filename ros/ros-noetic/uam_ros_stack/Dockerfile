FROM lmark1/uav_ros_simulation:focal-bin-0.0.1 as uav_base

ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=noetic

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install apt packages
RUN apt-get install -y \
    nano 

RUN apt-get update
# Install ROS packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-planners-ompl \
    ros-${ROS_DISTRO}-moveit-planners-chomp \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-moveit-simple-controller \
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-ompl \
    ros-${ROS_DISTRO}-chomp \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-pilz-industrial-motion-planner 

FROM uav_base as uam_base
# Create new workspace for aerial manipulator
RUN mkdir -p $HOME/uam_ws/src
WORKDIR $HOME/uam_ws/src
# ardupilot_models
RUN --mount=type=ssh git clone git@github.com:larics/ardupilot_gazebo -b fzoric/devel 
# uam_ctl
RUN --mount=type=ssh git clone git@github.com:fzoric8/uam_ros_pkg.git
# aerial_manipulators
RUN --mount=type=ssh git clone git@github.com:larics/aerial_manipulators -b fzoric/devel
# mav_comm
RUN --mount=type=ssh git clone git@github.com:larics/mav_comm.git -b larics_master
# rotors simulator
RUN --mount=type=ssh git clone git@github.com:larics/rotors_simulator.git -b larics_noetic_master
WORKDIR $HOME/uam_ws/
RUN bash -c "source /opt/ros/noetic/setup.bash; source ~/.bashrc;  catkin build" 

# Add localhost for it 
RUN echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
RUN echo "source /root/uam_ws/devel/setup.bash" >> ~/.bashrc 

CMD ["bash"]

