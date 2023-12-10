FROM fzoric8/uav_ros_img:latest

ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=melodic

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install apt packages
RUN apt-get install -y \
    nano 

# Install ROS packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-mav-msgs \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs

# Create new workspace for aerial manipulator
RUN mkdir -p $HOME/uam_ws/src
WORKDIR $HOME/uam_ws/src
# ardupilot_models
RUN --mount=type=ssh git clone git@github.com:larics/ardupilot_gazebo -b fzoric/devel 
# uam_ctl
RUN echo "TEST" 
RUN --mount=type=ssh git clone git@github.com:fzoric8/uam_ros_pkg.git
# aerial_manipulators
RUN --mount=type=ssh git clone git@github.com:larics/aerial_manipulators -b fzoric/devel
WORKDIR $HOME/uam_ws/
RUN bash -c "source /opt/ros/melodic/setup.bash; source ~/.bashrc;  catkin build" 

# Add localhost for it 
RUN echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
RUN echo "source /root/uam_ws/devel/setup.bash" >> ~/.bashrc 

CMD ["bash"]

