FROM lmark1/uav_ros_simulation:focal-bin-0.0.1 

ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root
ARG ROS_DISTRO=noetic

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Install apt packages
RUN apt-get install -y \
    nano 

# Add ROS to the list of sources
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update packages 
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
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-ompl \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-pilz-industrial-motion-planner 

# YAML-CPP 
WORKDIR /root
RUN git clone https://github.com/jbeder/yaml-cpp.git
WORKDIR /root/yaml-cpp
RUN git checkout tags/yaml-cpp-0.6.3 
RUN mkdir build 
WORKDIR /root/yaml-cpp/build 
RUN bash -c "cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON .."
RUN make -j${nproc}
RUN make install 

# Update Build workspace with correct repositories 
WORKDIR /root/sim_ws/src/uav_ros_simulation 
RUN --mount=type=ssh git pull
RUN git checkout fzoric/uam_moveit 
WORKDIR /root/sim_ws/src/uav_ros_simulation/ros_packages/ardupilot_gazebo
RUN --mount=type=ssh git pull origin fzoric/devel
RUN git checkout fzoric/devel
WORKDIR /root/sim_ws/src
RUN --mount=type=ssh git clone git@github.com:fzoric8/uam_ros_pkg.git
WORKDIR /root/sim_ws/src/uam_ros_pkg
RUN --mount=type=ssh git pull origin new_order 
RUN git checkout new_order
WORKDIR /root/sim_ws/src
RUN --mount=type=ssh git clone https://github.com/ros-visualization/rviz.git
WORKDIR /root/sim_ws/src/rviz 
RUN git checkout noetic-devel 
WORKDIR /root/sim_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# Add localhost for it 
RUN echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
# RUN echo "source /root/uam_ws/devel/setup.bash" >> ~/.bashrc 

CMD ["bash"]

