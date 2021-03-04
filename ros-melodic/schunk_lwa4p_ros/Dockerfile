FROM moveit_melodic_img:latest 

RUN sudo apt-get update

RUN sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-canopen-chain-node \ 
    ros-${ROS_DISTRO}-canopen-motor-node \
    ros-${ROS_DISTRO}-ros-canopen

WORKDIR /home/developer/moveit_ws/src

RUN git clone https://github.com/fpetric/schunk_robots.git 

WORKDIR /home/developer/moveit_ws/src
RUN bash -c "source /opt/ros/melodic/setup.bash; catkin build" 

CMD ["bash"]
