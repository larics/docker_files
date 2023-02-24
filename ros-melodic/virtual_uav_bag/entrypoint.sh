#!/bin/bash
bash -c "source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; roslaunch uav_ros_bridge virtual_uav_mqtt.launch" & \
bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; rosbag play -l virtual_uav_minimal.bag" & \
bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; rosbag play -l virtual_uav_video.bag"