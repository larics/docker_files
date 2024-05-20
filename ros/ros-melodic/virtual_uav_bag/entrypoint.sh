#!/bin/bash
bash -c "source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; roslaunch uav_ros_bridge virtual_uav_mqtt.launch" & \
bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; rosbag play -l virtual_uav_minimal.bag" & \
bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; rosbag play -l virtual_uav_video.bag" & \
bash -c "sleep 8; source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw" & \
bash -c "sleep 4; source /opt/ros/melodic/setup.bash; source /root/catkin_ws/devel/setup.bash; rosbag play -l virtual_uav_qr_code.bag"