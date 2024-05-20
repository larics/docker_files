#!/bin/bash

# Source ROS 1 first:
source ${ROS1_INSTALL}/setup.bash
# Source ROS 2 next:
source ${ROS2_INSTALL}/setup.bash
source ${ROS2_WS}/install/local_setup.bash

export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics

