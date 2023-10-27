#!/bin/bash

cd ${ROS2_WS}
source ${ROS2_INSTALL}/setup.bash
source ${ROS2_WS}/install/local_setup.bash
colcon build --packages-skip ros1_bridge
source ${ROS1_INSTALL}/setup.bash
source ${ROS2_INSTALL}/setup.bash
source ${ROS1_WS}/devel/setup.bash
source ${ROS2_WS}/install/local_setup.bash
colcon build --packages-select ros1_bridge --cmake-force-configure

