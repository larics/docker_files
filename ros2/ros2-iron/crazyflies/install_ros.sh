#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: Building ros"

CATKIN_WS=/home/developer/ros_noetic/catkin_ws

[ ! -e $CATKIN_WS/src ] && mkdir -p $CATKIN_WS/src

cd $CATKIN_WS/src

git clone https://github.com/ros/actionlib.git -b 1.14.0
git clone https://github.com/ros/bond_core.git -b 1.8.6
git clone https://github.com/ros/catkin.git -b 0.8.10
git clone https://github.com/ros/class_loader.git -b 0.5.0
git clone https://github.com/ros/cmake_modules.git -b 0.5.0
git clone https://github.com/ros/common_msgs.git -b 1.13.1
git clone https://github.com/ros/dynamic_reconfigure.git -b 1.7.3
git clone https://github.com/ros/gencpp.git -b 0.7.0
git clone https://github.com/jsk-ros-pkg/geneus.git -b 3.0.0
git clone https://github.com/ros/genlisp.git -b 0.4.18
git clone https://github.com/ros/genmsg.git -b 0.6.0
git clone https://github.com/RethinkRobotics-opensource/gennodejs.git -b 2.0.1
git clone https://github.com/ros/genpy.git -b 0.6.16
git clone https://github.com/ros/message_generation.git -b 0.4.1
git clone https://github.com/ros/message_runtime.git -b 0.4.13
git clone https://github.com/ros/nodelet_core.git -b 1.10.2
git clone https://github.com/ros/pluginlib.git -b 1.13.0
git clone https://github.com/ros/ros.git -b 1.15.8
git clone https://github.com/ros/ros_comm.git -b 1.16.0
git clone https://github.com/ros/ros_comm_msgs.git -b 1.11.3
git clone https://github.com/ros/ros_environment.git -b 1.3.2
git clone https://github.com/ros/rosbag_migration_rule.git -b 1.0.1
git clone https://github.com/ros/rosconsole.git -b 1.14.3
git clone https://github.com/ros/rosconsole_bridge.git -b 0.5.4
git clone https://github.com/ros/roscpp_core.git -b 0.7.2
git clone https://github.com/ros/roslisp.git -b 1.9.25
git clone https://github.com/ros/rospack.git -b 2.6.2
git clone https://github.com/ros/std_msgs.git -b 0.5.13

cd /home/developer/ros_noetic

git clone https://github.com/ros-infrastructure/catkin_pkg.git -b 0.5.2
git clone https://github.com/ros-infrastructure/rospkg.git -b 1.5.0

cd catkin_pkg && python3 setup.py install
cd ..
cd rospkg && python3 setup.py install

cd $CATKIN_WS/src/ros_comm
git apply --ignore-whitespace /home/developer/.ros_setup/ros_comm.patch
cd $CATKIN_WS/src/rosconsole
git apply --ignore-whitespace /home/developer/.ros_setup/rosconsole.patch


cd $CATKIN_WS/src/pluginlib/pluginlib
rm CMakeLists.txt
cp /home/developer/.ros_setup/CMakeLists.txt .

cd $CATKIN_WS 
./src/catkin/bin/catkin_make install \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3

echo "$0: Running tests"

cd $CATKIN_WS 
./src/catkin/bin/catkin_make install \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
	  run_tests


