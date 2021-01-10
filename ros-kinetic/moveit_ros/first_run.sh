#!/bin/bash

docker run -it --network host --gpus all --privileged -e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--name moveit_kinetic_cont moveit_ros_img:kinetic

