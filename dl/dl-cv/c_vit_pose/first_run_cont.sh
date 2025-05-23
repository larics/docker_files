#!/usr/bin/bash 

# export DATA_PATH=$PWD in correct folder /home/Work/data
docker run -it --network host --privileged --gpus all \
       -e DISPLAY=$DISPLAY \
       -v /dev:/dev \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       --name vit_pose_cont vit_pose:noetic
