#!/usr/bin/bash

docker run -it --network host --privileged --gpus all \
       --device=/dev/ttyUSB0 \
       -e DISPLAY=$DISPLAY \
       -v /dev:/dev \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       --name dynamixel_cont dynamixel:kinetic
