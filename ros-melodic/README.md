# Run at first start: 

Command to run this docker image at the first start up is following: 
```
 docker run -it  --network host --privileged --gpus all -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix  -v /dev:/dev --name ros_uav_cont ros_uav_img:latest
```
