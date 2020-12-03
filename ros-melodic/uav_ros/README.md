# Run at first start: 

Command to run this docker image at the first start up is following: 
```
 docker run -it  --network host --privileged --gpus all -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix  -v /dev:/dev --name ros_uav_cont ros_uav_img:latest

```


This dockerfile is used to create docker image which has all ros packages and libraries for running kopterworx 
and ArduPilot. In order to start it, open 4-5 terminals and run following commmands in separate terminals:


First run `sim_vehicle.launch` from `ardupilot_gazebo` pkg:
```
roslaunch ardupilot_gazebo sim_vehicle.launch 
```

Run then communitation for software in the loop: 
```
roslaunch ardupilot_gazebo mavros.launch
```

Run kopterworks drone: 
```
roslaunch ardupilot_gazebo kopterworx.launch 
```

Run control pkg which contains implementations for drone control:
```
roslaunch uav_ros_control pid_carrot.launch # Nasa kontrola letjelice
```


