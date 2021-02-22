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


Run following commands for quadcopter control: 

```
roscore
roslaunch ardupilot_gazebo sim_vehicle.launch
roslaunch ardupilot_gazebo mavros.launch
roslaunch gazebo empty_world.launch
roslaunch ardupilot_gazebo spawn_kopterworx.launch fdm_port_in:=9002 fdm_port_out:=9003
```

Activate Carrot PID ROS control: 

```
rostopic pub --once /<uav_namespace>/joy sensor_msgs/Joy "buttons: [0, 0, 0, 0, 1]"; 
```

After activating Carrot PID ROS control it's possible to plan and 
execute trajectories by using `topp_ros` package. 

In order to use it run following (plan trajectory end execute it) : 
```
roslaunch toppra_uav_ros_tracker.launch
```

After that generate trajectory by calling one of the example files such as: 
```
python toppra_trajectory_call_example.py
```

Trajectory example can be found in upper file. 

In order to enable executing trajectories, call service which enables 
execution of current commanded trajectory: 
```
rosservice call /red/topp/enable "{}"
```

It's also possible to send only one pose. In order to do so, run 
following launch file: 
```
roslaunch ardupilot_trajectory_gen.launch
```

And publish wanted pose on following topic: 
```
<uav_namespace>/topp/input/pose
```

Before running anything for trajectory generation, make sure you're 
quadrotor is in mode GUIDED and that motors are armed. 
In order to do so, run following two commands in mavProxy: 
```
mode guided
arm throttle
```
