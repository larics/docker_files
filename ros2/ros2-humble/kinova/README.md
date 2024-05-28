# Dockerfile for moveit 

Specify target you want to build. 

If you want to build moveit for the Franka robot call: 

```
docker build --target real_robot -t <img_name>:<tag_name> .
```

If you want to build docker that has moveit and gazebo, run: 

```
docker build --target ign_gazebo -t <img_name>:<tag_name>
```


### How to create better Dockerfile for ROS 2

First decouple following workspaces: 

`ros2_ws`: 

Contains `ros2_control`. 

Then: 

`moveit2_ws`: 

Contains `moveit2` built from source. 

And after that: 

`dev_ws`: is the workspace where we develop. 


