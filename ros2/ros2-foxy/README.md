# ROS 2 foxy

Docker installation of the ROS 2 foxy. 

## Instructions used 

[ROS 2 installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) 

[Colcon build instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) 

## Multi-stage build 

If you want to build gazebo with ROS 2, run following command:
```
docker build --target ign_gazebo -t ros2_img:foxy .
```


# TODO: 
- [x] Build init Dockerfile 
- [x] Add multistage build for the ignition gazebo 
- [x] Test gazebo 
- [x] Add init workspace build 
- [ ] Add gazebo_ros_pkgs
- [ ] Add different gazebos 
