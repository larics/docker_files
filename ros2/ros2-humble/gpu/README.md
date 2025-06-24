# Dockerfile for moveit 

Specify target you want to build. 

If you want to build moveit for the Franka robot call: 

```
docker build -t <img_name>:<ros2_distro> .
```
Recommended command is: 
```
docker build -t arm_api2_img:humble 
```

Or you can pull it straight from the dockerhub as: 
```
docker pull crobotic_solutions/arm_api2:humble 
```

You can run the new docker image with: 
```
./run_docker.sh
```

After that, you can start docker with: 
```
docker start -i arm_api2_cont 
```

After starting container you can execute it with: 
```
docker exec -it arm_api2_cont bash
```

#### Possible issues: 

- [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/21)

## TODO: 

- [x] ROS 2 + humble working again
- [ ] One click run 
- [ ] SSH keys 
- [ ] Dev setup [autocomplete + standard] 
