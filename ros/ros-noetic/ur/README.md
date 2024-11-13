## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

## Run UR in the Docker container

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git

```
Navigate to `ros-noetic/ur` folder:
```
cd docker_files/ros/ros-noetic/ur

```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
./docker_build.sh

# Run the container
./docker_run.sh

# This will create docker container ur_noetic and position you into the container

# Start the container:
docker start -i ur_noetic

# Execute a command:
docker exec -it ur_noetic bash

# Stop the conatainer
docker stop ur_noetic

# Delete the container
docker rm ur_noetic
```

## Gazebo Simulation

To bring up the simulated robot in Gazebo, run:
```
roslaunch ur_gazebo ur10e.launch
```
For setting up the MoveIt! nodes to allow motion planning run:
```
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch sim:=true
```
For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
```

NOTE:
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:
```
roslaunch ur_gazebo ur10e.launch limited:=true
```
```
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch sim:=true limited:=true
```
```
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
```

## Real Hardware

For preparing the robot and extracting calibration information, see [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).

To actually start the robot driver, run: 
```
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.0.155
```
If you calibrated your robot before, pass that calibration to the launch file:
```
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.0.155 \
kinematics_config:=<path-to-calibration-file>/calibration.yaml
```
For setting up the MoveIt! nodes to allow motion planning run:
```
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch 
```
For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
```

NOTE:
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:
```
roslaunch ur_robot_driver ur10e_bringup.launch limited:=true robot_ip:=192.168.0.155
```
```
roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch limited:=true
```
```
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true
```