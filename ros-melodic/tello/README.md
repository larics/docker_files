#### tello-docker

This Dockerfile is used to control the dji-tello UAV.

## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

## Run Tello in the Docker container

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git

```
Navigate to `ros_melodic` folder:
```
cd docker_files/ros-melodic

```
Add  to  `~/.bashrc` and source it, or type in the current terminal: 
```
export DOCKER_BUILDKIT=1
```

Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t tello_img:latest tello

# Run the tello_cont container for the fist time
./tello/first_run.sh

# This will create docker container tello_cont and position you into the container

# Start the tello_cont:
docker start -i tello_cont

# Execute container in terminal while already started in another terminal:
docker exec -it tello_cont bash

# Stop the conatainer
docker stop tello_cont

# Delete the container
docker rm tello_cont

```
Launch tello_driver in the container: 
```
roslaunch tello_driver tello_node.launch
```

Tello driver node is described in [Running the Driver](http://wiki.ros.org/tello_driver).

You can send commands to tello with joystick with running this node: 

```
roslaunch  med_uav_control tello_joy_override.launch
```
