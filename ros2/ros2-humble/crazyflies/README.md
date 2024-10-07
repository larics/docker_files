## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

## Run Crazyflies in the Docker container

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git

```
Navigate to `ros2-humble` folder:
```
cd docker_files/ros2-humble/crazyflies

```
Add  to  `~/.bashrc` and source it, or type in the current terminal:
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t crazysim_img2 .

# Run the crazysim_img2 container for the fist time
./first_run.sh

# This will create docker container crazyswarm_container and position you into the container

# Start the crazysim_cont:
docker start -i crazysim_cont2

# Start the crazysim_cont in another terminal, while it is already started:
docker exec -it crazysim_cont2 bash

# Stop the conatainer
docker stop crazysim_cont2

# Delete the container
docker rm crazysim_cont2

```
The docker contains packages for crazyflies simulator [CrazySim](https://github.com/gtfactslab/CrazySim).

The ros2 workspace is located in /root/CrazySim/ros2_ws

Note that before running any ros2 package you need to source ros2  using alias:

```
ros2_ws
source_ros2
```

In the folder /root/startup is start.sh script, that starts the session with the examples from [CraySim page](https://github.com/gtfactslab/CrazySim).

Since this is primarly supported in ROS2, there is an example how to start rosbridge, if code is written using ROS1.

Also note that ros2 and ros1 packages should be always run in separate terminals to avoid mixing paths to libraries.

Before starting the bridge start roscore either locally or if starting it in container, open new terminal:

```
source_ros
roscore
```

Command to start bridge in the new terminal:

```
source_ros
ros2_ws
source_ros2
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

General info on bridge can be found [here](https://github.com/ros2/ros1_bridge/blob/master/README.md) and [here](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)

### TODO

- [x] Add SSH AUTH to first_run.sh
- [ ] Change https to ssh
- [ ] Add ros2 to ros1 interfaces and ./configure
- [x] Add general example
- [x] Add startup scripts
- [ ] One alias for sourcing
- [ ] GPU / NO GPU version
