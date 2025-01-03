## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

## Run Crazyflies in the Docker container

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git

```
Navigate to `ros2-iron` folder:
```
cd docker_files/ros2-iron/crazyflies

```
Add  to  `~/.bashrc` and source it, or type in the current terminal: 
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t crazyswarm2_img . 

# Run the crazyswarm2_img container for the fist time
./first_run.sh

# This will create docker container crazyswarm_container and position you into the container

# Start the bebop_cont:
docker start -i crazyswarm2_cont

# Stop the conatainer
docker stop crazyswarm2_cont

# Delete the container
docker rm crazyswarm2_cont
```
Before connecting to crazyflies locally on your laptop, copy file: `to_copy/99-bitcraze.rules` to  `/etc/udev/rules.d` directory.

General application can be started with (this shouldn't be running while crazyswarm2 packages are launched):

```
cfclient
```

For connecting with ROS2 crazyswarm2 package follow [instructions] (https://imrclab.github.io/crazyswarm2/usage.html)

Note that before running any ros2 package you need to source ros2  with:

```
ros2_ws
source_ros2
```
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

Tutorials on Crazyflies are [here](https://www.bitcraze.io/documentation/start/).

General information about Cfclient can be found [here](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/).

More info about Crazyswarm2 is described in [here](https://imrclab.github.io/crazyswarm2/).

General info on bridge can be found [here](https://github.com/ros2/ros1_bridge/blob/master/README.md) and [here](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)

### TODO

- [ ] Add SSH AUTH to first_run.sh
- [ ] Change https to ssh 
- [ ] Add ros2 to ros1 interfaces and ./configure
- [ ] Add general example
- [ ] Add startup scripts
- [ ] One alias for sourcing
- [ ] GPU / NO GPU version