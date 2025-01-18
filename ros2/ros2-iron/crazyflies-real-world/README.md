## Docker
Very briefly, Docker is a tool that provides a simple and efficient way to pack everything needed for a specific application in one container. You can also look at it as a lightweight virtual machine running on your computer.

Basic information about Docker and its main concepts can be found [here](https://github.com/larics/docker_files/wiki). Of course, you can also take a look at the [official website](https://www.docker.com/). Don't follow any instructions from these links just yet. They are provided as a general overview and reference you can use in the future. Detailed step-by-step instructions are given below.

#### Prerequisites
You must have Ubuntu OS installed on your computer. Ideally, this would be Ubuntu 24.04, but another version should work as well. If you have an NVIDIA GPU, please follow [these instructions](https://github.com/larics/docker_files/wiki/2.-Installation#gpu-support) to prepare for Docker installation.

#### Step-by-step instructions
Follow these [instructions](https://docs.docker.com/engine/install/ubuntu/) to install the Docker engine.

Then follow these [optional steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non-root user. If you skip this, every `docker` command will have to be executed with `sudo`. Skip the _"Note: To run Docker without root privileges, see Run the Docker daemon as a non-root user (Rootless mode)."_ part. This is just a note and we do not need it.

Docker containers are intended to run inside your terminal. In other words, you won't see a desktop like in regular virtual machines. However, graphical applications can still run in the container if you give them permission. To do that, execute
```bash
xhost +local:docker
```
To avoid having to do this every time, we can add that command to our `.profile` file which executes on every login.
```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

## Run Crazyflies in the Docker container
### Setting up
Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git

```
Navigate to `ros2-iron` folder:
```
cd docker_files/ros2/ros2-iron/crazyflies

```
Add  to  `~/.bashrc` and source it, or type in the current terminal: 
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
# To install ros1_bridge and ROS Noetic set the argument INSTALL_BRIDGE to true.
# Otherwise set it to false, and it will only install ROS2.
docker build --build-arg INSTALL_BRIDGE=false -t crazyswarm2_img . 

# Run the crazyswarm2_img container for the fist time
./first_run.sh

# This will create docker container crazyswarm2 container and position you into the container
```

For future runs, you can use the following commands:
```
# Start the crazyswarm2_cont:
docker start -i crazyswarm2_cont

# Open the crazyswarm2_cont in another terminal, while it is already started:
docker exec -it crazyswarm2_cont bash

# Stop the conatainer
docker stop crazyswarm2_cont

# Delete the container
docker rm crazyswarm2_cont
```
## Connecting to crazyflies
Before connecting to crazyflies locally on your laptop, copy file: `to_copy/99-bitcraze.rules` and `to_copy/99-lps.rules` to  `/etc/udev/rules.d` directory. Please check these links on USB permissions: [crazyradio](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/) and [lps](https://github.com/bitcraze/lps-tools?tab=readme-ov-file#usb-access-right-on-linux)

To connect to crazyflie you must plug in the Crazyradio [flashed USB dongle](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/) into the laptop. 

The general application [cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/)to connect to crazyflie can be started with (this shouldn't be running while crazyswarm2 packages are launched):

```
cfclient
```
For connecting with ROS2 crazyswarm2 package follow [instructions] (https://imrclab.github.io/crazyswarm2/usage.html)

## ROS1 info
If you have chosen to build the docker including ros1_bridge, please mind the sourcing.

This package is primarily supported in ROS2, and we strongly suggest using ROS2 for development. If you insist on using ROS1, here is an example on how to start rosbridge that will enable communication between ROS1 and ROS2.

Note that ROS2 and ROS1 packages should always run in separate terminals to avoid mixing library paths.

Before starting the bridge, start roscore either locally, or if starting it in container, open new terminal:

```
source_ros
roscore
```

Command to start bridge in the new terminal (please mind the order of using aliases for sourcing):

```
source_ros
ros2_ws
source_ros2
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
Mentioned `source_ros`, `ros2_ws`, `source_ros2` are all aliases defined in the `.bashrc` inside the docker.

## INFO part
If you are working in the group and you are all using the same network, please check [ROS_DOMAIN_ID](https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html#the-ros-domain-id-variable) in .bashrc in container. Random number should be set during the build, however it is not impossible that some of you got the same number. If that is the situatation please change it, so that your simulations do  not crash.

General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

Tutorials on Crazyflies are [here](https://www.bitcraze.io/documentation/start/).

General information about Cfclient can be found [here](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/).

More info about Crazyswarm2 is described in [here](https://imrclab.github.io/crazyswarm2/).

General info on bridge can be found [here](https://github.com/ros2/ros1_bridge/blob/master/README.md) and [here](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html)

## Bonus section
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool that allows you to start a tmux session with a complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal.

You don't need to write new configuration files for your projects, but some examples will use Tmuxinator. You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside the Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with the command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in Ranger.

**Htop** is a better version of `top` - command line interface task manager. Start it with the command `htop` and exit with `q`.

**Mcap** is a format to bag data in ROS2, it is added in this dockerfile and you can use it to store data from runs. Check [this link] (https://mcap.dev/guides/getting-started/ros-2).

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i mrs_project`) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).

### TODO

- [x] Add SSH AUTH to first_run.sh
- [ ] Change https to ssh 
- [ ] Add starting example
- [x] GPU / NO GPU version
