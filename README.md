# Dockerfile repo

Brief theoretic introduction can be found in Instructions.md

This repository contains all Dockerfiles which are used for software containerization that implies easier 
integration and development. 

## How to install Docker engine?

In order to install docker engine follow this [instructions](https://docs.docker.com/engine/install/ubuntu/) for installation on ubuntu. 

You will also need nvidia-docker which can be installed by executing following commands: 
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```
```
sudo apt-get update
```
```
sudo apt-get install -y nvidia-docker2
```

**Add permissions for X server to enable GUI applications:**
```
echo `xhost local:root` >> ~/.bashrc  
```

## Conceptual understanding

In next figure it is shown  how to create docker container from Dockerfile 

![Figure 1.](./utils/assets/figure1.png) 

In following figure it's possible to see how different commands for docker container are used: 

![Figure 2.](./utils/assets/figure2.png)



## How to build Dockerfile? 

Enter corresponding directory and run following command to build docker image: 
```
docker build -t <image_name>:<tag_name> <dockerfile_path> 
```

For concrete example (ros-kinetic with gazebo) run following: 
```
docker build -t ros_gazebo_img:melodic_11 .
```

## How to create container out of the Dockerfile? 

In order to create container from Dockerfile run following commmand:
```
docker run -it --network host --privileged -v /dev:/dev --name <container_name> <img_name>:<tag_name> 
```

Run docker container with GUI support 
```
docker run -it --network host --gpus all --privileged -e DISPLAY=$DISPLAY -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix \
       --name <container_name> <img_name>:<tag_name> 
```
Also, before using param `--gpus` make sure you've installed `nvidia-container-toolkit` as follows: 

```
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```
Make sure you have latest drivers for your GPU. 

## How to start existing docker container after stopping it? 
```
docker start -i <container_name> 
```

## How to open running container in new bash? 
```
docker exec -it <container_name> bash 
```

## How to kill, stop, remove container? 

Kill container: 
```
docker kill  <container_name> 
```

Stop container: 
```
docker  stop <container_name> 
```

Remove container: 
```
docker rm <container_name> 
```

Attach existing detached cotainer: 
```
docker attach <container_name> 
```

Clean dangling images and stopped containers (bear in mind that this deletes 
every container that's not currently running, not recommended): 
```
docker system prune 
```

### [Docker-CLI reference](https://docs.docker.com/engine/reference/commandline/build/)

### How to create configuration file that runs multiple docker files at once? 

In order to run multiple docker containers which are build from docker images in certain configuration 
use [docker-compose](https://docs.docker.com/compose/) 

## [Hardware acceleration](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)

To be able to properly use available GPU's it's necessary to follow instructions related to setting up docker 
container with proper arguments depending on GPU vendor. There are different commands and prerequisites (drivers) 
for almost every GPU vendor (NVIDIA,ATI,Intel). Rest of this instructions related to graphical rendering are written 
for NVIDIA GPUs, but  [here](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration) you can find everything 
else necessary to achieve same behaviour for different GPUs (Intel/ATI). 

#### Some examples can be found in ros-melodic folder, such as moveit_intel_ros and moveit_ros (NVIDIA) 

### How to correctly use OpenGL for better rendering capabilities in container? 

Explanation for using ROS + Docker for GUI applications is available [here](http://wiki.ros.org/docker/Tutorials/GUI)

### Explanation of OpenGL + NVIDIA in Docker

Explanation for creating OpenGL image for CUDA applications can be found [here](https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1). 

Available prebuilt nvidia/openGL dockers can be found [here](https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1) 

### Example of build args 

There's example of using build-args for installing IDE in container in following [Dockerfile](https://github.com/larics/docker_files/blob/master/ros-melodic/moveit_ros/Dockerfile) 

```
docker build --build-arg <arg_name>=<arg_value> -t <image_name>:<image_tag> .
```
### Commit docker container to docker image

You can commit current state of docker image as follows: 
```
docker commit --author filip.zoric@fer.hr --pause --message "[fer-auth] integration" mmuav_audio_cont auth_fer_uav_img:latest
```

Such image you can then compress to `tar.gz` or push to docker registry. 

### Save docker image

You can save docker image with docker save command and compress it to gzip as follows: 
```
docker save <img_name>:<tag> | gzip > <archive_name>.tar.gz

```


### Clion ROS Setup 

Info about CLion IDE setup can be found [here](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html#launch-in-sourced)

### Most common problems 

One of the main problems that occurs when using Docker files for robotic applications is expiration of GPG keys for ROS. 
More about this incident can be found [here](https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669/)

In order to fix that, in case that some `sudo apt-get update` or `sudo apt-get install` fails, please replace GPG 
keys to newer ones. Error for expiration of GPG keys looks like following: 
```
An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu bionic InRelease: 
The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
```

You can add keys by following 1.3 section in Installation instructions for corresponding ROS distribution. 

If you already have docker image built, and you get following error message when trying to install new ROS package: 
```

E: Failed to fetch http://packages.ros.org/ros/ubuntu/pool/main/r/ros-melodic-sound-play/ros-melodic-sound-play_0.3.11-1bionic.20210414.224641_amd64.deb  404  Not Found [IP: 140.211.166.134 80]
E: Unable to fetch some archives, maybe run apt-get update or try with --fix-missing?

```

You can fix it by typing following command: 
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

```

## Git submodules

You can use and initialize git submodules with following command: 
```
git submodule update --init <submodule_name>
```

In this repo, currently is only `mbzirc` submodule. 

## TODO: 

- [x] Create example docker files with different hardware accelerations 
- [x] Create base images for ROS/Gazebo combinations (Kinetic/Gazebo9, Kinetic/Gazebo11, Melodic/Gazebo9, Melodic/Gazebo11) 
- [ ] Refactor images to be multistage
- [x] Add BUILD_ARGS -> ide for now
- [x] Add image for decoupled NVIDIA + pytorch support for OpenPose
- [ ] Add entrypoint script for every Docker containing ROS (can be used from darknet ros) 
- [ ] Build image with [GAZEBO gym](https://github.com/erlerobot/gym-gazebo)  
- [ ] Build image with [AirSim](https://microsoft.github.io/AirSim/docker_ubuntu/) 
- [ ] Build base blender image 
- [ ] Add cloning of Github repo with ssh keys 
- [x] Create intial compose 
- [ ] Check integration of Github actions for CI/CD
