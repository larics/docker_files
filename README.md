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


## How to build Dockerfile? 

Enter corresponding directory and run following command to build docker image: 
```
docker build -t <image_name>:<tag_name> <dockerfile_path> 
```

For concrete example (ros-kinetic with gazebo) run following: 
```
docker build -t ros_gazebo_img:melodic_11
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

## Hardware acceleration 

To be able to properly use available GPU's it's necessary to follow instructions related to setting up docker 
container with proper arguments depending on GPU vendor. There are different commands and prerequisites (drivers) 
for almost every GPU vendor (NVIDIA,ATI,Intel). Rest of this instructions related to graphical rendering are written 
for NVIDIA GPUs, but  [here](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration) you can find everything 
else necessary to achieve same behaviour for different GPUs (Intel/ATI). 

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

### Clion ROS Setup 

Info about CLion IDE setup can be found [here](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html#launch-in-sourced). 

### [How to use hardware acceleration(GPU)  with Docker?](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)

## TODO: 

- [ ] Create example docker files with different hardware accelerations 
- [ ] Create base images for ROS/Gazebo combinations (Kinetic/Gazebo9, Kinetic/Gazebo11, Melodic/Gazebo9, Melodic/Gazebo11) 
- [ ] Refactor images to be multistage
- [x] Add BUILD_ARGS 
- [ ] Add cloning of Github repo with ssh keys 
