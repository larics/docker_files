# mmuav_gazebo

This Dockerfile is used to start mmuav_gazebo simulation with corresponding 
UAVs. 

# Repos used:

Main repositories used in this docker are: 
 * [mmuav_gazebo](https://github.com/larics/mmuav_gazebo.git)
 * [rotors_simulator](https://github.com/larics/rotors_simulator.git) 
 * [mav_comm](https://github.com/larics/mav_comm) 

There is `first_run.sh` script which runs (first time) docker container with GUI 
support. 

# Instructions:

Build Docker image with following command when you're located in folder 
containing this Dockerfile:
```
docker build -t mmuav_img:latest .
```
After building image, run first_run.sh script to start container `mmuav_cont`
```
./first_run.sh
```

When you've run (create + start) container, something like this should be 
visible: 
```
Permissions:
-rw-r--r-- 1 your_pc_name your_pc_name  52 mar 26 11:53 /tmp/.docker.xauth

Running docker...
developer@<your_pc_name>:~/catkin_ws$ 

```

After that you may need to source `catkin_ws` as follows: 
```
source /home/developer/catkin_ws/devel/setup.bash
```

When you've sourced workspace, you can run following launches which 
should launch simulation and hoovering example: 

```
roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="none" manipulator_tool:="none" z:=1.0
```

Or if you want manipulator mounted on uav, run following: 
```
roslaunch mmuav_gazebo uav_attitude_position.launch manipulator_type:="wp_manipulator" start_gazebo:=true z:=1.0 manipulator_tool:="rod"
```

## How to generate machine-id? 

Generate machine-id executing following command: 
```
systemd-machine-id-setup
```

# ADD Following stuff: 


What is d-bus? 
How to set up pulseaudio and alsa in docker container? 

# Used links: 

* [run apps using audio in a docker container](https://stackoverflow.com/questions/28985714/run-apps-using-audio-in-a-docker-container)   
* [docker pulseaudio](https://github.com/TheBiggerGuy/docker-pulseaudio-example/blob/master/Dockerfile)  
* [x11-alsa/pulseaudio config](https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio)   
* [simple solution?](https://stackoverflow.com/questions/45700653/can-my-docker-container-app-access-the-hosts-microphone-and-speaker-mac-wind/48795232)   
* [jasper docker](https://github.com/danielchalef/jasper-docker) -- check PCM devices to share 

### [PulseAudio examples](https://wiki.archlinux.org/title/PulseAudio/Examples)
