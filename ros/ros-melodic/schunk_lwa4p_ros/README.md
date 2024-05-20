# Multistage docker image 

Utilizes other Docker image for succesful build. 
Image is located in `/docker_files/ros-melodic/moveit_ros` and built under name 
`moveit_melodic_img:latest`. 

In order to build this Dockerfile you must build `moveit_melodic_img:latest` 
as follows: 
```
cd <path_to_docker_files>/docker_files/ros_melodic/moveit_ros
docker build -t moveit_melodic_img:latest .
```


