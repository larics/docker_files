# Fundamentals of Robotics 2023/2024

This folder contains Dockerfile for the Fundamentals of Robotics 2023/2024. 

You can build docker image from Dockerfile with: 
```
docker build -t for_img . 
```

After building docker image, you can use `first_run.sh` script to create container. 
Bash script `first_run.sh` has all arguments needed to enable starting gazebo, using USB/other 
PC input/outputs as well as access to internet. 

After creating container, you can start it with: 
```
docker start -i for_2324_cont 
```

When you've started container, you can open it in your bash window with: 
```
docker exec -it for_2324_cont bash
```

When you finish working with docker image, you can close it with: 
```
docker stop for_2324_cont 
```

