# Dockerfile for moveit 

Specify target you want to build. 

If you want to build moveit for the Franka robot call: 

```
docker build --target real_robot -t <img_name>:<tag_name> .
```

If you want to build docker that has moveit and gazebo, run: 

```
docker build --target ign_gazebo -t <img_name>:<tag_name>
```


