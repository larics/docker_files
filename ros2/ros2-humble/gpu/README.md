# Dockerfile for moveit 

Specify target you want to build. 

If you want to build moveit for the Franka robot call: 

```
docker build -t <img_name>:<ros2_distro> .
```
Recommended command is: 
```
docker build -t ros2_gpu_img:humble 
```

You can run the new docker image with: 
```
./run_docker.sh
```

After that, you can start docker with: 
```
docker start -i ros2_gpu_cont 
```

After starting container you can execute it with: 
```
docker exec -it ros2_gpu_cont bash
```


