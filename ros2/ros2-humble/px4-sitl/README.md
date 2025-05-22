# PX4-SITL for ROS2

Run UAV in the Gazebo: 

```
gnome-terminal --tab -- sh -c "PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 1; bash"
```

Run MicroXRCEAgent: 

```
MicroXRCEAgent udp4 -p 8888
```

Useful info: 

[PX4 - ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup) 
