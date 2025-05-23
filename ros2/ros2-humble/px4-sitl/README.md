# PX4-SITL for ROS2

Run tmuxinator as: 

tmuxinator start px4_ros_start

### Params to disable GCS check for take off

```
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
```

Setting parameters enables arming and takeoff. 

### Useful info: 

[PX4 - ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup) 
