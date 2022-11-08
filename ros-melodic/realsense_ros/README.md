## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t ros_realsense_img realsense_ros 

# Run the ros_realsense_cont container for the fist time
export DOCKER_BUILDKIT=1
./realsense_ros/first_run.sh

#  Run the container 
docker start -i ros_realsense_cont

# Stop the conatainer
docker stop ros_realsense_cont

# Delete the container
docker rm ros_realsense_cont
```

## RealSense camera - usage Instructions for ROS

### Start the camera node
To start the camera node in ROS:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This will stream all camera sensors and publish on the appropriate ROS topics.

Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

### Published Topics
The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `rostopic list`):
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/color/metadata
- /camera/depth/camera_info
- /camera/depth/image_rect_raw
- /camera/depth/metadata
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/gyro/imu_info
- /camera/gyro/metadata
- /camera/gyro/sample
- /camera/accel/imu_info
- /camera/accel/metadata
- /camera/accel/sample
- /diagnostics

If you want to use/check camera depth run:

```bash
roslaunch realsense2_camera rs_camera.launch
```
and the topic `/camera/depth_registered/points` will be available.

For more information, please refer to [realsense-ros](https://github.com/IntelRealSense/realsense-ros).