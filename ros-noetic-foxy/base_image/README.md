# ROS Noetic + ROS2 Foxy on Ubuntu 20.04
This Docker file does the following:
- Installs ROS and ROS2 on Ubuntu 20.04. from the official repositories (apt install)
- Creates empty workspaces for ROS and ROS 2.
- Installs ros1_bridge from source.
- Allows running ROS and ROS2 nodes in the same container, or in separate containers.
- Shows how to forward an ssh agent into the container, so that you can use ssh keys from the host (i.e., you can use your ssh key to clone private repositories from inside the container).
- Uses cool tools such as tmux, tmuxinator, and ranger with custom bindings (see [to_copy](to_copy)).
- Uses cool aliases for ROS and ROS2 commands such as `killp`, `waitForRos`, and better `colcon` that works from anywhere in the workspace (see [aliases](to_copy/aliases)).

## Usage
### Build
```bash
docker build -t ros_noetic_foxy:latest .
```

### Run
```bash
bash run_docker.sh <container_name> ros_noetic_foxy:latest
```

### Start
```bash
bash start_docker.sh <container_name>
```

### Running ros1_bridge
Instructions to come. In the meantime, see [official tutorial](https://github.com/ros2/ros1_bridge/blob/master/README.md).

## Important note
Due to the very specific requirements of ros1_bridge, ROS and ROS 2 are not sourced by default using the `.bashrc`. Remember to source them manually in each terminal you open. For convenience, you can use the `source_ros1`, `source_ros1ws`, `source_ros2`, and `source_ros2ws` aliases.


## TODO:
- [ ] Make ROS distros configurable. Currently they are hardcoded to noetic and foxy.