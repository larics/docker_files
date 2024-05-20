# ROS 2 Humble on Ubuntu 22.04
This Docker file does the following:
- Installs ROS2 on Ubuntu 22.04. from the official repositories (apt install)
- Creates empty workspace for ROS 2.
- Shows how to forward an ssh agent into the container, so that you can use ssh keys from the host (i.e., you can use your ssh key to clone private repositories from inside the container).
- Uses cool tools such as tmux, tmuxinator, and ranger with custom bindings (see [to_copy](to_copy)).
- Uses cool aliases for ROS and ROS2 commands such as `killp`, `waitForRos`, and better `colcon` that works from anywhere in the workspace (see [aliases](to_copy/aliases)).

## Usage
### Build
```bash
docker build -t ros_humble:latest .
```

### Run
```bash
bash run_docker.sh <container_name> ros_humble:latest
```

### Start
```bash
bash start_docker.sh <container_name>
```