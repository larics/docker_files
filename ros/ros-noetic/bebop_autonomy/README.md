## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

## Run Parrot Bebop in the Docker container

Clone the [repository](https://github.com/larics/docker_files):
```
git clone https://github.com/larics/docker_files.git

```
Navigate to `ros_noetic` folder:
```
cd docker_files/ros-noetic

```
Add  to  `~/.bashrc` and source it, or type in the current terminal: 
```
export DOCKER_BUILDKIT=1
```

Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t bebop_img bebop_autonomy 

# Run the bebop_cont container for the fist time
./bebop_autonomy/first_run.sh

# This will create docker container bebop_cont and position you into the container

# Start the bebop_cont:
docker start -i bebop_cont

# Stop the conatainer
docker stop bebop_cont

# Delete the container
docker rm bebop_cont

```
Launch bebop_driver in the container: 
```
roslaunch bebop_driver bebop_node.launch
```
General information about Parrot Bebop can be found [here](https://bebop-autonomy.readthedocs.io/en/latest/).

Bebop driver node is described in [Running the Driver](https://bebop-autonomy.readthedocs.io/en/latest/running.html).

You can send commands to Bebop as shown in [Sending Commands to Bebop](https://bebop-autonomy.readthedocs.io/en/latest/piloting.html).