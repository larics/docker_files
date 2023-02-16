## Docker
Install Docker using installation instruction found [here](https://docs.docker.com/engine/install/ubuntu/).

Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t virtual_uav_bag . 

# Run the virtual_uav_bag_cont container for the fist time
export DOCKER_BUILDKIT=1
./first_run.sh

#  Run the container 
docker start -i virtual_uav_bag_cont

# Stop the conatainer
docker stop virtual_uav_bag_cont

# Delete the container
docker rm virtual_uav_bag_cont
```