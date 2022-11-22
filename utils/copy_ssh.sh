#!/bin/bash

echo "What's the name of the container where you want to copy ssh keys?"

read cont_name

echo "Container name where ssh_config is copied is: $cont_name"

docker cp /home/$USER/.ssh/id_rsa $cont_name:/home/developer/.ssh/

