
#!/bin/bash

PROJECT_NAME=$1
TMUXINATOR_PATH="/home/zozan-server/.config/tmuxinator"

cat ./$1 >>  "${TMUXINATOR_PATH}/${PROJECT_NAME}.yml"
