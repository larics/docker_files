#!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env SSH_AUTH_SOCK=/ssh-agent \
    --env XAUTHORITY=${XAUTH} \
    --env TERM=xterm-256color \
    --volume="/dev:/dev" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
    --volume $XSOCK:$XSOCK:rw \
    --volume $XAUTH:$XAUTH:rw \
    --net=host \
    --privileged \
    --gpus all \
    --name ros_realsense_cont \
    ros_realsense_img:latest

