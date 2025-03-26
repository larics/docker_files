# How to build image with SSH keys


In order to build docker image with SSH keys, you can use this command: 

```
DOCKER_BUILDKIT=1 docker build -t <img_name> --ssh default=${SSH_AUTH_SOCK} .
```

You can use ${SSH_AUTH_SOCK} after executing following commands: 
```
eval $(ssh-agent)
ssh-add ~/.ssh/id_rsa
```
To be consistent with the `first_run.sh` run: 
```
DOCKER_BUILDKIT=1 docker build -t uam_ros_img:latest --ssh default=${SSH_AUTH_SOCK} .
```
which enables cloning repos that require ssh keys (git@github.com format). 

After building docker image, you can run it with: 
```
./first_run.sh
```

In order to run aerial manipulator run following script 
```
cd /root/uav_ros_simulation/startup/kopterworx_arm_one_flying
./start.sh 
```
 
By default it starts tmuxintor. 

Navigate tmuxinator panes with: 
```
Ctrl + B and then arrow (if you want to switch between panes) 
Ctrl + B and then number (4 for example arm pane)
```

Ctrl + B don't have to be pressed simultaneousy. Press Ctrl, 
and then press B. 
