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

which basically add your private keys during docker image build. 


## TODO: 
- [x] Build image with SSH 
- [x] Decouple build commands (if one fails, everything fails) (install/catkin build) 
- [ ] Squash image and push it on Dockerhub for students 
- [ ] Use it as base image for aerial_manipulation for students 

