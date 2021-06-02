# OpenPose docker

Most of the info was gathered from this [link](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/1_prerequisites.md) 
which states prerequisites for building OpenPose from source. 


# Reinstall CUDNN stuff

Because of reinstall it's neccessary to swap between available cudnn7 installations. 
You can do that with following command: 
```
sudo update-alternatives --config libcudnn
```
