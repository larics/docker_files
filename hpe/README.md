https://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-using-search-and-replace-in-nanohttps://unix.stackexchange.com/questions/458694/find-and-replace-tabs-usin# Human pose estimation 

This Dockerfile contains neccessary environment to train and test different HPE 
algorithms for GymX. Currently only for pytorch. Intended for training and testing 
different HPE algorithms. 


# Remap data folder to the docker folder in order to have access to data 
# Input folder is always same, output folder is related to the docker container only 

```
docker run -it --network host --gpus all --privileged -e DISPLAY=$DISPLAY -v /dev:/dev -v <in_data_path>:<out_data_path> 
-v /tmp/.X11-unix:/tmp/.X11-unix --name <container_name> <img_name>:<tag_name> 
```

