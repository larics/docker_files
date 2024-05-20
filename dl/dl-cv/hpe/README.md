# OpenPose 

`openpose_x64` is OpenPose container for the Linux x64 (Normal PC) 
`openpose_arm` is OpenPose container for the ARM processor (Jetson Xavier)


## openpose_arm

For the openpose_arm it is crucial to set `LD_LIBRARY_PATH` and `PATH` variable, 
and set in `cmake-gui` USE_CUDNN on `False` and compute_capability to `72` 

 

