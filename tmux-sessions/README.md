# tmux-safekill 

Interesting plugin for [tmux-safekill](https://github.com/jlipps/tmux-safekill) but doesn't fit my needs. 

[Tmuxinator cheat sheet](https://gist.github.com/crittelmeyer/5924454be991ed61d6d7) 

[Tmux cheat sheet](https://tmuxcheatsheet.com/) 


# Usage

Currently I use tmuxinator sessions to enable starting of multiple different programs at the same time: 

1. Start bebop simulation / record experiment 
2. Start HPE control
3. Start RTSP streaming 

In order to start rc_experiment run following: 
```
tmuxinator start rc_experiment user_id=<user_id_num> run=r<0||1>
```

In order to start hpe_experiment run following: 
```
tmuxinator start hpe_experiment user_id=<user_id_num> run=h<0||1>
```


# epfl 

`epfl` folder was used for initial experiments at EPFL. 

`epfl-final.yml` is tmuxinator session that is used for EPFL experiments. 

# acore-integration 

`acore-integration` folder contains all tmuxinator sessions for starting 
experiments at EPFL for review meeting. 

## acore-precise-exp.yml

Contains tmux configuration for running auth hpe + our ctl. 

Every git repository used for experimental purposes contains a branch called `acore/integration`.
If it doesn't than `master` branch was used.  

Repositories used: 
 * `uav_ar_gui` --> draw FPV 
 * `hpe_ros_package` --> generate reference from human pose estimation
 * `med_uav_control` --> control UAV with joystick and HPE
 * `bebop_autonomy` --> driver for bebop UAV
 * `ros_rtsp` --> RTSP stream from bebop 
 * `visualanalysis_acw` --> human detection and human pose estimation 

## sb-integration 

Uses different HPE simplebaselines (not AUTH). 

Every git repository used for experimental purposes contains a branch called `acore/integration`. 
If it doesn't than master `branch` was used.  

Repositories used:
 * `uav_ar_gui` --> draw FPV 
 * `hpe_ros_package` --> human pose estimation & reference generation 
 * `med_uav_control` --> control UAV with joystick and HPE 
 * `bebop_autonomy` --> driver for bebop UAV 
 * `ros_rtsp` --> rtsp stream for bebop 
