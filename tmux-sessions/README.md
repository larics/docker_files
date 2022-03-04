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


