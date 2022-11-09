# ROS shell scripts
# credit to https://github.com/ctu-mrs/mrs_uav_system

ODOM_TOPIC="topic"
CAMERA_TOPIC="/"


waitForDockerContainer() {

  until [ `docker inspect -f '{{.State.Status}}' mmuav_audio_cont | grep -c running` = 1 ]; do 
    echo "Waiting for docker container to start";
    sleep 1;   
  done
}

waitForRos() {
  until rostopic list > /dev/null 2>&1; do
    echo "waiting for ros"
    sleep 1;
  done
}

waitForPing() {
  while ! ping -c 1 -n -w 1 $1 &> /dev/null
    do
      echo "Waiting for $1 to come online!"
      sleep 1; 
  done
      echo "$1 is online!"
      sleep 1; 
}

waitForSimulation() {
  until timeout 3s rostopic echo /gazebo/model_states -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for simulation"
    sleep 1;
  done
  sleep 1;
}

waitFor() {
  until timeout 3s rostopic echo $1 -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for $1"
    sleep 1;
  done
}

waitForOdometry() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/mavros/local_position/odom -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for odometry"
    sleep 1;
  done
}

waitForXtionCamera() {
  until timeout 3s rostopic echo /camera/rgb/image_raw -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for camera stream"
    sleep 1; 
  done 
}


#waitForMultiMasterDiscovery() {
#  
#}


waitForSLAM(){
  until timeout 3s rostopic echo /$UAV_NAMESPACE/uav/cartographer/pose -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for SLAM odometry"
    sleep 1;
  done
}

waitForGlobal() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/mavros/global_position/local -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for global odometry"
    sleep 1;
  done
}

waitForCarrot() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/carrot/status -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for carrot"
    sleep 1;
  done
}

waitForMavros() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/mavros/state -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for mavros"
    sleep 1;
  done
}

# allows killing process with all its children
killp() {

  if [ $# -eq 0 ]; then
    echo "The command killp() needs an argument, but none was provided!"
    return
  else
    pes=$1
  fi

  for child in $(ps -o pid,ppid -ax | \
    awk "{ if ( \$2 == $pes ) { print \$1 }}")
    do
      # echo "Killing child process $child because ppid = $pes"
      killp $child
    done

# echo "killing $1"
kill -9 "$1" > /dev/null 2> /dev/null
}

waitForSysStatus() {
  until timeout 3s rostopic echo /$UAV_NAMESPACE/mavros/state -n 1 --noarr > /dev/null 2>&1; do
    echo "waiting for /$UAV_NAMESPACE/mavros/state"
    sleep 1;
  done

  while true
    do
      system_status=$(echo "$(rostopic echo /$UAV_NAMESPACE/mavros/state -n 1| grep system_status)" | awk '{print $2}')
      if [[ $system_status == "3" ]]; then
          break
        else
          echo "waiting for system_status"
        fi
      sleep 1
    done
}


