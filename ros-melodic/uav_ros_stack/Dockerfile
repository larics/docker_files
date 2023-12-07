FROM ubuntu:bionic

# Use ARG - persists only during docker build
# https://github.com/moby/moby/issues/4032#issuecomment-192327844
ARG CATKIN_WORKSPACE=uav_ws
ARG DEBIAN_FRONTEND=noninteractive
ARG HOME=/root

# Install all the things to stop docker build from breaking
RUN ln -fs /usr/share/zoneinfo/Europe/Zagreb /etc/localtime && \
    apt-get update && apt-get install -q -y \
    git \
    sudo \
    lsb-release \
    gnupg2 \
    apt-utils \
    dialog \
    curl \
    vim \
    tzdata \
    openssh-client \
    vim \
    git \
    bash-completion && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Download public key for github.com
ENV GIT_SSH_COMMAND="ssh -v"
USER root
RUN --mount=type=ssh \
    mkdir -p $HOME/.ssh/ && \
    ssh-keyscan -H github.com >> $HOME/.ssh/known_hosts 

# Clone uav_ros_stack
WORKDIR $HOME
RUN --mount=type=ssh git clone git@github.com:larics/uav_ros_stack.git -b gitman-ssh-fix

RUN --mount=type=ssh ./uav_ros_stack/installation/install.sh 
RUN --mount=type=ssh  ./uav_ros_stack/installation/workspace_setup.sh $CATKIN_WORKSPACE && \
  # Install uav_ros_stack
  cd $HOME/$CATKIN_WORKSPACE/src && \
# 1 ) Clean the current repository
  catkin clean --yes && \
# 2 ) Activate the deploy profile
  catkin profile set deploy && \
# 3 ) Initial build
  bash -c "source /opt/ros/melodic/setup.bash; catkin build" && \
# 4 ) Link up the stack
  ln -s $HOME/uav_ros_stack && \
  cd uav_ros_stack && gitman update && cd .. && \
# 5 ) Build vrpn_catkin - it has a weird installation process where it is force installed to devel folder - move it to the install folder instead
  bash -c "source /opt/ros/melodic/setup.bash; source $HOME/$CATKIN_WORKSPACE/install/setup.bash; catkin build vrpn_catkin --cmake-args -DCATKIN_DEVEL_PREFIX=$HOME/$CATKIN_WORKSPACE/install" && \
# 6 ) Build ros_vrpn_client with no dependencies
  bash -c "source /opt/ros/melodic/setup.bash; source $HOME/$CATKIN_WORKSPACE/install/setup.bash; catkin build glog_catkin ros_vrpn_client --no-deps" && \
# 7 ) Ignore the already built packages...
  catkin config --blacklist ros_vrpn_client vrpn_catkin catkin_simple glog_catkin && \ 
# 8 ) Build everything else!
  bash -c "source /opt/ros/melodic/setup.bash; source $HOME/$CATKIN_WORKSPACE/install/setup.bash; catkin build" && \
# 9 ) Clean everything except the install folder
  rm -rf ../src/* ../devel ../build ../logs $HOME/uav_ros_stack $HOME/.gitcache

RUN sed -i "s|source $HOME/$CATKIN_WORKSPACE/devel/setup.bash|source $HOME/$CATKIN_WORKSPACE/install/setup.bash|g" $HOME/.bashrc

# export ROS_PACKAGE_PATH for some reason it does
RUN echo 'export ROS_PACKAGE_PATH='"$HOME/$CATKIN_WORKSPACE"'/install/share:$ROS_PACKAGE_PATH' >> $HOME/.bashrc && \
  # Shell scripts are now deleted... :(
  sed -i "s|.*shell_scripts.*||g" $HOME/.bashrc && \
  # Get new shell scripts and source them!
  curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/miscellaneous/shell_additions/shell_scripts.sh >> $HOME/.shell_scripts.sh && \
  echo 'source '"$HOME"'/.shell_scripts.sh' >> $HOME/.bashrc && \
  echo 'export ROS_HOME='"$HOME"'/uav_ws' >> $HOME/.bashrc && \
  # Get dottmux
  rm -rf $HOME/.tmux.conf && \
  curl https://raw.githubusercontent.com/larics/uav_ros_stack/main/installation/dependencies/tmux/dottmux.conf >> $HOME/.tmux.conf && \
  # Add bash completion
  echo " \
if ! shopt -oq posix; then \n\
  if [ -f /usr/share/bash-completion/bash_completion ]; then \n\
    . /usr/share/bash-completion/bash_completion \n\
  elif [ -f /etc/bash_completion ]; then \n\
    . /etc/bash_completion \n\
  fi \n\
fi" >> $HOME/.bashrc


WORKDIR /root

# https://stackoverflow.com/questions/39400886/docker-cannot-resolve-dns-on-private-network - still did not fix
# https://www.linuxuprising.com/2020/07/ubuntu-how-to-free-up-port-53-used-by.html - free up port
# https://github.com/pop-os/pop/issues/773 - Something something, resolv.conf

## SDK
# Position key setting - PC remote
