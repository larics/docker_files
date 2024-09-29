#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

echo "$0: Installing tmux and tmuxinator"
sudo apt-get -y install tmux tmuxinator

FILE=$HOME/.tmux.conf
if [ -e "$FILE" ]; then
  echo "$0: .tmux.conf exists, not copying"
else
  echo "$0: copying .tmux.conf"
  ln -sf $MY_PATH/dottmux.conf $FILE
fi