#!/usr/bin/env bash

export ROS_MASTER_URI=http://192.168.4.20:11311
export ROS_IP=192.168.4.20

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)" 
source $DIR/../devel/setup.bash
roslaunch argo_mini startup.launch mapping:=true
