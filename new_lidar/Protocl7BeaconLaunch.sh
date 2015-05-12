#!/bin/bash
source ~/AggreGator_ws/devel/setup.bash
export ROS_IP=192.168.0.24
export ROS_MASTER_URI="http://192.168.0.24:11311"
export | grep ROS
roslaunch ~/AggreGator_ws/src/AggreGatorRMC/new_lidar/beacon.launch
