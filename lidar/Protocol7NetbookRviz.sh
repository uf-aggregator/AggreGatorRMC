#!/bin/bash
source ~/AggreGator_ws/devel/setup.bash
export ROS_IP=192.168.0.14
export ROS_MASTER_URI=http://192.168.0.24:11311
export LIBGL_ALWAYS_SOFTWARE=1
export | grep ROS
rosrun rviz rviz

