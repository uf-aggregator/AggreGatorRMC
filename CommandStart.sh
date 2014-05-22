#!/‌bin/bash
cd /home/fnivek/GitHub/NASAboticsCode/
source /opt/ros/hydro/setup.sh
cd AggreGator_ws
source devel/setup.bash
export ROS_PACKAGE_PATH="/home/fnivek/GitHub/NASAboticsCode/AggreGator_ws/src:/home/joey/NASAboticsCode/AggreGator_ws/src:/home/viki/GIThub/NASAboticsCode/NASAboticsCode/AggreGator_ws/src:/home/odroid/NASAboticsCode/AggreGator_ws/src:/home/viki/GIThub/NASAboticsCode/AggreGator_ws/src:/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src:/opt/ros/hydro/share:/opt/ros/hydro/stacks"
roslaunch src/rosMultiCommand.launch
#rosrun image_view image_view image:=/rotated/image
