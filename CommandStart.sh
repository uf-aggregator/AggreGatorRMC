#!/‌bin/bash
source /opt/ros/hydro/setup.sh
cd AggreGator_ws
source devel/setup.bash
rosrun image_rotate image_rotate image:=/usb_cam/image_raw __name:=image_rotater _image_transport:=theora
#rosrun image_view image_view image:=/rotated/image
