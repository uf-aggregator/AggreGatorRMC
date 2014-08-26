#!/bin/bash

#clear console
clear

option="y"
echo "Installing third party packages for ROS Hydro"
echo "Would you like to be notified of each installation? y/n"
read option

#install all third party packages
echo ""
if [ $option == "y" ]; then
        read -p "Installing joy. Press enter to continue...Ctrl-C to cancel"
fi
sudo apt-get install ros-hydro-joy

echo ""
if [ $option == "y" ]; then
        read -p "Installing image_common. Press enter to continue...Ctrl-C to cancel"
fi
sudo apt-get install ros-hydro-image-common

echo ""
if [ $option == "y" ]; then
        read -p "Installing usb_cam. Press enter to continue...Ctrl-C to cancel"
fi
sudo apt-get install ros-hydro-usb-cam

echo ""
if [ $option == "y" ]; then
        read -p "Installing urg_node. Press enter to continue...Ctrl-C to cancel"
fi
sudo apt-get install ros-hydro-urg-node

echo ""
if [ $option == "y" ]; then
	read -p "Installing urg-c. Press enter to continue...Ctrl-C to cancel"
fi
sudo apt-get install ros-hydro-urg-c

