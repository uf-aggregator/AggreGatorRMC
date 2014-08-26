#Third_party_packages
------------------------------
For some reason, the files for the packages were not properly committed to GitHub. Forgoing adding them to the repo and added a bash script to install files. Internet connection required.
<br>
###Summary

#####image_commmon
This ROS package performs operations like transforms, corrections, and compressions to any image topics. This package is actually a series of different packages. We use image _ transport and image _ common [See their wiki](http://wiki.ros.org/image_common).

#####laser_proc
This is a series of laser scan utilities.
[See their wiki](http://wiki.ros.org/sensor_msgs)

#####urg_c
Library for interfacing with a laser range finder. For a full package, see `urg_node`. [See their wiki](https://github.com/ros-drivers/urg_c)

#####urg_node
This is the node for getting data from the LADAR.
[See their wiki](http://wiki.ros.org/urg_node)

#####usb_cam
This package reads in streaming video or images and broadcasts published to ROS.
[See their wiki](http://wiki.ros.org/usb_cam)
<br>
###Compatibility
* __image common__: groovy, hydro, indigo
* __joy__: groovy, hydro, indigo
* __laser proc__: groovy, hydro
* __urg c__: groovy, hydro, indigo
* __urg node__: groovy, hydro, indigo
* __usb cam__: groovy, hydro, indigo
