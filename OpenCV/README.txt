SETTING UP OPENCV ON ROS
=====================================
There are dependency issues with installing OpenCV via Software Center
Too much to install using "apt-get install", so there's this handy script
scalped from the internet.

Additionally, you can 
	sudo apt-get install ros-<distro>-opencv2
	sudo apt-get install ros-<distro>-vision-opencv

The latter is the "bridge" between ROS and OpenCV.


In your CMake, independently have
	find_package(OpenCV REQUIRED)

somewhere. Also, include this in target_link_libraries
	${OpenCV_LIBRARIES}

And in include_directories have
	${OpenCV_INCLUDE_DIRS}