#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ladar/ladar_data.h"


	
int main(int argc, char **argv){
	ros::init(argc, argv, "ladar_range_node");
	
	//ros::NodeHandle n;
	
	//ros::Subscriber sub = n.subscribe("/scan", 1, getY);//change callback function

	//ros::spin();
	
	return 0;
}
