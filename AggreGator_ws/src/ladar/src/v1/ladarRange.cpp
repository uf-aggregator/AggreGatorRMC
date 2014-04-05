#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ladar/ladar_data.h"
#include <unistd.h>

void printLadarData(const sensor_msgs::LaserScan laser) {
	float ranges[laser.ranges.size()];
	for(int i = 0; i < laser.ranges.size(); i++){
		ranges[i] = laser.ranges[i];
	}
	Ladar* ladar = new Ladar(laser.ranges.size());

	std::string toPrint = ladar->coordinatesToString( ladar->
	getCoordinates
		(ranges, laser.ranges.size(),
		laser.angle_min, laser.angle_increment, laser.range_min, laser.range_max)
	)
	;
	//std::cout << toPrint << std::endl;

	usleep(10000000);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "ladar_range_node");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/scan", 1, printLadarData);//change callback function
	ros::spin();

	return 0;
}
