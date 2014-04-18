#include <iostream>
#include <string>
#include "ladar/SDL/SDL.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ladar/ladar_data.h"
#include "ladar/draw.h"

void scanCallback(const sensor_msgs::LaserScan laser){
	
	int numSample = laser.ranges.size();
	float ranges[numSample];
	for(int i = 0; i < numSample; i++){
		ranges[i] = laser.ranges[i];
	}
	float angle_min = laser.angle_min;
	float angle_increment = laser.angle_increment;
	float min_range = laser.range_min;
	float max_range = laser.range_max;
	Ladar *ladar = new Ladar(numSample);
	std::vector<float> slopes = ladar->getSlopes(
	ladar->fivePointAverager(
	ladar->getCoordinates(ranges, numSample, angle_min, angle_increment, min_range, max_range)));
	//  /*
	ladar->print(1);
	std::cout << "---------------------------------------------------------------------" << 
	"\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
	// */
	/*for(int i = 0; i < slopes.size(); i++){
		std::cout << i << ": \t" << slopes.at(i) << std::endl;
	}*/

	//ladar->drawCoordinates();
	
	usleep(1000000);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "ladar_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 1, scanCallback);//change callback function

	ros::spin();
	
	return 0;
}
