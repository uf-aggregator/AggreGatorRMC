#include <iostream>
#include <string>
#include "ladar/SDL/SDL.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ladar/ladar_data.h"
#include "ladar/localization.h"
#include "ladar/processed_data.h"
#include "ladar/conversions.h"

//TODO Refactor into a ladar_node class

/*================================================
 *Callback Methods
 *================================================*/
void scanCallback(const sensor_msgs::LaserScan laser){
	//Prep arguments from LaserScan message
	int numSample = laser.ranges.size();
	float ranges[numSample];
	for(int i = 0; i < numSample; i++){
		ranges[i] = laser.ranges[i];
	}
	float angle_min = laser.angle_min;
	float angle_increment = laser.angle_increment;
	float min_range = laser.range_min;
	float max_range = laser.range_max;

	//instantiate classes
	Ladar *ladar = new Ladar(numSample);
	Conversions *convert = new Conversions();

	//process Ladar data
	std::vector<std::pair<float, float> > coordinates = 
		ladar->getCoordinates(ranges, numSample, angle_min, angle_increment, min_range, max_range);
	
	//print operations  /*
	ladar->print(1);
	std::cout << "---------------------------------------------------------------------" << 
	"\n\n\n\n\n\n\n\n\n\n\n" << std::endl;
	// */

	//graphics test
	//ladar->drawCoordinates(coordinates);

	
	//publish data
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<ladar::processed_data>("ladar_info", 1);
	ros::Rate loop_rate(10);
	
	std::cout << "Starting publishing..."<< std::endl;

	while(ros::ok()) {
		std::cout << "Publishing..." << std::endl;
		
		ladar::processed_data msg;
		msg.connection = true;



		pub.publish(msg);
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	usleep(1000000);
}

/*================================================
 *Main Method
 *================================================*/
int main(int argc, char **argv){
	ros::init(argc, argv, "ladar_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 1, scanCallback);//change callback function

	ros::spin();
	
	return 0;
}
