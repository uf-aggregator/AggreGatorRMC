#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

void printLadarData(const sensor_msgs::LaserScan laser){
	//data members
	std::string range, intensity;
	
	//check if we have ranges. This value will be printed
	//additionally, craft string for size info
	if(laser.ranges.size() > 0) range = "True. ";
	else range = "False. ";
	
	
	//check if we have intensities. This value will be printed
	//additionally, craft string with size info
	if(laser.intensities.size() > 0) intensity = "True. ";
	else intensity = "False. ";


	//PRINT OUT STATEMENTS============================================
	std::cout<<"\n------------------------\nRunning..."<<std::endl;
	std::cout<<"We got ranges? "<<range<<std::endl;
	//Print out ranges
	for(int i = 0; i < laser.ranges.size();i++){
		std::cout<<laser.ranges[i]<<std::endl;
		}
	std::cout<<"We got intensities? "<<intensity<<std::endl;
	//print out intensities
	for(int i = 0; i < laser.ranges.size();i++){
		
		}	
	std::cout<<"------------------------"<<std::endl;
	
}//end print method
	
int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/scan", 1, printLadarData);

	ros::spin();
	
	return 0;
}
