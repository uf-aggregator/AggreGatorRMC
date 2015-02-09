#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "filter/filter.cpp"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

using namespace std;

//last scan 
sensor_msgs::LaserScan last_scan;
//filtered out
std_msgs::Float32MultiArray last_output;

//the filter
Filter filter;

//ladar_node output messages - vector of differences and average difference
//ros::Publisher output;

ros::Time last_time, current_time;
ros::Duration send_time(1);       //time in seconds between sends (send 1 message a sec)

void scanCallback(const sensor_msgs::LaserScan laser){
	if(filter.isReady()){
		//run the filter with the laser scan
		filter.runFilter(laser); 
		
	}
	// if a snapshot has not yet been taken, only save the scan
	
	last_scan = laser;
}

void snapshotCallBack(std_msgs::Bool snapshot){
	if(snapshot.data){
		//set the offset to be the last scan taken
		filter.setOffset(last_scan);
	}
	//else do nothing
}

int main(int argc, char **argv){
	ros::init(argc, argv, "filter_node");
	ros::NodeHandle nh;
	
	//ros::Publisher average_noise = nh.advertise<std_msgs::Float32> ("average_difference", 1);
	
	ros::Publisher output = nh.advertise<std_msgs::Float32MultiArray> ("output", 1);
	
	
	//ladar_node input messages - laser scans and snapshot commands
	ros::Subscriber snapshot_cmd = nh.subscribe("/snapshot", 10, snapshotCallBack);
	ros::Subscriber laser = nh.subscribe("/scan", 10, scanCallback);
	
	// Create a ROS publisher for the output point cloud
	//pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
	
	while(ros::ok()){
		current_time = ros::Time::now();	
		ros::spinOnce();
		
		if(filter.isReady() && current_time - last_time > send_time){
			last_time = ros::Time::now(); // save the time this output was sent
			//output.publish( filter.getFiltered() );
			
			std::vector<float> filtered = filter.getFiltered();
			std_msgs::Float32MultiArray output;
			
			for(int i = 0; i < filtered.size(); i++){
				output.data.push_back(filtered.at(i));	
			}
			
			/*calculate average difference in offset for analyzation
			float summation = 0.0;
			for(int i = 0; i < filtered.size(); i++){
				summation = summation + filtered.at(i);	
			}
			float average = summation / ((float)filtered.size());
			average_noise.publish(average);
			*/
		}
	};
	
	ros::spin();
	return 0;
	
}
