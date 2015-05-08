/*
	This node is meant to poll any and all sensors on the i2c bus
		-It currently only reads the Mega
		


*/

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "common_files/ReadI2C.h"


int main(int argc, char** argv){
	ros::init(argc, argv, "sensor_node"); 
	ros::NodeHandle n;
	
	ros::ServiceClient read_i2c = n.serviceClient<common_files::CurrentSense>("read_i2c");

	common_files::ReadI2C srv;
	
	while(ros::ok()){
		common_files::ReadI2C srv;
		srv.request.addr = 2; //read from mega
		srv.request.size = 4; //four bytes to read
		if(client.call(srv)){
			ROS_INFO("%d, %d, %d, %d", srv.response.data[0], srv.response.data[1], srv.response.data[2], srv.response.data[3]);
			ROS_INFO("Data was acquired from the i2c bus sucesfully");
		}else{
			ROS_ERROR("Failed to call read_i2c service");
		}
		
		ros::spinOnce();
	}
}
