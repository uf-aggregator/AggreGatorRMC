#include "ros/ros.h"
#include "std_msgs/String.h"

#include "i2c/I2CMSG.h"
#include "i2c/ina226.h"

//define functions for each topic here
void PIDCallback(const i2c::I2CMSG& msg){
}

int main(int argc, char **argv){
	ros::init(argc, argv, "i2c_node");
	ros::NodeHandle n;
	
	//add subscribed topics here
	ros::Subscriber subBucket = n.subscribe("I2C", 1000, PIDCallback);
	
	while(ros::ok()){
		std::cout << "About to spin" << std::endl;
		ros::spinOnce();
	}//endwhile
	
	return 0;
}
