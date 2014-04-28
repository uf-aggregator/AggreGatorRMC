#include <iostream>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "ladar/processed_data.h"
 
void testCallback(const ladar::processed_data data){
	ROS_INFO("Connection: ", data.connection);
}
int main(int argc, char **argv){
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("ladar_info", 1, testCallback);

	ros::spin();

	return 0;

}