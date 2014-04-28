#include <iostream>
#include "std_msgs/"
#include "ros.h"

void testCallback(const ladar::ladar_data data){
	ROS_INFO("Connection: ", data.connection);
}
int main(int argc, char **argv){
	ros::init(argc, argv, "test_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("ladar_data", 1, testCallback);

	ros::spin();

	return 0;

}