#include <ros/ros.h>

#include "sensors/ir_reader.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "ir_node");

	ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("/", 1, IrReader::publishDistances);
	ros::spin();
	return 0;
}
