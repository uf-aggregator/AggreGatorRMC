#include <ros/ros.h>

#include "ir_processing/ir_reader.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "ir_processing_node");

	ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("/", 100, IrReader::publishDistances);
	ros::spin();
	return 0;
}
