#include <ros/ros.h>

#include "ir_processing/ir_processing.h"
#include "hardware_interface/ReadI2C.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "ir_processing_node");


	return 0;
}