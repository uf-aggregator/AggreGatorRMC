#include <ros/ros.h>

#include "ir_processing/ir_reader.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "ir_processing_node");

	IrReader *ir1 = new IrReader(21, 128);
	IrReader *ir2 = new IrReader(22, 128);
	IrReader *ir3 = new IrReader(23, 128);
	return 0;
}
