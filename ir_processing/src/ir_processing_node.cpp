#include <ros/ros.h>

#include "ir_processing/ir_processing.h"

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "ir_processing_node");

	
	cout << IrProcessing::getMeters() << endl;


	return 0;
}
