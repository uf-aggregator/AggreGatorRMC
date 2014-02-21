#include <iostream>
#include <string>
#include <cstring>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ladar/SerialCtrl.h"
#include "ladar/ConnectionUtils.h"
#include "ladar/ScipUtils.h"

using namespace qrk;

void printLadarData(){

}//end print method
	
int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/scan", 1, printLadarData);

	ros::spin();
	
	return 0;
}
