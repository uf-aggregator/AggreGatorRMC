#include <iostream>
#include <string>
#include "ladar/SDL/SDL.h"
#include "ros/ros.h"
#include "ladar/ladar.h"
#include "ladar/localization.h"

/*================================================
 *Main Method
 *================================================*/
int main(int argc, char **argv){
	ros::init(argc, argv, "ladar_node");
	Ladar *ladar = new Ladar();
	ladar->sub = ladar->nh.subscribe("/scan", 1, ladar->scanCallback);
	ros::spin();
	return 0;
}
