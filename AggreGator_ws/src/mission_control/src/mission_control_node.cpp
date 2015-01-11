/*************************
 * PENDING REMOVAL - this program is potentially unneeded
 * as we could just run everything from MissionControl
 *I can't think of much reason why I had this level of abstraction
 *************************/

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mission_control/mission_control.h"

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "mission_control_node");

	MissionControl *mc = new MissionControl();
	mc->StartSenseAct();

	return 0;
}