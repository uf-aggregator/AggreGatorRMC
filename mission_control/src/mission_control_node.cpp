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
#include <common_files/Motor.h>

using namespace std;

ros::Publisher pub;

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(5.0);  //turn for 1 second

int main(int argc, char **argv){
	ros::init(argc, argv, "mission_control_node");
	ros::NodeHandle nh;

	MissionControl *mc = new MissionControl();
	mc->start();

	return 0;
}
