#include <iostream>
#include <string>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <common_files/Motor.h>
#include "mission_control/stateMachines/state_machine.h"


using namespace std;

ros::Publisher pub;

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(5.0);  //turn for 1 second

int main(int argc, char **argv){
	ros::init(argc, argv, "mission_control_node");
	ros::NodeHandle nh;

	StateMachine *sm = new StateMachine();
	sm->start(0);

	return 0;
}
