#include <iostream>
#include <string>
#include <ros/ros.h>
#include <signal.h>
#include "std_msgs/String.h"
#include <common_files/Motor.h>
#include "mission_control/stateMachines/state_machine.h"


using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "mission_control_node");
	ros::NodeHandle nh;

	StateMachine *sm = new StateMachine();
	sm->start(0);

	ros::shutdown();
	return 0;
}
