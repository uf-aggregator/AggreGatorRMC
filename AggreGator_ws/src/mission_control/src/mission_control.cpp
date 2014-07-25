#include <iostream>
#include <sstream>
#include "mission_control/mission_control.h"

using namespace std;

MissionControl::MissionControl(){
	this->debug = false;
	this->class_name = "[MISSION CONTROL]";
	lh = new LadarHandler();
}

MissionControl::MissionControl(bool debug){
	this->debug = debug;
	this->class_name = "[MISSION CONTROL]";
	lh = new LadarHandler();
}

void StateHandlerCallback(const std_msgs::String::ConstPtr& msg){
	cout << msg->data.c_str() << endl;
}
void MissionControl::Publish(){
	pub = nh.advertise<std_msgs::String>("/mission_control", 1000);

	ros::Rate loop_rate(1);

	std_msgs::String msg;
	std::stringstream ss;     
	ss << "YOU HAVE A VIRUS, RUN AWAY.";
	msg.data = ss.str();

	pub.publish(msg);
	ros::spinOnce();
}
void MissionControl::Subscribe(){
	sub = nh.subscribe("/mission_control",1000, StateHandlerCallback);
	ros::spinOnce();
}
void MissionControl::Abort(){
	cout << class_name << " is aborting..." << endl;
}

void MissionControl::Start(){
	int input;
	for(int i = 0;; i++) {
		cout << "Enter 0 to abort. Any other key to continue." << endl;
		cin >> input;

		if(input == 0){ 
			Abort();
			break;
		}
		else {
			//lh->executeActions();
			Publish();
			Subscribe();
		}
	}
}