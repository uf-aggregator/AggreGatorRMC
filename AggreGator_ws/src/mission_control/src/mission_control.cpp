#include <iostream>
#include <sstream>
#include "mission_control/mission_control.h"

using namespace std;
//====================================================
//CONSTRUCTORS
//====================================================
/*-----------------------------------------	
 *	Default Constructor
 *		initialize debug and class_name
 *		creates an instance of all the handlers
 *-----------------------------------------*/
MissionControl::MissionControl(){
	this->debug = false;
	this->class_name = "[MISSION CONTROL]";
	sh = new StateHandler();
}

/*-----------------------------------------	
 *	Debug Constructor
 *		same as default constructor except
 *		can turn on debug mode, e.g. all if(debug) will run
 *-----------------------------------------*/
MissionControl::MissionControl(bool debug){
	this->debug = debug;
	this->class_name = "[MISSION CONTROL]";
	sh = new StateHandler();
}

//====================================================
//CALLBACKS
//====================================================
/*-----------------------------------------	
 *	MissionControlCallback
 *		
 *-----------------------------------------*/
void MissionControlCallback(const std_msgs::String::ConstPtr& msg){
	cout << msg->data.c_str() << endl;
}


//====================================================
//METHODS
//====================================================
/*-----------------------------------------	
 *	Publish
 *		publishes to the /mission_control topic
 *-----------------------------------------*/
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

/*-----------------------------------------	
 *	Subscribe
 *		subsribes to the topics mission_control_node needs
 *-----------------------------------------*/
void MissionControl::Subscribe(){
	//currently only listening to itself
	sub = nh.subscribe("/mission_control",1000, MissionControlCallback);
	ros::spinOnce();
}

/*-----------------------------------------	
 *	Abort
 *		cleans up everything when program terminates expectedly
 *-----------------------------------------*/
void MissionControl::Abort(){
	cout << class_name << " is aborting..." << endl;
}

/*-----------------------------------------	
 *	Start
 *		starts up everything
 *-----------------------------------------*/
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
			Publish();
			Subscribe();
		}
	}//end Start
}