#include <iostream>
#include <sstream>
#include "mission_control.h"

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
 *	Start
 *		starts up everything
 *-----------------------------------------*/
void MissionControl::StartSenseAct(){
	int input;
	cout << "Ctrl-C to terminate." << endl;

	while(true) {
		

		
	}//end while

	//clean up anything here
}//end StartSenseAct
