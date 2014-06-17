#include <iostream>
#include "mission_control/mission_control.h"

MissionControl::MissionControl(){
	this->debug = false;
}

MissionControl::MissionControl(bool debug){
	this->debug = debug;
}

void MissionControl::Abort(){

}

void MissionControl::Start(){
	
}