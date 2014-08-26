#include <ros/ros.h>
#include <iostream>
#include <state_machine.h>

StateMachine::StateMachine () {
	//initialize all the state definitions
	mine.stateId = 2;
	mine.name = "Mine";
	mine.behavior = {-1, -1, -1};

	dump.stateId = 3;
	dump.name = "Dump";
	dump.behavior = {-1, -1, -1};

	wait.stateId = 0;
	wait.name = "Wait";
	wait.behavior = {-1, -1, -1};

	move.stateId = 1;
	move.name = "Move";
	move.behavior = {-1, -1, -1};

	//initialize starting index for stateHistory
	currentHistoryIndex = 0;
}

int StateMachine::start(int starting){
	currentState = starting;
	while(currentState > 0){
		currentState = next();
		
		//execute behaviors
	}
	return 0; //normal termination
}

int StateMachine::next(){

	return -1;
}
void StateMachine::printHistory() {
	
}

void StateMachine::retrace(int numBack){

}