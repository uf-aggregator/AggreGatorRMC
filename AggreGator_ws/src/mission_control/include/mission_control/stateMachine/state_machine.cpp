#include <ros/ros.h>
#include <iostream>
#include "state_machine.h"

StateMachine::StateMachine () {
	//initialize all the state definitions
	mine.stateId = 2;
	mine.name = "Mine";
	mine.behaviorCount = 2;
	mine.behavior = new int[mine.behaviorCount];
	mine.behavior[0] = -1;
	mine.behavior[1] = -1;

	dump.stateId = 3;
	dump.name = "Dump";
	dump.behaviorCount = 2;
	dump.behavior = new int[dump.behaviorCount];
	dump.behavior[0] = -1;
	dump.behavior[1] = -1;

	wait.stateId = 0;
	wait.name = "Wait";
	wait.behaviorCount = 1;
	wait.behavior = new int[wait.behaviorCount];
	wait.behavior[0] = -1;

	move.stateId = 1;
	move.name = "Move";
	move.behaviorCount = 1;
	move.behavior = new int[move.behaviorCount];
	move.behavior[0] = -1;

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
	int charPerLine = 7;
	for(int i = 0; i < currentHistoryIndex; i++){
		if(charPerLine % i == 0 && i != 0)  std::cout << std::endl;
		std::cout << stateHistory[i] << " ";
	}
}

void StateMachine::retrace(int numBack){

}