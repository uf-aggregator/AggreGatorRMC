#include <ros/ros.h>
#include <iostream>
#include "state_machine.h"
#include <cstdlib>
#include <stdlib.h>
#include <time.h>

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
	stateHistory[currentHistoryIndex] = currentState;
	currentHistoryIndex++;

	while(currentState > -1){
		currentState = next();
		stateHistory[currentHistoryIndex] = currentState;

		//execute behaviors
		std::cout << "Current State is " << currentState << "\n";

		//add behaviors to the queue
		switch(currentState){
			case 1:
				pushToQueue(move.behavior, move.behaviorCount);
				break;
			case 2:
				pushToQueue(mine.behavior, mine.behaviorCount);
				break;
			case 3:
				pushToQueue(dump.behavior, dump.behaviorCount);
				break;
			case 0:
				pushToQueue(wait.behavior, wait.behaviorCount);
				break;
			default:
				std::cout << "Unidentified state: " << currentState << "\n";
				return -1;
		}

		//execute behaviors
		/* It's only doing one at a time, because it's possible the queue may need
		 * to be cleared. That is, during emergency situations, e.g. obstacle ahead.
		 */
		int executeBhvId = behaviorQueue.front();
		behaviorQueue.pop();

		switch(executeBhvId){
			default:
				std::cout << "Executing bhvId " << executeBhvId << "\n";
				break;
		}
	}
	return 0; //normal termination
}

//Currently just random numbers, but we'll have to make
//actual logic based on ROS values
int StateMachine::next(){
	//subscribe to topics and get message whnever this method is called

	//ros::NodeHandle nh;
	//ros::Subscriber I2CListener;

	return rand() % 5;
}

//CALLBACKS============================================



//UTILITY METHODS======================================

bool StateMachine::pushToQueue(int* ids, int numIds) {
	for(int i = 0; i < numIds; i++){
		try{
			behaviorQueue.push(ids[i]);
			std::cout << "Pushing " << ids[i] << " to behaviorQueue\n";
		} catch(...) {
			std::cout << "Pushing to queue failed." << "\n";
			return false;
		}
	}

	return true;
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