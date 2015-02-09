#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <time.h>
#include <stack>
#include "state_machine.h"
#include "../behaviors/behavior_map.h"

StateMachine::StateMachine () {
	//initialize starting index for stateHistory
	currentHistoryIndex = 0;
}

int StateMachine::start(int starting){
	currentState = starting;
	stateHistory[currentHistoryIndex] = currentState;
	currentHistoryIndex++;

	while(currentState > -1){
		std::cout << "Current State was " << currentState << "\n";

		currentState = next();

		std::cout << "Current State is now " << currentState << "\n";

		stateHistory[currentHistoryIndex] = currentState;

		//add behaviors to the queue
		switch(currentState){
			case MOVE:
				behaviorQueue.push(MOVE);
				break;
			case MINE:
				behaviorQueue.push(MINE);
				break;
			case DUMP:
				behaviorQueue.push(DUMP);
				break;
			case WAIT:
				behaviorQueue.push(WAIT);
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
			case MOVE:
				
				break;
			case MINE:
				
				break;
			case DUMP:
				
				break;
			case WAIT:
				
				break;
			default:
				std::cout << "Unknown bhvId: " << executeBhvId << "\n";
				break;
		}

		std::cout << std::endl;
	}
	return 0; //normal termination
}

//Currently just random numbers, but we'll have to make
//actual logic based on ROS values
int StateMachine::next(){
	//subscribe to topics and return the weighted value when this method is called
	ros::NodeHandle nh;

	switch(currentState){
		case MOVE:
			return MINE;
		case MINE:
			return DUMP;
		case DUMP:
			return QUIT;			
		case WAIT:
			return MOVE;
		case QUIT:
			return -1;
		default:
			return MOVE;
	}
}

//CALLBACKS============================================


//UTILITY METHODS======================================

void StateMachine::flush(){
	while(!behaviorQueue.empty()){
		behaviorQueue.pop();
	}
}

void StateMachine::printHistory() {
	int charPerLine = 7;
	for(int i = 0; i < currentHistoryIndex; i++){
		if(charPerLine % i == 0 && i != 0)  std::cout << std::endl;
		std::cout << stateHistory[i] << " ";
	}
}

void StateMachine::retrace(int numBack){
	std::queue<int> temp;
	int limit = currentHistoryIndex - numBack;
	
	if(limit < 0) limit = 0;

	for(int i = limit; i <= currentHistoryIndex; i++){
		temp.push(stateHistory[i]);
	}

	for(int i = 0; i < behaviorQueue.size(); i++){
		temp.push(behaviorQueue.front());
		behaviorQueue.pop();
	}

	while(!temp.empty()){
		behaviorQueue.push(temp.front());
		temp.pop();
	}

	currentHistoryIndex = limit;
}