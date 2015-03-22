#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <time.h>
#include <stack>
#include "state_machine.h"
#include "../behaviors/behavior_map.h"
#include "../behaviors/motor_utility.h"
#include <common_files/Motor.h>

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
		
		if(starting == -1) currentState = next();
		else starting = -1;

		std::cout << "Current State is now " << currentState << "\n";

		stateHistory[currentHistoryIndex] = currentState;

		switch(currentState){
			case MOVE:
				{
				//motor_utility::write(1.0, -1.0);
				
				break;
				}
			case MINE:
				
				break;
			case DUMP:				
				break;
			case WAIT:
				
				break;
			default:
				std::cout << "Unknown state: " << currentState << "\n";
				break;
		}

		std::cout << std::endl;
	}
	motor_utility::stop();
	return 0; //normal termination
}

//Currently just random numbers, but we'll have to make
//actual logic based on ROS values
int StateMachine::next(){
	//subscribe to topics and return the weighted value when this method is called

	switch(currentState){
		case MOVE:
			return MOVE;
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
void StateMachine::printHistory() {
	int charPerLine = 7;
	for(int i = 0; i < currentHistoryIndex; i++){
		if(charPerLine % i == 0 && i != 0)  std::cout << std::endl;
		std::cout << stateHistory[i] << " ";
	}
}
