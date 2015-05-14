#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <time.h>
#include <stack>
#include <common_files/Motor.h>

#include "../behaviors/behaviors.h"
#include "state_dump.h"
#include "state_start.h"
#include "state_orientation.h"
#include "state_mine.h"
#include "state_navigation.h"

#include "state_machine.h"

//constructor
StateMachine::StateMachine () {
	//initialize starting index for stateHistory
	currentHistoryIndex = 0;
}

//primary start method
int StateMachine::start(int startState){
	currentState = startState;
	stateHistory[currentHistoryIndex] = currentState;
	currentHistoryIndex++;

	startStateMachines(currentState);
	
	return 0; //normal termination
}

//function for starting state machine
//primarily to organize blocks of code
int StateMachine::startStateMachines(int startState){
	while(currentState > -1){
		std::cout << "Current State was " << currentState << "\n";
		
		if(startState == -1) currentState = next();
		else startState = -1;

		std::cout << "Current State is now " << currentState << "\n";

		stateHistory[currentHistoryIndex] = currentState;

		executeState(currentState);

		std::cout << std::endl;
	}
}

//executes the passed in state
//for organization purposes
int StateMachine::executeState(int executingState) {
	int loopCnt = 0;

	switch(executingState){
		case START: {
			StartState *ss = new StartState(START);
			
			//loop for attempting to execute state until no errors or loop limit reached
			while( ss->start() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}

			delete ss;
			break;
		}

		case ORIENTATION_START: {
			OrientationState *oss = new OrientationState(ORIENTATION_START);
			while( oss->orientToMine() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete oss;
			break;
		}

		case NAVIGATE: {
			NavigationState *ns = new NavigationState(NAVIGATE);
			while( ns->navigateToMine() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete ns;
			break;
		}

		case MINE: {
			MineState *ms = new MineState(MINE);
			while( ms->mine() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete ms;
			break;
		}

		case GO_HOME: {
			NavigationState *ghs = new NavigationState(GO_HOME);
			while( ghs->navigateToDump() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete ghs;
			break;
		}

		case ORIENTATION_DUMP: {
			OrientationState *ods = new OrientationState(ORIENTATION_DUMP);
			while( ods->orientToDump() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete ods;
			break;
		}

		case DUMP: {
			DumpState *ds = new DumpState(DUMP);
			while( ds->dump() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete ds;
			break;
		}

		case GO_MINE: {
			NavigationState *gms = new NavigationState(GO_MINE);
			while( gms->navigateToMine() != 0 && (loopCnt = check(++loopCnt)) != 0 ){}
			delete gms;
			break;
		}

		default:
			std::cout << "Unknown state: " << currentState << "\n";
			break;
	}
}


//safety check for breaking loops
int StateMachine::check(int value) {
	if(value > LOOP_LIMIT) return value;
	return 0;
}


//Currently just random numbers, but we'll have to make
//actual logic based on ROS values
int StateMachine::next(){
	//subscribe to topics and return the weighted value when this method is called

	switch(currentState){
			case START:
				return ORIENTATION_START;

			case ORIENTATION_START:
				return NAVIGATE;

			case NAVIGATE:
				return MINE;

			case MINE:
				return GO_HOME;

			case GO_HOME:
				return ORIENTATION_DUMP;

			case ORIENTATION_DUMP:
				return DUMP;

			case DUMP:
				return GO_MINE;

			case GO_MINE:
				return MINE;
				
			default:
				return START;
		}
}

//UTILITY METHODS======================================
void StateMachine::printHistory() {
	int charPerLine = 7;
	for(int i = 0; i < currentHistoryIndex; i++){
		if(charPerLine % i == 0 && i != 0)  std::cout << std::endl;
		std::cout << stateHistory[i] << " ";
	}
}
