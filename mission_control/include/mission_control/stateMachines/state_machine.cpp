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
			case START:{
				StartState *ss = new StartState(START);
				delete ss;
				break;
			}
			case ORIENTATION_START:{
				OrientationState *oss = new OrientationState(ORIENTATION_START);
				delete oss;
				break;
			}
			case NAVIGATE:{
				NavigationState *ns = new NavigationState(NAVIGATE);
				delete ns;
				break;
			}
			case MINE:{
				MineState *ms = new MineState(MINE);
				delete ms;
				break;
			}
			case GO_HOME:{
				NavigationState *ghs = new NavigationState(GO_HOME);
				delete ghs;
				break;
			}
			case ORIENTATION_DUMP:{
				OrientationState *ods = new OrientationState(ORIENTATION_DUMP);
				delete ods;
				break;
			}
			case DUMP:{
				DumpState *ds = new DumpState(DUMP);
				delete ds;
				break;
			}
			case GO_MINE:{
				NavigationState *gms = new NavigationState(GO_MINE);
				delete gms;
				break;
			}
			default:
				std::cout << "Unknown state: " << currentState << "\n";
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

//CALLBACKS============================================


//UTILITY METHODS======================================
void StateMachine::printHistory() {
	int charPerLine = 7;
	for(int i = 0; i < currentHistoryIndex; i++){
		if(charPerLine % i == 0 && i != 0)  std::cout << std::endl;
		std::cout << stateHistory[i] << " ";
	}
}
