#include <iostream>
#include "mission_control/state_handler.h"

using namespace std;

//====================================================
//CONSTRUCTORS
//====================================================
//Default Constructor
StateHandler::StateHandler(){
	//change these values to the state enum
	this->current_state = 0;
	this->prev_state = 0;
}

//====================================================
//METHODS
//====================================================
/*-----------------------------------------	
 *	getCurrentState
 *		returns the current state
 *-----------------------------------------*/
int StateHandler::getCurrentState() const {
	return current_state;
}

/*-----------------------------------------	
 *	getPrevState
 *		return the previous state
 *		used for contextual reasons, e.g. know what you have already done
 *-----------------------------------------*/
int StateHandler::getPrevState() const {
	return prev_state;
}

/*-----------------------------------------	
 *	getStateHistory
 *		returns a vector of all states in ascending order
 *		for decision making or debugging
 *-----------------------------------------*/
vector<int> StateHandler::getStateHistory() const {
	return state_history;
}

/*-----------------------------------------	
 *	setCurrentState
 *		changes the state to the argument
 *-----------------------------------------*/
void StateHandler::setCurrentState(int newCurrentState) {
	this->current_state = newCurrentState;
}

/*-----------------------------------------	
 *	nextState
 *		decides what state to progress to based on instance data
 *-----------------------------------------*/
int StateHandler::nextState(){
	//will work on logic here
}