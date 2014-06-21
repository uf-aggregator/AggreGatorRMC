#include <iostream>
#include "mission_control/state_handler.h"

StateHandler::StateHandler(){
	this->current_state = 0;
	this->prev_state = 0;
}

int StateHandler::getCurrentState() const {
	return current_state;
}

int StateHandler::getPrevState() const {
	return prev_state;
}

vector<int> StateHandler::getStateHistory() const {
	return state_history;
}

void StateHandler::setCurrentState(int newCurrentState) {
	this->current_state = newCurrentState;
}