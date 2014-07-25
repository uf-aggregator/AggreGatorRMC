#include <iostream>
#include "mission_control/base_mid_handler.h"

BaseMidLevelHandler::BaseMidLevelHandler() {
	class_name = "[BASE MIDLEVEL HANDLER]";
	std::cout << class_name << " initialized" << std::endl;
}

int BaseMidLevelHandler::getStateId() const{
	return state_id;
}
void BaseMidLevelHandler::setStateId(int newId){
	state_id = newId;
}