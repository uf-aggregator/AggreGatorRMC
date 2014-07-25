#include <iostream>
#include "mission_control/base_mid_handler.h"

//====================================================
//CONSTRUCTORS
//====================================================
/*-----------------------------------------	
 *	BaseMidLevelHandler
 *		sets the class_name 
 *-----------------------------------------*/
BaseMidLevelHandler::BaseMidLevelHandler() {
	class_name = "[BASE MIDLEVEL HANDLER]";
}

//====================================================
//METHODS
//====================================================
/*-----------------------------------------	
 *	getStateId
 *		returns the state_id
 *-----------------------------------------*/
int BaseMidLevelHandler::getStateId() const{
	return state_id;
}

/*-----------------------------------------	
 *	setStateId
 *		sets the state_id
 *-----------------------------------------*/
void BaseMidLevelHandler::setStateId(int newId){
	state_id = newId;
}