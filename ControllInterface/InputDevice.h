//Controller base class

/*
	This class is the base for all controllers
*/

#pragma once

#include "CommandStruct.h"

class InputDevice 
{

public:
	/*
		Constructors & Destructors
	*/
	//not needed due to this being an abstract/interface class

	/*
		Virtual Functions
	*/
	virtual CommandStruct GetInput() = 0;

};