//Controller base class

/*
	This class is the base for all controllers
*/

#pragma once

#include "CommandHolder.h"

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
	virtual CommandHolder* GetInput() = 0;

};