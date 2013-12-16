// RemoteControl.cpp : Defines the entry point for the console application.
//

/*	Implements the robot controller 
		The computer acts as a interface that supports multiple different
		controllers. Each controller implements its own functions which
		the interface calls
*/

#include "stdafx.h"

#include "XBoxController.h"
#include "ControllerInterface.h"

int _tmain(int argc, _TCHAR* argv[])
{
	XBoxController* player1 = new XBoxController();

	ControllerInterface control(player1);

	control.Run();

	CommandStruct test = control.pollInput();

	return 0;
}

