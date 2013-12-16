/*
	Xbox controller

*/

#include "stdafx.h"
#include "XBoxController.h"
#include <cmath>

//Default constructor
XBoxController::XBoxController():
	mine_(true)
{}

CommandStruct XBoxController::GetInput()
{
	//DWORD is definded as unsigned long
	DWORD result;
	//XINPUT_STATE is a struct that holds data about the controller input
	XINPUT_STATE state;
	
	//get state of controller
	result = XInputGetState( 0, &state);	//0 for the first attached controller

	CommandStruct newCmds;

	if(result == ERROR_SUCCESS)
	{
		//32767 is the maximum value of the thumb sticks
		newCmds.left_wheel_velocity = (float)state.Gamepad.sThumbLY / 32767;
		newCmds.right_wheel_velocity = (float)state.Gamepad.sThumbRX / 32767;

		
		newCmds.bucket_angle = (float)state.Gamepad.bRightTrigger;

	}
	else
	{
		// Do nothing
		// leave defaults in cmds
	}

	return newCmds;
}