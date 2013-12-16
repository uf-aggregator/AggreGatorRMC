/*
	Implements an X Box controller as a input device
*/

#include "stdafx.h"
#include "InputDevice.h"
#include <Windows.h>	//required for Xinput
#include <Xinput.h>		//library for working with Xbox controllers

#pragma comment(lib, "XInput.lib")

class XBoxController: public InputDevice
{
	/*
		Private variables
	*/
private:
	bool mine_;

	/*
		Constructors and Destructors
	*/
public:
	XBoxController();

	/*
		Function to obtain input from the xBox controller
	*/
public:
	virtual CommandStruct GetInput();
};