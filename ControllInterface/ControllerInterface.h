/*
	The ControllerInteface acts as the "Connector" for all input devices
	The interface defines what services all controllers must provide
	It can only have one input device attached at a time, however;
	an input device may be composed of multiple hardware input devices
*/

#pragma once

#include "CommandStruct.h"
#include "InputDevice.h"

class ControllerInterface
{
	/*
		Private Variables
	*/
private:
	InputDevice* input_device_;

	/*
		Constructors and Destructors
	*/
public:
	//Sets the initial controller
	ControllerInterface(InputDevice* input_device);

	//Destructor
	~ControllerInterface();

	/*
		Functions
	*/
public:
	//Polls the current input device for new input
	CommandStruct pollInput();

	//Switches the controller
	void AttachNewController(InputDevice* input_device);

	//this funciton starts a loop that runs until the end of the program
	void Run();

	//Write command
	void WriteCommands(CommandStruct cmds);
};