#include "stdafx.h"

//DEBUG
#include <iostream>
//DEBUG

#include "ControllerInterface.h"

/*
	Constructors and Destructors
*/
//add input device constructor
ControllerInterface::ControllerInterface(InputDevice* input_device) :
 input_device_(NULL)
{
	//pass to attach controller function
	AttachNewController(input_device);
}

//destructor
ControllerInterface::~ControllerInterface()
{
	if(input_device_ != NULL)
		delete input_device_;
}

/*
	General functions
*/

/*
	This function polls input from the attached controller
*/
CommandStruct ControllerInterface::pollInput()
{
	return input_device_->GetInput();
}

//attach a new input device
void ControllerInterface::AttachNewController(InputDevice* input_device)
{
	//delete old controller if not the same
	if(input_device_ != NULL)
	{
		delete input_device_;
		input_device_ = NULL;
	}

	//add new input device
	input_device_ = input_device;
}

//Main loop funciton
void ControllerInterface::Run()
{
	while(input_device_ != NULL)
	{
		WriteCommands(pollInput());
	}
}

//Write out the commands to be sent to communicator
void ControllerInterface::WriteCommands(CommandStruct cmds)
{
	std::cout << "Command struct debug output:\n" <<
		"LeftWheelVelocity: " << cmds.left_wheel_velocity <<
		"\nRightWheelVelocity: " << cmds.right_wheel_velocity <<
		"\nBucketDrumAngle: " << cmds.bucket_angle <<
		"\nDump-Mine: " << cmds.mine_dump <<
		"\nPollPicture: " << cmds.poll_picture <<
		"\nPollLadarData: " << cmds.poll_ladar_data <<
		"\nSwitchOperationMode" << cmds.switch_operation_mode <<
		"\nOperationMode" << cmds.operation_mode;
}