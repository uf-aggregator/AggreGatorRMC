/*
	Xbox controller

*/


#include "XBoxController.h"
#include <cmath>

//Default constructor
XBoxController::XBoxController():
	mine_(true)
{}

CommandHolder* XBoxController::GetInput()
{
	//DWORD is definded as unsigned long
	DWORD result;
	//XINPUT_STATE is a struct that holds data about the controller input
	XINPUT_STATE state;
	
	//get state of controller
	result = XInputGetState( 0, &state);	//0 for the first attached controller

	CommandHolder* newCmds = new CommandHolder();

	if(result == ERROR_SUCCESS)
	{
		//switch between mining to dumping with right sholder button
		if((state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) == XINPUT_GAMEPAD_RIGHT_SHOULDER)
		{
			right_shoulder_pressed_ = true;
		}
		else if (right_shoulder_pressed_)			//switch on buton release 
		{
			mine_ = !mine_;							//switch between mining and dumping
			right_shoulder_pressed_ = false;		//reset button
		}

		//Switch between raise to lower with left sholder button
		if((state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) == XINPUT_GAMEPAD_LEFT_SHOULDER)
		{
			left_shoulder_pressed_ = true;
		}
		else if(left_shoulder_pressed_)
		{
			left_shoulder_pressed_ = false;
			lift_bucket_ = !lift_bucket_;
		}

		//Poll all data
		if((state.Gamepad.wButtons & XINPUT_GAMEPAD_A) == XINPUT_GAMEPAD_A)
		{
			newCmds->TakePicture();
			newCmds->PollLadarData();
		}
		else if((state.Gamepad.wButtons & XINPUT_GAMEPAD_X) == XINPUT_GAMEPAD_X)
			newCmds->TakePicture();
		else if((state.Gamepad.wButtons & XINPUT_GAMEPAD_Y) == XINPUT_GAMEPAD_Y)
			newCmds->PollLadarData();

		//Stering control
		//32767 is the maximum value of the thumb sticks
		//If greater than dead zone, output equals ((in - deadzone)/(maxValue - deadzone))^2
		int thumbstick = state.Gamepad.sThumbLY;

		//std::cout << thumbstick << std::endl;
		if(abs(thumbstick) > kDeadzone)
		{
			if(thumbstick > 0)
			{
				newCmds->SetLeftWheelsVelocity(
					pow(
					(float)(thumbstick - kDeadzone) / (kMaxThumstickValue - kDeadzone), /*^*/2));
			}
			else
			{
				newCmds->SetLeftWheelsVelocity(-1*
					pow(
					(float)(abs(thumbstick) - kDeadzone) / (kMaxThumstickValue - kDeadzone), /*^*/2));
			}
		}
		else
		{
			newCmds->SetLeftWheelsVelocity(0.0f);		//set to zero if in deadzone
		}

		//right thumstick
		thumbstick = state.Gamepad.sThumbRY;
		if(abs(thumbstick) > kDeadzone)
		{
			if(thumbstick > 0)
			{
				newCmds->SetRightWheelsVelocity(
					pow(
					(float)(thumbstick - kDeadzone) / (kMaxThumstickValue - kDeadzone), /*^*/2));
			}
			else
			{
				newCmds->SetRightWheelsVelocity(-1*
					pow(
					(float)(abs(thumbstick) - kDeadzone) / (kMaxThumstickValue - kDeadzone), /*^*/2));
			}
		}
		else
		{
			newCmds->SetRightWheelsVelocity(0.0f);		//set to zero if in deadzone
		}

		//mine and dump
		if(mine_)
			newCmds->SetBucketMineControl((float)state.Gamepad.bRightTrigger / kMaxTriggerValue);
		else
			newCmds->SetBucketMineControl((float)state.Gamepad.bRightTrigger / -kMaxTriggerValue);

		//lift and lower bucket drum
		if(lift_bucket_)
			newCmds->SetBucketPitchControl((float)state.Gamepad.bLeftTrigger / kMaxTriggerValue);
		else
			newCmds->SetBucketPitchControl((float)state.Gamepad.bLeftTrigger / -kMaxTriggerValue);
	}
	else
	{
		// Do nothing
		// leave defaults in cmds
	}

	return newCmds;
}