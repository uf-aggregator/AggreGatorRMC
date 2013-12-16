#pragma once

#include "stdafx.h"
#include "OperationModeEnum.h"

struct CommandStruct
{
	/*
		Mechanical actuation controlls
	*/
	//Wheels
	float left_wheel_velocity;		//controls the speed and direction of the left wheels
	float right_wheel_velocity;		//Controls the speed and direction of the right wheels

	//Bucket Drum
	float bucket_angle;				//Controls the speed and direction of the bucket drum arm
	float mine_dump;					//Controls the speed of mining or dumping negative values
									//corespond to dumping while positive values are mining

	/*
		Electrical/Computer controlls
	*/
	bool poll_picture;				//Polls a picture from the camera on the NASAbot
	bool poll_ladar_data;				//Polls the proccesed LadarData from the NASAbot
	bool switch_operation_mode;		//If true then the NASAbot will change opperation mode
	OperationMode operation_mode;	//If operation mode switch is true then switch to this mode


	/*
		Constructor
	*/
	CommandStruct():
		left_wheel_velocity(0.0f),		right_wheel_velocity(0.0f),
		bucket_angle(0.0f),				mine_dump(0.0f),
		poll_picture(false),			poll_ladar_data(false),
		switch_operation_mode(false),	operation_mode(AUTONOMOUS)

	{}

};