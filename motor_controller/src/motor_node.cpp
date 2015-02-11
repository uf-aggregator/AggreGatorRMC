	/*
	Wheel Controller Node
	Author: Daniel Kelly
	Description: This ROS node catches data from the remote control node,processes it, and sends it out to the i2c node to be used by the wheel motors	*/

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "common_files/Motor.h"
#include "common_files/I2CGeneric.h"
#include "controller.h"

#define epsilon 0.001

using namespace std;

//Global variables
//float* controlOutput = new float[5]; 	//Array used for organizing the data on i2c for each motor
										//Temporarily removing, may be permanent -Joey

ros::Subscriber sub; 		//Subscriber object, used for accepting messages from the remote controller

ros::Publisher pub; 		//Publisher object, used for publishing messages to the I2C node

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(0.01);       //time in seconds between sends

/* Differential equations are hard - let's get the bot moving before we implement Controls engineering
	-Joey
SSController leftFrontWheel; 		//create controller object for left front wheel
SSController leftRearWheel; 		//create controller object for left rear wheel
SSController rightRearWheel; 		//create controller object for right rear wheel
SSController rightFrontWheel; 		//create controller object for right front wheel
*/

//The last message sent from the xbox controller
common_files::Motor lastRemoteMsg;

/*
The ODROID will not need to directly access pins not related to I2C -Joey
enum MotorPins
{
	LF_A = 34,
	LF_B = 35,
	LR_A = 36,
	LR_B = 37,
	RR_A = 41,
	RR_B = 43,
	RF_A = 44,
	RF_B = 45
};

*/
	//Old purpose: Function that controls the direction of the motors using the GPIO pins on the O-Droid
	/*New purpose:
		Direction is no longer controlled with ODroid GPIO pins in the new design
		Instead, the ODroid will send one "direction" byte of data representing the direction for all motors
		This function will generate that byte from a Motor msg from the xbox controller
	-Joey
	*/
unsigned char getMotorDirection(common_files::Motor msg)
{
	//convert the motor msg into an array to easily loop through values
	float controllerOutput[4];
	controllerOutput[0] = msg.leftFront_motorVal;
	controllerOutput[1] = msg.rightFront_motorVal;
	controllerOutput[2] = msg.rightRear_motorVal;
	controllerOutput[3] = msg.leftRear_motorVal;
	
	//first establish whether nor not each motorVal is positive or negative
	//save these values in the motorDir array
	int motorDir[4];
	for(int i = 0; i < 4; ++i)
	{
		if(controllerOutput[i] > 0.001)
			motorDir[i] = 1;
		else
			motorDir[i] = -1;
	}
	
	/* The "direction" byte - each bit represents an individual motor's direction
		1 = forward, 0 = backward
		bit 0: leftFront motor
		bit 1: rightFront motor
		bit 2: rightRear motor
		bit 3: rightFront motor
		bits 4-7: unassigned (can be used for other motors in the future)
	*/
	unsigned char direction = 0b00000000;
	
	switch(motorDir[0])
	{
		//Forward
		case 1:
			direction = direction | 0b00000001; //set bit 0
	
		//Reverse
		case -1:
			direction = direction & 0b11111110; //clear bit 0
			break;
		//Stop
		default:		
			direction = direction | 0b00000001; //set bit 0
	}

	switch(motorDir[1])
	{
		//Forward
		case 1:
			direction = direction | 0b00000010; //set bit 1
		//Reverse
		case -1:
			direction = direction & 0b11111101; //clear bit 1
		//Stop
		default:		
			direction = direction | 0b00000010; //set bit 1
	}	

	switch(motorDir[2])
	{
		//Forward
		case 1:
			direction = direction | 0b00000100; //set bit 2
			break;
	
		//Reverse
		case -1:
			direction = direction & 0b11111011; //clear bit 2
			break;
		//Stop
		default:		
			direction = direction | 0b00000100; //set bit 2
	}
	
	switch(motorDir[3])
	{
		//Forward
		case 1:
			direction = direction | 0b00001000; //set bit 3
			break;
		//Reverse
		case -1:
			direction = direction & 0b11110111; //clear bit 3
			break;
		//Stop
		default:		
			direction = direction | 0b00001000; //set bit 3
	}
		
	//return direction byte
	return direction;
}


//Controller logic and math is implemented here.
/* Save the Controls engineering for later -Joey
void controlFunction() 
{		
	leftFrontWheel.update();
	leftRearWheel.update();
	rightRearWheel.update();
	rightFrontWheel.update();
		
	//Does some scaling on the control output values to get PWM value for AdaFruit downstream
	controlOutput[0] = (leftFrontWheel.getY()[0][0]) * 100 / 24; 
	controlOutput[1] = (leftRearWheel.getY()[0][0]) * 100 / 24; 
	controlOutput[2] = (rightRearWheel.getY()[0][0]) * 100 / 24; 
	controlOutput[3] = (rightFrontWheel.getY()[0][0]) * 100 / 24;

}
*/ 

//takes absolute value of  motorVal [-1, 1] and scales it to [0,255] for Teensy
uint8_t convertTo8bit(float controller_motorVal){	
	return (uint8_t)(abs(controller_motorVal) * 255.0);
}

//Creates a message to be sent to the I2C node based on message from xbox controller
common_files::I2CGeneric generateMessage(common_files::Motor input)
{
	common_files::I2CGeneric msg;

	msg.addr = 1; //all motors are controlled by the same Teensy, address = 1
	
	msg.data.push_back(getMotorDirection(input));
	
	msg.data.push_back(convertTo8bit(input.leftFront_motorVal));
	msg.data.push_back(convertTo8bit(input.rightFront_motorVal));
	msg.data.push_back(convertTo8bit(input.rightRear_motorVal));
	msg.data.push_back(convertTo8bit(input.leftRear_motorVal));
	
	return msg;

}

//Creates a message to be sent to the I2C node
//Turns off all motors
common_files::I2CGeneric generateZeroMessage(){
	common_files::I2CGeneric msg;

	msg.addr = 1; //all motors are controlled by the same Teensy, address = 1
	
	//creates five bytes, one for direction and four for motors, all 0
	for(int i = 0; i < 5; i++)
	{
	    msg.data.push_back(0);
	}

	return msg;

}

//Callback function which fills motorArray with values from the message
void callBack (const common_files::Motor msg){
//	leftFrontWheel.setU(msg.leftFront_motorVal * 24.0 / 32768.0); //Set controller inputs
//	leftRearWheel.setU(msg.leftRear_motorVal * 24.0 / 32768.0);
//	rightRearWheel.setU(msg.rightRear_motorVal * 24.0 / 32768.0);
//	rightFrontWheel.setU(msg.rightFront_motorVal * 24.0 / 32768.0);
// Let's save Controls for later -Joey
	lastRemoteMsg = msg;
}

//MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "motor_node"); //Initialize node

	ros::NodeHandle n; //Create nodehandle object

	sub = n.subscribe("motor_rc", 1000, callBack); //Create object to subscribe to topic "motor_rc"
	
	pub = n.advertise<common_files::I2CGeneric>("write_i2c",1000); //Create object to publish to topic "write_i2c"
	while(pub.getNumSubscribers()==0);//Prevents message from sending when publisher is not completely connected to subscriber.
	
	//ros::Rate loop_rate(10); //Set frequency of looping. 10 Hz
	
//	setGPIOWrite(33,1); //ODROID does not use GPIO pins for motor enables - Joey

	while(ros::ok())
	{
		//Update time
		current_time = ros::Time::now();
		//Check if interval has passed
		if(current_time - last_time > update_rate)
		{
			//Reset time
			last_time = current_time;
			if(sub.getNumPublishers() == 0) //In case of loss of connection to publisher, set controller outputs to 0
			{
				ROS_WARN("Loss of wheel motor controller input!");
				/* Let's save Controls engineering for later -Joey
				leftFrontWheel.setU(0); //Set controller inputs
				leftRearWheel.setU(0);
				rightRearWheel.setU(0);
				rightFrontWheel.setU(0);
				*/
			}

			//controlFunction(); //Motor controller function
			pub.publish(generateMessage(lastRemoteMsg));
		}		
			ros::spinOnce();
	}
	
	//shut down if ros not ok
	ROS_WARN("Shutting down motors");
	
	//controlFunction();
	pub.publish(generateZeroMessage());

	return 0;
}
