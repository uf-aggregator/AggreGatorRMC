	/*
	Wheel Controller Node
	Author: Daniel Kelly
	Description: This ROS node catches data from the remote control node,processes it, and sends it out to the i2c node to be used by the wheel motors	*/

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "common_files/Motor.h"
#include "common_files/WriteI2C.h"
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
ros::Duration update_rate(0.1);       //time in seconds between sends

//The last message sent from the xbox controller
common_files::Motor lastRemoteMsg;

/*
	Direction is no longer controlled with ODroid GPIO pins in the new design
	Instead, the ODroid will send one "direction" byte of data representing the direction for all motors
	This function will generate that byte from a Motor msg from the xbox controller
		-Joey
*/
unsigned char getMotorDirection(common_files::Motor msg)
{
	double left = msg.leftFront_motorVal;
	double right = msg.rightFront_motorVal;
	//first establish whether nor not each motorVal is positive or negative
	//save these values in the motorDir array

	if(left < 0.0){
		if(right < 0.0){
			//both going backwards
			return 14;
		}else{
			//right forwards, left backwards
			return 6;
		}
	}else{
		if(right < 0.0){
			//left forwards, right backwards
			return 10;
		}else{
			//both forwards
			return 2;
		}
	}

}


//takes absolute value of  motorVal [-1, 1] and scales it to [0,255] for Teensy
uint8_t convertTo8bit(float controller_motorVal){	
	if(abs(controller_motorVal) < 0.15){
		return 0;
	}else{
		return (uint8_t)(abs(controller_motorVal) * 255.0);
	}
}

//Creates a message to be sent to the I2C node based on message from xbox controller
common_files::WriteI2C generateMessage(common_files::Motor input)
{
	common_files::WriteI2C msg;

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
common_files::WriteI2C generateZeroMessage(){
	common_files::WriteI2C msg;

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
	lastRemoteMsg = msg;
}

//MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "motor_node"); //Initialize node

	ros::NodeHandle n; //Create nodehandle object

	sub = n.subscribe("motor_rc", 1000, callBack); //Create object to subscribe to topic "motor_rc"
	
	pub = n.advertise<common_files::WriteI2C>("write_i2c",1000); //Create object to publish to topic "write_i2c"
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
