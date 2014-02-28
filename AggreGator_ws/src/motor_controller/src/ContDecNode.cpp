	/*
	PID Controller Node
	Author: Daniel Kelly
	Description: This ROS node catches data from the remote control node,processes it, and sends it out to the i2c node to be used by the motors	*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "motor_controller/motorMSG.h"
#include "motor_controller/I2CMSG.h"
using namespace std;
#include <iostream>

//Global variables
int* motorArray =  new int[4]; //Array used for organizing incoming motor data
int* contArray = new int[4]; //Array used for performing controller operations on the motor data
int* i2cdata = new int[4];//Array used for organizing the data on i2c for each motor
int* i2caddr = new int[4];//Array used for organizing the addresses for i2c of each motor




//Callback function which fills motorArray with values from the message
void PIDCallback (const motor_controller::motorMSG& msg)
{

	motorArray[0] = msg.LF_motorVal;
	motorArray[1] = msg.LR_motorVal;
	motorArray[2] = msg.RR_motorVal;
	motorArray[3] = msg.RF_motorVal;


}

//Copies data from motorArray to contArray to be used in performing mathematical operations
void dataCopy()
{	
	for (int i = 0; i < 4; i++)
		{
			contArray[i] = motorArray[i];
		}

}


//Controller logic will go here once it is implemented. This is just foor testing
void testControlFunction() //does random maths
{
		dataCopy();

		for(int i = 0; i < 4; i++)
		{
			i2cdata[i] = (contArray[i]/2) + 23;
			if(i == 0)
			{
				i2caddr[i] = 0x2560;
			}
			else if(i == 1)
			{
				i2caddr[i] = 0x2561;
			}
			else if(i == 2)
			{
				i2caddr[i] = 0x2562;
			}
			else if(i == 3)
			{
				i2caddr[i] = 0x2563;
			}
	
		}
	
	

}

//Creates a message to be sent to the I2C node.
motor_controller::I2CMSG createI2CMSG(int i)
{

	motor_controller::I2CMSG msg;

	msg.addr = i2caddr[i]; //Assign this address to the addr data field
	msg.data = i2cdata[i];//Assign data to the data field
	
	return msg;

}
//MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "ContDecNode"); //Initialize node

	ros::NodeHandle n; //Create nodehandle object

	ros::Subscriber sub = n.subscribe("motor_rc", 1000, PIDCallback); //Create object to subscribe to topic "motor_rc"
	
	ros::Publisher pub = n.advertise<motor_controller::I2CMSG>("I2C",1000); //Create object to publish to topic "I2C"

	ros::Rate loop_rate(500); //Set frequency of looping. 10 Hz
	
	//Loop is true while ctrl-c isn't pressed, or ros::shutdown() has not been called, or the node has not been kicked off the network.
	while(ros::ok()){

		ros::spinOnce(); //Call the callback function

		testControlFunction();//Test function
		
		for(int i = 0; i < 4; i++)//Loop that creates and publishes the message
			{
			motor_controller::I2CMSG msg;
			msg = createI2CMSG(i);
			pub.publish(msg);
			}
		
		loop_rate.sleep(); //Sleep for the time remaining to allot 10 Hz publishing rate.

	}

	
	return 0;
}
