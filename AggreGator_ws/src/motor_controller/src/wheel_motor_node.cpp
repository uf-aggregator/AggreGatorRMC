	/*
	Wheel Controller Node
	Author: Daniel Kelly
	Description: This ROS node catches data from the remote control node,processes it, and sends it out to the i2c node to be used by the wheel motors	*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "motor_controller/WheelMotor.h"
#include "motor_controller/I2CGeneric.h"
#include "hardware_interface/GPIO.h"
#include <iostream>

using namespace std;
//Global variables
int* motorArray =  new int[4]; //Array used for organizing incoming motor data
int* controllerArray = new int[4]; //Array used for performing controller operations on the motor data
int* controlOutput = new int[4];//Array used for organizing the data on i2c for each motor

ros::Subscriber sub; //Subscriber object, used for accepting messages from the remote controller

ros::Publisher pub; //Publisher object, used for publishing messages to the I2C node

//Copies data from motorArray to controllerArray to be used in performing mathematical operations
void dataCopyMagnitude()
{	
	for (int i = 0; i < 4; i++)
		{
			controllerArray[i] = abs(motorArray[i]);
		}

}

//Function that controls the direction of the motors using the GPIO pins on the O-Droid
void setMotorDirection()
{
	
	for (int i = 0; i < 4; i++)
	{
		if(motorArray[i] == 0) //Brake to GND, write 0 to every pin
		{
			setGPIOWrite(44,0);
			setGPIOWrite(43,0);
			setGPIOWrite(42,0);
			setGPIOWrite(41,0);
			setGPIOWrite(40,0);
			setGPIOWrite(39,0);
			setGPIOWrite(38,0);
			setGPIOWrite(37,0);
		}
		if(i == 0) //Left front motor, INA = GPIO pin 44, INB = GPIO pin 43,
		{
			if(motorArray[i] > 0) //Turn CCW
			{
				setGPIOWrite(44,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(43,1); 
			}
			else if(motorArray[i] < 0) //Turn CW
			{
				setGPIOWrite(44,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(43,0);
			}
		}
		if(i == 1) //Left Rear motor, INA = GPIO pin 42, INB = GPIO pin 41
		{
			if(motorArray[i] > 0) //Turn CCW
			{
				setGPIOWrite(42,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(41,1);
			}
			else if(motorArray[i] < 0) //Turn CW
			{
				setGPIOWrite(42,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(41,0);
			}
		}
		if(i == 2) //Right rear motor, INA  = GPIO pin 40, INB = GPIO pin 39
		{
			if(motorArray[i] > 0) //Turn CW
			{
				setGPIOWrite(40,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(39,0);
			}
			else if(motorArray[i] < 0) //Turn CCW
			{
				setGPIOWrite(40,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(39,1);
			}
		}
		if(i == 3) //Right front motor, INA = GPIO pin 38, INB = GPIO pin 37
		{
			if(motorArray[i] > 0) //Turn CW
			{
				setGPIOWrite(38,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(37,0);
			}
			else if(motorArray[i] < 0) //Turn CCW
			{
				setGPIOWrite(38,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(37,1);
			}
		}
			
	}	
}
//Controller logic and math will go here once it is implemented. This is just for motor_controllering
void controlFunction() 
{
		dataCopyMagnitude();

				//TRANSFER  FUNCTION GOES HERE!!!!!!!!!!!
				//Need to implement some feedback method once the function is implemented as well
		for(int i = 0; i < 4; i++)
		{
			controlOutput[i] = controllerArray[i]*10;
		}
	

}

//Creates a message to be sent to the I2C node.
motor_controller::I2CGeneric createOutput()
{

	motor_controller::I2CGeneric msg;

	/*msg.LF_motorVal = controlOutput[0];//Assign data to the data field
	msg.LR_motorVal = controlOutput[1];
	msg.RR_motorVal = controlOutput[2];
	msg.RF_motorVal = controlOutput[3];*/
	
	return msg;

}

//Callback function which fills motorArray with values from the message
void wheelCallback (const motor_controller::WheelMotor& msg)
{
	
	motorArray[0] = msg.LF_motorVal;
	motorArray[1] = msg.LR_motorVal;
	motorArray[2] = msg.RR_motorVal;
	motorArray[3] = msg.RF_motorVal;

	setMotorDirection(); //Sets direction of motor using GPIO pins
		
	controlFunction();   //motor_controller function

	pub.publish(createOutput());

}


//MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_motor_node"); //Initialize node

	ros::NodeHandle n; //Create nodehandle object

	sub = n.subscribe("wheel_motor_rc", 1000, wheelCallback); //Create object to subscribe to topic "wheel_motor_rc"
	
	pub = n.advertise<motor_controller::I2CGeneric>("I2C",1000); //Create object to publish to topic "I2C"
	
	ros::Rate loop_rate(10); //Set frequency of looping. 10 Hz
	
	setGPIOWrite(36,1); //Motor enable

	ros::spin();
	
	resetGPIO();
	
	return 0;
}
