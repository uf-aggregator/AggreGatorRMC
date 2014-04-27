	/*
	Wheel Controller Node
	Author: Daniel Kelly
	Description: This ROS node catches data from the remote control node,processes it, and sends it out to the i2c node to be used by the wheel motors	*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "motor_controller/WheelMotor.h"
#include "motor_controller/AdaCmd.h"
#include "hardware_interface/GPIO.h"
#include <iostream>
#include "controller.h"

using namespace std;

//Global variables
//float* controllerArray = new float[4]; 	//Array used for performing controller operations on the motor data
float* controlOutput = new float[4]; 	//Array used for organizing the data on i2c for each motor

ros::Subscriber sub; 		//Subscriber object, used for accepting messages from the remote controller

ros::Publisher pub; 		//Publisher object, used for publishing messages to the I2C node

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(0.01);       //time in seconds between sends

SSController leftFrontWheel; 		//create controller object for left front wheel
SSController leftRearWheel; 		//create controller object for left rear wheel
SSController rightRearWheel; 		//create controller object for right rear wheel
SSController rightFrontWheel; 		//create controller object for right front wheel

int motorDir[4] = {0};

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


	//Function that controls the direction of the motors using the GPIO pins on the O-Droid
void setMotorDirection()
{
	for(int i = 0; i < 4; ++i)
	{
		if(controlOutput[i] > 0)
			motorDir[i] = 1;
		else if(controlOutput[i] < 0)
			motorDir[i] = -1;
		else
			motorDir[i] = 0;
	}
	
	
	//Set GPIOs
	switch(motorDir[0])
	{
	//Forward
	case 1:
		setGPIOWrite(LF_A, 0);
		setGPIOWrite(LF_B, 1);
		break;
	
	//Reverse
	case -1:
		setGPIOWrite(LF_A, 1);
		setGPIOWrite(LF_B, 0);
		break;
	//Stop
	default:		
		setGPIOWrite(LF_A, 0);
		setGPIOWrite(LF_B, 0);
	}

	//Set GPIOs
	switch(motorDir[1])
	{
	//Forward
	case 1:
		setGPIOWrite(LR_A, 0);
		setGPIOWrite(LR_B, 1);
		break;
	
	//Reverse
	case -1:
		setGPIOWrite(LR_A, 1);
		setGPIOWrite(LR_B, 0);
		break;
	//Stop
	default:		
		setGPIOWrite(LR_A, 0);
		setGPIOWrite(LR_B, 0);
	}


	//Set GPIOs
	switch(motorDir[2])
	{
	//Forward
	case 1:
		setGPIOWrite(RR_A, 0);
		setGPIOWrite(RR_B, 1);
		break;
	
	//Reverse
	case -1:
		setGPIOWrite(RR_A, 1);
		setGPIOWrite(RR_B, 0);
		break;
	//Stop
	default:		
		setGPIOWrite(RR_A, 0);
		setGPIOWrite(RR_B, 0);
	}

	//Set GPIOs
	switch(motorDir[3])
	{
	//Forward
	case 1:
		setGPIOWrite(RF_A, 0);
		setGPIOWrite(RF_B, 1);
		break;
	
	//Reverse
	case -1:
		setGPIOWrite(RF_A, 1);
		setGPIOWrite(RF_B, 0);
		break;
	//Stop
	default:		
		setGPIOWrite(RF_A, 0);
		setGPIOWrite(RF_B, 0);
	}
}


//Controller logic and math is implemented here.
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

	//Sets direction of motor using GPIO pins
	setMotorDirection();	

}

//Creates a message to be sent to the I2C node.
motor_controller::AdaCmd generateMessage()
{

	motor_controller::AdaCmd msg;

	msg.device = motor_controller::AdaCmd::wheelMotors;
	
	for(int i = 0; i < 4; i++)
	{
	    msg.value.push_back(abs(controlOutput[i]));
	}
	
	//Debug
	/*ROS_INFO("LF: %f, LB: %f, RB: %f, RF: %f", 
		msg.value[0], 
		msg.value[1], 
		msg.value[2], 			
		msg.value[3]);*/

	return msg;

}

//Callback function which fills motorArray with values from the message
void callBack (const motor_controller::WheelMotor& msg)
{
	leftFrontWheel.setU(msg.LF_motorVal * 24.0 / 32768.0); //Set controller inputs
	leftRearWheel.setU(msg.LR_motorVal * 24.0 / 32768.0);
	rightRearWheel.setU(msg.RR_motorVal * 24.0 / 32768.0);
	rightFrontWheel.setU(msg.RF_motorVal * 24.0 / 32768.0);
}

void stopAllMotors()
{
	setGPIOWrite(LF_A, 0);
	setGPIOWrite(LF_B, 0);
	setGPIOWrite(LR_A, 0);
	setGPIOWrite(LR_B, 0);
	setGPIOWrite(RF_A, 0);
	setGPIOWrite(RF_B, 0);
	setGPIOWrite(RR_A, 0);
	setGPIOWrite(RR_B, 0);
}


//MAIN
int main(int argc, char** argv)
{

	ros::init(argc, argv, "wheel_motor_node"); //Initialize node

	ros::NodeHandle n; //Create nodehandle object

	sub = n.subscribe("wheel_motor_rc", 1000, callBack); //Create object to subscribe to topic "wheel_motor_rc"
	
	pub = n.advertise<motor_controller::AdaCmd>("adaFruit",1000); //Create object to publish to topic "I2C"
	while(pub.getNumSubscribers()==0);//Prevents message from sending when publisher is not completely connected to subscriber.
	
	ros::Rate loop_rate(10); //Set frequency of looping. 10 Hz
	
	setGPIOWrite(33,1); //Motor enable

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
				for(int i = 0; i < 4; ++i)
					controlOutput[i] = 0.0;
			}
			else
			{	
				//motor_controller function
				controlFunction();
			}

			pub.publish(generateMessage());
		}
		
			ros::spinOnce();
	}


	stopAllMotors();

	return 0;
}

