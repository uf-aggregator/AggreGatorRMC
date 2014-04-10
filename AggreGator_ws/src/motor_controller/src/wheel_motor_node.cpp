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
int* motorArray =  new int[4]; //Array used for organizing incoming motor data
int* controllerArray = new int[4]; //Array used for performing controller operations on the motor data
float* controlOutput = new float[4];//Array used for organizing the data on i2c for each motor

ros::Subscriber sub; //Subscriber object, used for accepting messages from the remote controller

ros::Publisher pub; //Publisher object, used for publishing messages to the I2C node


	//Function that controls the direction of the motors using the GPIO pins on the O-Droid
void setMotorDirection()
{
	
	for (int i = 0; i < 4; i++)
	{
		if(motorArray[i] == 0) //Brake to GND, write 0 to every pin
		{
			setGPIOWrite(45,0);
			setGPIOWrite(44,0);
			setGPIOWrite(43,0);
			setGPIOWrite(41,0);
			setGPIOWrite(37,0);
			setGPIOWrite(36,0);
			setGPIOWrite(35,0);
			setGPIOWrite(34,0);
		}
		if(i == 0) //Left front motor, INA = GPIO pin 44, INB = GPIO pin 43,
		{
			if(motorArray[i] > 0) //Turn CCW
			{
				setGPIOWrite(34,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(35,1); 
			}
			else if(motorArray[i] < 0) //Turn CW
			{
				setGPIOWrite(34,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(35,0);
			}
		}
		if(i == 1) //Left Rear motor, INA = GPIO pin 42, INB = GPIO pin 41
		{
			if(motorArray[i] > 0) //Turn CCW
			{
				setGPIOWrite(36,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(37,1);
			}
			else if(motorArray[i] < 0) //Turn CW
			{
				setGPIOWrite(36,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(37,0);
			}
		}
		if(i == 2) //Right rear motor, INA  = GPIO pin 40, INB = GPIO pin 39
		{
			if(motorArray[i] > 0) //Turn CW
			{
				setGPIOWrite(41,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(43,0);
			}
			else if(motorArray[i] < 0) //Turn CCW
			{
				setGPIOWrite(41,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(43,1);
			}
		}
		if(i == 3) //Right front motor, INA = GPIO pin 38, INB = GPIO pin 37
		{
			if(motorArray[i] > 0) //Turn CW
			{
				setGPIOWrite(44,1); //Corresponds to INA,INB = 1,0
				setGPIOWrite(45,0);
			}
			else if(motorArray[i] < 0) //Turn CCW
			{
				setGPIOWrite(44,0); //Corresponds to INA,INB = 0,1
				setGPIOWrite(45,1);
			}
		}
			
	}	
}

//Controller logic and math will go here once it is implemented. This is just for motor_controllering
void controlFunction() 
{
	float J = 3.456*(10^(-6)); //kg*m^2
	float L = 0.00054; // H
	float R = 0.8; // Ohms
	float K = 0.034; //    V/(rad/s)
	float changeI = 0.3;  //     Amps/s
	float maxMotorSpeed = (24 - L*changeI - R)/(K); //Max control output value using the highest voltage of 24 V
	
	for(int i = 0; i < 4; i++)
	{
		controllerArray[i] = (abs(motorArray[i]) - L*changeI - R)/K; //Scales input to be an equivalent voltage input to motor controller
	}

		
	SSController leftFrontWheel; //create controller object for left front wheel
	SSController leftRearWheel; //create controller object for left rear wheel
	SSController rightRearWheel; //create controller object for right rear wheel
	SSController rightFrontWheel; //create controller object for right front wheel
	
	leftFrontWheel.setU(controllerArray[0]); //Set input for left front wheel motor controller
	leftRearWheel.setU(controllerArray[1]); //set input for left rear wheel motor controller
	rightRearWheel.setU(controllerArray[2]); //set input for right rear wheel motor controller
	rightFrontWheel.setU(controllerArray[3]); //set input for right ront wheel motor controller
	
	int iterations = 0; //Number of iterations to update control outputs
	
	while(iterations < 100) //Updates 100 times before accepting new values to update on and then sends out updated values on controlOutput
	{
		leftFrontWheel.update();
		leftRearWheel.update();
		rightRearWheel.update();
		rightFrontWheel.update();
		
		controlOutput[0] = 100 - (leftFrontWheel.getY()[0][0])/(maxMotorSpeed)*100; //Does some scaling on the control output values to get PWM value for AdaFruit downstream
		controlOutput[1] = 100 - (leftRearWheel.getY()[0][0])/(maxMotorSpeed)*100;
		controlOutput[2] = 100 - (rightRearWheel.getY()[0][0])/(maxMotorSpeed)*100;
		controlOutput[3] = 100 - (rightFrontWheel.getY()[0][0])/(maxMotorSpeed)*100;

		iterations++; //Increments iterations variable
	}
	
	

}

//Creates a message to be sent to the I2C node.
motor_controller::AdaCmd generateMessage()
{

	motor_controller::AdaCmd msg;

	msg.device = motor_controller::AdaCmd::wheelMotors;
	
	for(int i = 0; i < 4; i++)
	{
	    msg.value.push_back(controlOutput[i]);
	}
	return msg;

}

//Callback function which fills motorArray with values from the message
void callBack (const motor_controller::WheelMotor& msg)
{
	
	motorArray[0] = msg.LF_motorVal;
	motorArray[1] = msg.LR_motorVal;
	motorArray[2] = msg.RR_motorVal;
	motorArray[3] = msg.RF_motorVal;
		
	setMotorDirection(); //Sets direction of motor using GPIO pins
		
	controlFunction();   //motor_controller function

	pub.publish(generateMessage());

}


//MAIN
int main(int argc, char** argv)
{

	ros::init(argc, argv, "wheel_motor_node"); //Initialize node

	ros::NodeHandle n; //Create nodehandle object

	sub = n.subscribe("wheel_motor_rc", 1000, callBack); //Create object to subscribe to topic "wheel_motor_rc"
	
	pub = n.advertise<motor_controller::AdaCmd>("adaFruit",1000); //Create object to publish to topic "I2C"
	
	ros::Rate loop_rate(10); //Set frequency of looping. 10 Hz
	
	setGPIOWrite(33,1); //Motor enable

	ros::spin();
	
	resetGPIO();
	
	return 0;
}

