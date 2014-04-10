#include "ros/ros.h"
#include "std_msgs/builtin_int16.h"
#include "motor_controller/AdaCmd.h"
#include "hardware_interface/GPIO.h"
#include "controller.h"

ros::Subscriber sub;
ros::Publisher pub;
int motorInput;


//Function that controls the direction of the motors using the GPIO pins on the O-Droid
void setMotorDirection()
{
	if(motorInput == 0) //Brake to GND, write 0 to every pin
	{
		setGPIOWrite(38,0);
		setGPIOWrite(39,0);
	}
	else if(motorInput > 0) //INA = GPIO pin 38, INB = GPIO pin 39, bucket drum digs
	{
		setGPIOWrite(38,1); 
		setGPIOWrite(39,0);
	}
	else //Bucket drum dumps
	{
		setGPIOWrite(38,0);
		setGPIOWrite(39,1);
	}
	
}

float controlFunction() //WARNING: CONTROL USED BELOW WERE DESIGNED FOR WHEEL MOTORS, THIS IS JUST HERE FOR PLACE HOLDER UNTIL LINEAR 				ACTUATOR DESIGN IS DEVELOPED
{
	//Linear actuator controller goes here!!!!
	float J = 3.456*(10^(-6)); //kg*m^2
	float L = 0.00054; // H
	float R = 0.8; // Ohms
	float K = 0.034; //    V/(rad/s)
	float changeI = 0.3;  //     Amps/s
	float maxMotorSpeed = (24 - L*changeI - R)/(K); //Max control output value using the highest voltage of 24 V
	
	
	float controlInput = (abs(motorInput) - L*changeI - R)/K; //Scales input to be an equivalent voltage input to motor controller

		
	SSController bucketDrumMotor; //create controller object for the linear actuator
	
	bucketDrumMotor.setU(controlInput);
	
	int iterations = 0; //Number of iterations to update control outputs
	
	float controlOutput;
	while(iterations < 100) //Updates 100 times before accepting new values to update on and then sends out updated values on 					controlOutput
	{
		bucketDrumMotor.update();
		
	  	controlOutput = 100 - (bucketDrumMotor.getY()[0][0])/(maxMotorSpeed)*100; //Does some scaling on the control output values to get PWM value for AdaFruit downstream


		iterations++; //Increments iterations variable
	}
	return controlOutput;
}

motor_controller::AdaCmd generateMessage()
{
    motor_controller::AdaCmd msg;

    msg.device = motor_controller::AdaCmd::bucketDrum;

    msg.value.push_back(controlFunction());

    return msg;
}

void callBack(const std_msgs::Int16& msg)
{
    setMotorDirection();

    pub.publish(generateMessage());   //Generate and publish an I2C msg
}

//MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bucket_motor_node");                              //Initialize node
    ros::NodeHandle n;                                                       //Create nodehandle object

    sub = n.subscribe("bucket_motor_rc", 1000, callBack);                 //Create object to subscribe to topic "linear_actuator_rc"
    pub = n.advertise<motor_controller::AdaCmd>("adaFruit",1000);         //Create object to publish to topic "I2C"

    ros::Rate loop_rate(10); //Set frequency of looping. 10 Hz

    //Loop as long as ros system is ok
    ros::spin();

    resetGPIO();

    return 0;
}
