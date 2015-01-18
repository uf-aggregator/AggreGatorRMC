#include "ros/ros.h"
#include "std_msgs/builtin_int16.h"
#include "hardware_interface/GPIO.h"
#include "motor_controller/AdaCmd.h"
#include "motor_controller/controller.h"

#define epsilon 0.001

ros::Subscriber sub;
ros::Publisher pub;

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(0.01);       //time in seconds between sends

SSController bucketDrumMotor; //create controller object for the linear actuator

int last_dir = 0;

enum
{
	BD_A = 38,
	BD_B = 39
};

//Function that controls the direction of the motors using the GPIO pins on the O-Droid
void setMotorDirection()
{

	//float mc_out = linearAct.getY()[0][0];			//motorController_out
	float mc_out = bucketDrumMotor.getY()[0][0];
	int dir = 0;

	if (mc_out > epsilon)		//Dig
	{
		dir = 1;
	}
	else if (mc_out < -epsilon) //Dump
	{
		dir = -1;
	}
	else //Stop
	{
		dir = 0;
	}

	if (dir != last_dir)
	{
		switch (dir)
		{
		case 1:
			setGPIOWrite(BD_A, 1);
			setGPIOWrite(BD_B, 0);
			break;
		case -1:
			setGPIOWrite(BD_A, 0);
			setGPIOWrite(BD_B, 1);
			break;
		default:
			setGPIOWrite(BD_A, 0);
			setGPIOWrite(BD_B, 0);
		}
	}

	last_dir = dir;
	
}

void controlFunction() //WARNING: CONTROL USED BELOW WERE DESIGNED FOR WHEEL MOTORS, THIS IS JUST HERE FOR PLACE HOLDER UNTIL BUCKET MOTOR CONTROLLER IS DESIGNED
{
	//bucket motor controller goes here!
	
	bucketDrumMotor.update();
	
	//Sets direction of motor using GPIO pins
	setMotorDirection();
}

motor_controller::AdaCmd generateMessage()
{
    motor_controller::AdaCmd msg;

    msg.device = motor_controller::AdaCmd::bucketDrum;

	msg.value.push_back(abs(bucketDrumMotor.getY()[0][0] / 24 * 100));

    return msg;
}

void callBack(const std_msgs::Int16& msg)
{
	//Scales input to be an equivalent voltage input to motor controller
	bucketDrumMotor.setU(msg.data * 24.0 / 32768.0);
}

//Stop the bucket motor
void stopBucketMotor()
{
	setGPIOWrite(BD_A, 0);
	setGPIOWrite(BD_B, 0);
}
//MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "bucket_motor_node");                              //Initialize node
    ros::NodeHandle n;                                                       //Create nodehandle object

    sub = n.subscribe("bucket_motor_rc", 1000, callBack);                 //Create object to subscribe to topic "linear_actuator_rc"
    pub = n.advertise<motor_controller::AdaCmd>("adaFruit",1000);         //Create object to publish to topic "I2C"

     while(ros::ok())
	{
		//Update time
		current_time = ros::Time::now();
		//Check if interval has passed
		if(current_time - last_time > update_rate)
		{
			//Reset time
			last_time = current_time;
			if (sub.getNumPublishers() == 0) //In case of loss of connection to publisher, set controller inputs to 0
			{
				//Stop Everything!!!!
				ROS_WARN("Loss of bucket motor input!");
				bucketDrumMotor.setU(0);
			}

			//motor_controller function
			controlFunction();
			pub.publish(generateMessage());
		}
		ros::spinOnce();
	}

    stopBucketMotor();

    return 0;
}
