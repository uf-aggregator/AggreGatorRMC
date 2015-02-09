#include "ros/ros.h"
#include "std_msgs/builtin_int16.h"
#include "hardware_interface/GPIO.h"
#include "common_msgs/LinActMotor.h"
#include "common_msgs/AdaCmd.h"
#include "controller.h"

#define epsilon 0.001

ros::Subscriber sub;
ros::Publisher pub;

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(0.01);       //time in seconds between sends

//create controller object for the linear actuator
SSController linearActuator;

int last_dir = 0;
	
enum
{
	LA_A = 40,
	LA_B = 42
};

void setMotorDirection(); //forward declaration

void controlFunction() //WARNING: CONTROL USED BELOW WERE DESIGNED FOR WHEEL MOTORS, THIS IS JUST HERE FOR PLACE HOLDER UNTIL LINEAR ACTUATOR DESIGN IS DEVELOPED
{
	linearActuator.update();

	//Sets direction of motor using GPIO pins
	setMotorDirection();
}

//Function that controls the direction of the motors using the GPIO pins on the O-Droid
void setMotorDirection()
{
	float mc_out = linearActuator.getY()[0][0];			//motorController_out
	int dir = 0;

	if (mc_out > epsilon)		 //Linear Actuator moves up
	{
		dir = 1;
	}
	else if (mc_out < -epsilon) //linear actuator moves down
	{
		dir = -1;
	}
	else						//Linear actuator stops
	{
		dir = 0;
	}

	if (dir != last_dir)
	{
		switch (dir)
		{
		case 1:
			setGPIOWrite(LA_A, 1);
			setGPIOWrite(LA_B, 0);
			break;
		case -1:
			setGPIOWrite(LA_A, 0);
			setGPIOWrite(LA_B, 1);
			break;
		default:
			setGPIOWrite(LA_A, 0);
			setGPIOWrite(LA_B, 0);
		}
	}

	last_dir = dir;
	
}


common_msgs::AdaCmd generateMessage()
{
    common_msgs::AdaCmd msg;
    
    msg.device = common_msgs::AdaCmd::linearActuator;

	//Scale and push_back msg
	msg.value.push_back(abs(linearActuator.getY()[0][0] / 24 * 100));

    return msg;
}


void LinActCallback(const common_msgs::LinActMotor& msg)
{
	//Scales input to be an equivalent voltage input to motor controller
	linearActuator.setU(msg.mining_motorVal * 24.0 / 32768.0); 
}

void stopLinearActuator()
{
	setGPIOWrite(LA_A, 0);
	setGPIOWrite(LA_B, 0);
}

//MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "linear_actuator_node");                     //Initialize node
    ros::NodeHandle n;                                                 //Create nodehandle object

    sub = n.subscribe("linear_actuator_rc", 1000, LinActCallback);           //Create object to subscribe to topic "linear_actuator_rc"
    pub = n.advertise<common_msgs::AdaCmd>("adaFruit",1000);      //Create object to publish to topic "adaFruit"
	
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
				ROS_WARN("Loss of Linear Actuator input!");
				linearActuator.setU(0);
			}

			//motor_controller function
			controlFunction();
			pub.publish(generateMessage());

		}
		ros::spinOnce();
	}

    stopLinearActuator();

    return 0;
}
