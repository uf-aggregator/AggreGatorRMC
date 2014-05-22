#include <cstdlib>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "remote_control/WheelMotor.h"
#include <std_msgs/builtin_int16.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <string.h>
#include <queue>


/*
* Forward declarations
*/
void StopEverything();
void StopMining();
void StopWheels();

struct WheelMotorStruct{
	int LF;
	int RF;
	int RR;
	int LR;
};

//Buttons on the Xbox Controller
enum XboxButtons
{
	A,
	B,
	X,
	Y,
	LB,
	RB,
	BACK,
	START,
	POWER,
	LEFT_STICK,
	RIGHT_STICK,

	NUM_BTNS
};

/*
* LR = left right
* UD = up down
* Left = left stick
* Right = right stick
* RT = right trigger
* LT = left trigger
*/
enum XboxAxis
{
	LR_LEFT,
	UD_LEFT,
	LT,
	LR_RIGHT,
	UD_RIGHT,
	RT
};


/*
 * Variables
 */
//int running_avg = 0;
double left_motors(0.0), right_motors(0.0);
double bucket_motor(1.0), linear_actuator(1.0);		//One is stop due to controller input
short int bucket_motor_dir(1), linear_actuator_dir(1);
bool wheel_motion_enable = false;			//If this is zero msgs will be published to wheels
bool mining_motion_enable = false;			//If this is false bucket and actuators will be published 0
float wheel_gear = 0.7f;				//Multiplier to simulate wheel_gears (Default to 70%)
double bucket_gear = 0.5f;
bool simulation_delay = false;


//Create a queue for each publisher
unsigned int delay_queue_size = 0;		//Size of queue to get desired delay
const double delay_time = 2;			//delay x seconds
std::queue<WheelMotorStruct> wheel_queue;
std::queue<int> linear_queue;
std::queue<int> bucket_queue;

//Keep track of button presses (to find button releases)
bool btn_pressed[NUM_BTNS] = { false }; 
bool btn_released[NUM_BTNS] = { false };

//Timing variables
ros::Time last_time, current_time;
ros::Duration send_time(0.1);       //time in seconds between sends

//Ros publishers and subscribers
ros::Publisher wheel_motor_pub;
ros::Publisher linear_actuator_pub;
ros::Publisher bucket_motor_pub;

/*
 * Int for mapping
 * 1 is linear, 2 is quadratic, 3 cubic (anything else defaults to linear
 */
int32_t mapping;


/*
 * Write the motor msg out
 * Rights the current command out
 * IN: none
 * Out: WheelMotor to wheel_motor_rc topic
 */
void WriteMotorValue()
{ 
    current_time = ros::Time::now();
    if(current_time - last_time > send_time)
    {
	    //Update time
	last_time = current_time;
	//reset running avg
	//running_avg = 0;

	    if(wheel_motion_enable)
	    {
		//Format data
		remote_control::WheelMotor wheel_msg;

		int left = left_motors * 32767 * wheel_gear;       //Scale to signed 16 bit int
		int right = right_motors * 32767 * wheel_gear;     //Scale to signed 16 bit int
		wheel_msg.LF_motorVal = left;
		wheel_msg.LR_motorVal = left;
		wheel_msg.RR_motorVal = right;
		wheel_msg.RF_motorVal = right;

		if(simulation_delay)
		{
			WheelMotorStruct wheel_temp = {left, right, right, left};
			wheel_queue.push(wheel_temp);
			if(wheel_queue.size() >= delay_queue_size && (wheel_queue.size() > 0))
			{
				
				WheelMotorStruct wheel_temp = wheel_queue.front();
				wheel_queue.pop();				
				wheel_msg.LF_motorVal = wheel_temp.LF;
				wheel_msg.LR_motorVal = wheel_temp.LR;
				wheel_msg.RR_motorVal = wheel_temp.RR;
				wheel_msg.RF_motorVal = wheel_temp.RF;
				wheel_motor_pub.publish(wheel_msg);
			}
		}
		else
		{
	
		//Send msg
		wheel_motor_pub.publish(wheel_msg);
		}
		
	    }

	if(mining_motion_enable)
	{
		//Format actuator and bucket drum data
		/*
		 *      LT and RT are maped to:
		 *          not pressed = 1
		 *          half = 0
		 *          full pressed = -1
		 *      Transform to:
		 *          not pressed = 0
		 *          half = 16384
		 *          full pressed = 32767
		 *      Math:
		 *          value -= 1
		 *          value /= -2
		 *          value *= 32767
		 *
		 *          Subtract one from the scale factor so that we do not go over(-16383)
		 *
		 *      Make positive or negative based on booleans:
		 *          value *= direction
		 *          direction = 1 or -1 based on LB and RB
		 */
		int actuator = (linear_actuator - 1) * -16383 * linear_actuator_dir;
		float bucket = (bucket_motor - 1) * -16383 * bucket_motor_dir;

		if(!btn_pressed[X]){
			//bucket *= bucket_gear;
			bucket *= (wheel_gear);
		}
		//Generate msgs
		std_msgs::Int16 actuator_msg, bucket_msg;
		actuator_msg.data = actuator;
		bucket_msg.data = (int)bucket;                   //Float -> int

		if(simulation_delay)
		{
			linear_queue.push(actuator);
			if(linear_queue.size() >= delay_queue_size && (linear_queue.size() > 0))
			{
				
				int linear_temp = linear_queue.front();
				linear_queue.pop();				
				actuator_msg.data = linear_temp;  
				linear_actuator_pub.publish(actuator_msg);
			}

			bucket_queue.push((int)bucket);			
			if(bucket_queue.size() >= delay_queue_size && (bucket_queue.size() > 0))
			{
				
				int bucket_temp = bucket_queue.front();
				bucket_queue.pop();				
				bucket_msg.data = bucket_temp;  
				bucket_motor_pub.publish(bucket_msg);
			}
		}
		else
		{

			//Publish msg
			linear_actuator_pub.publish(actuator_msg);
			bucket_motor_pub.publish(bucket_msg);
		}
	}

	
     }
    
}

/*
 * Average motor input
 * Takes a runing average of the remote control input
 * ((count - 1) * last_value + current_value) / count
 * the count must be kept externaly
 * IN: left motor setpoint, right motor setpoint
 * Out: Sets global variables to the average
 *
void AvgMotorInput(float left, float right, float bucket, float actuator)
{
    /*
     * Set up mapping (Quite easy since values are already scaled to -1 to 1)
     * linear
     *      No change required
     * Quadratic:
     *      value *= value
     *      for negative orignal values * -1
     * Cubic:
     *      value = value^3
     *
    if(mapping % 2)
    {
        left        = pow(left, mapping);
        right       = pow(right, mapping);
    }
    else
    {
        left        = pow(left, mapping)     * ((left >= 0) ? 1 : -1);
        right       = pow(right, mapping)    * ((right >= 0) ? 1 : -1);
    }

    /*
     *Take a running Avg
     * NewAvg = ((count - 1) * lastAvg + newValue) / count
     *

    if(running_avg)
    {
        left_motors = ((running_avg - 1) * left_motors + left) / running_avg;
        right_motors = ((running_avg - 1) * right_motors + right) / running_avg;
        bucket_motor = ((running_avg - 1) * bucket_motor + bucket) / running_avg;
        linear_actuator = ((running_avg - 1) * linear_actuator + actuator) / running_avg;
    }
    else //Set to first value after last send
    {
        left_motors = left;
        right_motors = right;
        bucket_motor = bucket;
        linear_actuator = actuator;
    }

    //Increment running_avg
    running_avg++;
}
*/

void XboxCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//Check for button releases
	for (int iii = 0; iii < NUM_BTNS; ++iii)
	{
		if (btn_pressed[iii] & !joy->buttons[iii])
			btn_released[iii] = true;
		else
			btn_released[iii] = false;

		btn_pressed[iii] = joy->buttons[iii];
	}

	//Linear Actuator toggle
	if (btn_released[RB])
	{
		linear_actuator_dir *= -1;
		if(linear_actuator_dir)
			ROS_INFO("Linear actuator set to extend");
		else
			ROS_INFO("Linear actuator set to retract");
	}

	//Bucket Drum toggle
	if (btn_released[LB])
	{
		bucket_motor_dir *= -1;
		if(bucket_motor_dir)
			ROS_INFO("Bucket drum set to dump");
		else
			ROS_INFO("Bucket drum set to mine");
	}
    
	//Wheel motor enable toggle
	if (btn_released[START])
	{
		wheel_motion_enable = !wheel_motion_enable;
		//If we disable input imediatly send a stop msg
		if (!wheel_motion_enable)
		{
			StopWheels();
			ROS_INFO("Drive motors disabled");
		}
		else
		{
			ROS_INFO("Drive motors enabled");
		}
	}

	//Mining system enable toggle
	if(btn_released[BACK])
	{
		mining_motion_enable = !mining_motion_enable;
		//Stop imediatly if stop
		if(!mining_motion_enable)
		{
			StopMining();
			ROS_INFO("Mining system disabled");
		}
		else
		{
			ROS_INFO("Mining system enabled");
		}
	}

	//wheel_gear up
	if (btn_released[B])
	{
		wheel_gear += 0.1f;
		if (wheel_gear >= 1.0f)
		{
			wheel_gear = 1.0f;
			ROS_INFO("Speed not limited (100%% of max)");
		}
		else
			ROS_INFO("Speed limited to %.2f%% of max", wheel_gear * 100.0f);
	}

	//wheel_gear down
	if (btn_released[A])
	{
		wheel_gear -= 0.1f;
		if (wheel_gear <= 0.1f)
		{
			wheel_gear = 0.1f;
			ROS_INFO("Speed limited to minimum 10%% of max");
		}
		else
			ROS_INFO("Speed limited to %.2f%% of max", wheel_gear * 100.0f);
	}
    //average the inputs
    //AvgMotorInput(joy->axes[UD_LEFT], joy->axes[UD_RIGHT], joy->axes[LT], joy->axes[RT]);

    //Set motors to current value
    left_motors = joy->axes[UD_LEFT];
    right_motors = joy->axes[UD_RIGHT];

    
    bucket_motor = joy->axes[LT];
    linear_actuator = joy->axes[RT];
}

//Stop wheels
void StopMining()
{
    std_msgs::Int16 actuator_msg, bucket_msg;
	
    actuator_msg.data = 0;
    bucket_msg.data = 0;

    //Send msg
    linear_actuator_pub.publish(actuator_msg);
    bucket_motor_pub.publish(bucket_msg);

    while(bucket_queue.size() > 0){
	bucket_queue.pop();
    }
    bucket_queue.push(0);
    bucket_queue.pop();

    while(linear_queue.size() > 0){
	linear_queue.pop();
    }
    bucket_queue.push(0);
    bucket_queue.pop();

    left_motors = 0;
    right_motors = 0;
}

void StopWheels()
{
    //Generate msgs
    //Format data
    remote_control::WheelMotor wheel_msg;

    //Fill msgs with 0s
    wheel_msg.LF_motorVal = 0;
    wheel_msg.LR_motorVal = 0;
    wheel_msg.RR_motorVal = 0;
    wheel_msg.RF_motorVal = 0;

    wheel_motor_pub.publish(wheel_msg);
    
    while(wheel_queue.size() > 0){
	wheel_queue.pop();
    }    
    WheelMotorStruct temp = {0,0,0,0};
    wheel_queue.push(temp);
    wheel_queue.pop();

    bucket_motor = 1.0;
    linear_actuator = 1.0;
}
//StopEverything
//Publishes all zero messages to all motors
void StopEverything()
{
    StopWheels();
    StopMining();
}

int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the remote control node
    ros::init(argc, argv, "remote_control_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Get the parameters
    //Send frequence
    double temp;
    n.param<double>("/remote_control/send_freq", temp, 0.1);
    send_time.fromSec(1 / temp);
    ROS_INFO("Setting send period to %f, (frequency: %f)", send_time.toSec(), temp);

    //Calculate message size 
    delay_queue_size = (int)(delay_time * temp);

    //mapping
    n.param<int32_t>("/remote_control/xbox_controller/mapping", mapping, 1);
    if(mapping < 1 || mapping > 3)
        mapping = 1;

    //bucket_gear parameter
    n.param<double>("/remote_control/bucket_gear", bucket_gear, .12f);
    if(bucket_gear > 1.0f || bucket_gear < 0.0f)
    {
	bucket_gear = 1.0f;
	ROS_ERROR("You set the bucket gear to %.2f%%, which is outside of bounds, you idiot.  Setting to 100%%", bucket_gear * 100);
    }
    else if(bucket_gear < 0.02f)
    {
	bucket_gear = 0.02f;
	ROS_ERROR("You set the bucket gear to %.2f%%, which is outside of bounds, you idiot.  Setting to 2%%", bucket_gear * 100);
    }
    else
    {
	ROS_INFO("Bucket gear = %.2f%%", bucket_gear);
    }

    //parameter for simulating delay
    n.param<bool>("/remote_control/sim", simulation_delay, false);

    ROS_INFO("Xbox controller set to order %i mapping", mapping);

    //Set up subscriber, listens to joy topic, buffer only 10 messages, us XboxCallback
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, XboxCallback);


    //Set up publisher on motor_rc, buffer up to 1000 msgs
    wheel_motor_pub = n.advertise<remote_control::WheelMotor>("wheel_motor_rc", 1000);

    //Set up publisher on linear_actuator_rc
    linear_actuator_pub = n.advertise<std_msgs::Int16>("linear_actuator_rc", 1000);

    //Set up publisher on bucket_motor_rc
    bucket_motor_pub = n.advertise<std_msgs::Int16>("bucket_motor_rc", 1000);

    //Initilize time
    last_time = ros::Time::now();
    current_time = last_time;

    //Wait for publishers to inti then turn everything off
    //while((bucket_motor_pub.getNumSubscribers() == 0 || linear_actuator_pub.getNumSubscribers() == 0 || wheel_motor_pub.getNumSubscribers() == 0) && ros::ok());
    StopEverything();


    /*
     * Main loop
     */
    while (ros::ok())
    {
        WriteMotorValue();
        ros::spinOnce();
    }

   return 0;
}
