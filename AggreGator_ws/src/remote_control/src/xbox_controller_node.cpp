#include <cstdlib>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "remote_control/Motor.h"
#include <std_msgs/builtin_int16.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <string.h>
#include <queue>


/*
* Forward declarations
*/
void StopEverything();

//this struct represents the Motor.msg file, which is sent on the /motor_rc topic to the motor_node
struct MotorStruct{
	float leftFront; //left front wheel
	float rightFront; //right front wheel
	float rightRear; //right rear wheel
	float leftRear; //left rear wheel
	//MORE MOTORS CAN BE ADDED HERE
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
 * Global Variables for xbox_controller
 * NOTE: variables for bucket_motor and linear_actuator have been removed; use them if more motors are added
 */
double left_motors(0.0), right_motors(0.0);
bool enable = false; //for all motors, not just wheels
double wheel_gear = 0.7f;				//Multiplier to simulate wheel_gears (Default to 70%)
bool simulation_delay = false; //if true, uses a queue to simulate NASA lag

//double bucket_motor(1.0), linear_actuator(1.0);		//One is stop due to controller input
//short int bucket_motor_dir(1), linear_actuator_dir(1);
//bool mining_motion_enable = false;			//If this is false bucket and actuators will be published 0
//double bucket_gear = 0.5f;


//Create a queue for the motor publisher
unsigned int delay_queue_size = 0;		//Size of queue to get desired delay
const double delay_time = 2;			//delay x seconds
std::queue<MotorStruct> motor_queue;

//Keep track of button presses (to find button releases)
bool btn_pressed[NUM_BTNS] = { false };  //TODO: Eventually convert these to hashes
bool btn_released[NUM_BTNS] = { false };

//Timing variables
ros::Time last_time, current_time;
ros::Duration send_time(0.1);       //time in seconds between sends (send ten messages a second)

//Ros publishers and subscribers
//Combine all publishers into one
	//ros::Publisher wheel_motor_pub; --REMOVED
	//ros::Publisher linear_actuator_pub; --REMOVED
	//ros::Publisher bucket_motor_pub; --REMOVED
ros::Publisher motor_pub;


/*
 * Int for mapping
 * 1 is linear, 2 is quadratic, 3 cubic (anything else defaults to linear)
 */
int32_t mapping; //TODO: figure out what this actually does


/*
 * Write the motor msg out
 * Rights the current command out
 * IN: none (uses global variables "left_motors" and "right_motors")
 * Out: WheelMotor to wheel_motor_rc topic
 */
void WriteMotorValue()
{ 
    	double left = 0;
    	double right = 0;
    	current_time = ros::Time::now();
    	if(current_time - last_time > send_time){
	    	//Update time
		last_time = current_time;
		//reset running avg
		//running_avg = 0;
		
		if(enable){ //if overall motor enable is true
			//Format data
			remote_control::Motor motor_msg;
			if(enable){
				/* NOTE: Before, these values were scaled to a 16 bit integer
				The motor_controller node now has that responsibility
				This node will send the motor values based on what it receives from joy_node * wheel_gear
					Bounds: [-1,1] * wheel_gear = [-wheel_gear, wheel_gear]
				-Joey
				*/
				left = left_motors  * wheel_gear;       //Scale to gear
				right = right_motors * wheel_gear;     //Scale to gear
			}else{
				left = 0.0;
				right = 0.0;
			}
		//Format actuator and bucket drum data
		/*0
		TODO: Figure out if we still need this stuff (leaving it for now) - Joey
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

			//Write to all motors at the same time
			//Create a message with all motor values
			motor_msg.leftFront_motorVal = left;
			motor_msg.leftRear_motorVal = left;
			motor_msg.rightRear_motorVal = right;
			motor_msg.rightFront_motorVal = right;

			if(simulation_delay)
			{
				//if there is a simulation delay, use left and right to create a simulated message
				//push this message to the motor_queue
				//and then publish the message at the front of the queue (if the queue is at delay_queue_size)
				//this simulates a "lag" that we will have to deal with at NASA
				MotorStruct motor_temp = {left, right, right, left};
				motor_queue.push(motor_temp);
				if(motor_queue.size() >= delay_queue_size && (motor_queue.size() > 0))
				{
					
					MotorStruct motor_temp = motor_queue.front();
					motor_queue.pop();				
					motor_msg.leftFront_motorVal = motor_temp.leftFront;
					motor_msg.leftRear_motorVal = motor_temp.leftRear;
					motor_msg.rightRear_motorVal = motor_temp.rightRear;
					motor_msg.rightFront_motorVal = motor_temp.rightFront;
					motor_pub.publish(motor_msg);
				}
			}
			else
			{
		
				//Send msg right away
				motor_pub.publish(motor_msg);
			}
			
		}

	
     }
    
}

/* XboxCallback - executed each time a message is recieved from /joy topic
	INPUT: joy message (list of axis values as floats with bounds [-1, 1] 
		               and buttons as ints with 0 (not pressed) or 1 (pressed)
	OUTPUT: None, but global variables left_motor and right_motor are set
			left_motor and right_motor are used in WriteMotorValues()
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

/*
	//Linear Actuator toggle TODO: Change this to another mining motor later
	if (btn_released[RB])
	{
		linear_actuator_dir *= -1;
		if(linear_actuator_dir)
			ROS_INFO("Linear actuator set to extend");
		else
			ROS_INFO("Linear actuator set to retract");
	}
	//Bucket Drum toggle TODO: Change this to another mining motor later
	if (btn_released[LB])
	{
		bucket_motor_dir *= -1;
		if(bucket_motor_dir)
			ROS_INFO("Bucket drum set to dump");
		else
			ROS_INFO("Bucket drum set to mine");
	}
*/
    
	//OVERALL motor enable toggle
	if (btn_released[START])
	{
		enable = !enable;
		//If we disable input imediatly send a stop msg
		if (!enable)
		{
//			
			StopEverything();
			ROS_INFO("Motors disabled");
		}
		else
		{
			ROS_INFO("Motors enabled");
		}
	}

	//Mining system enable toggle TODO: Change this to another mining motor later
	/*
	if(btn_released[BACK])
	{
		mining_motion_enable = !mining_motion_enable;
		//Stop imediatly if stop
		if(!mining_motion_enable)
		{
//			StopMining();
			ROS_INFO("Mining system disabled");
		}
		else
		{
			ROS_INFO("Mining system enabled");
		}
	}
	*/
	
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

	//Bucket gear up TODO: Change this to another mining motor later
	/*
	if(btn_released[Y])
	{
		bucket_gear += 0.1f;
		if (bucket_gear >= 1.0f)
		{
			bucket_gear = 1.0f;
			ROS_INFO("Mining speed not limited (100%% of max)");
		}
		else
			ROS_INFO("Mining speed limited to %.2f%% of max", bucket_gear * 100.0f);
	}
	*/
	
	//bucket_gear down TODO: Change this to another mining motor later
	/*
	if (btn_released[X])
	{
		bucket_gear -= 0.1f;
		if (bucket_gear <= 0.1f)
		{
			bucket_gear = 0.1f;
			ROS_INFO("Mining speed limited to minimum 10%% of max");
		}
		else
			ROS_INFO("Mining speed limited to %.2f%% of max", bucket_gear * 100.0f);
	}
    */
    
    //Set motors to current value
    left_motors = joy->axes[UD_LEFT];
    right_motors = joy->axes[UD_RIGHT];

    
//    bucket_motor = joy->axes[LT];
//    linear_actuator = joy->axes[RT];

//    if(btn_released[BACK])
//   	ROS_INFO("Triger values B:L \t %.2f : %.2f", bucket_motor, linear_actuator);
}

//StopEverything
//Publishes all zero messages to all motors
void StopEverything()
{
	//Generate msgs
    //Format data
    remote_control::Motor motor_msg;

    //Fill msgs with 0s
    motor_msg.leftFront_motorVal = 0;
    motor_msg.leftRear_motorVal = 0;
    motor_msg.rightRear_motorVal = 0;
    motor_msg.rightFront_motorVal = 0;
	
	//publish the message
    motor_pub.publish(motor_msg);
    
    //StopEverything will be used for emergencies, so it will bypass the "simulated delay" if it's enabled
    //thus we will restart the delay by emptying the queue as well
    while(motor_queue.size() > 0){
		motor_queue.pop();
    }


//    bucket_motor = 1.0;
//    linear_actuator = 1.0;
}

int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the xbox_controller_node 
    //(renamed from remote_control_node for clarity; it should match the filename -Joey)
    ros::init(argc, argv, "xbox_controller_node");

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

    //bucket_gear parameter TODO: Reuse for another motor
/*
    n.param<double>("/remote_control/bucket_gear", bucket_gear, .12f);
    if(bucket_gear > 1.0f || bucket_gear < 0.0f)
    {
	bucket_gear = 1.0f;
	ROS_ERROR("You set the bucket gear to %.2f%%, which is outside of bounds.  Setting to 100%%", bucket_gear * 100);
    }
    else if(bucket_gear < 0.02f)
    {
	bucket_gear = 0.02f;
	ROS_ERROR("You set the bucket gear to %.2f%%, which is outside of bounds.  Setting to 2%%", bucket_gear * 100);
    }
    else
    {
	ROS_INFO("Bucket gear = %.2f%%", bucket_gear);
    }
*/
    //parameter for simulating delay
    n.param<bool>("/remote_control/sim", simulation_delay, false);

    ROS_INFO("Xbox controller set to order %i mapping", mapping);

    //Set up subscriber, listens to joy topic, buffer only 10 messages, use XboxCallback
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, XboxCallback);


    //Set up publisher on motor_rc, buffer up to 1000 msgs
    motor_pub = n.advertise<remote_control::Motor>("motor_rc", 1000);

    //Initilize time
    last_time = ros::Time::now();
    current_time = last_time;

    //Wait for publishers to inti then turn everything off
    //while((bucket_motor_pub.getNumSubscribers() == 0 || linear_actuator_pub.getNumSubscribers() == 0 || wheel_motor_pub.getNumSubscribers() == 0) && ros::ok());
    
    //To make sure the motors start at 0
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