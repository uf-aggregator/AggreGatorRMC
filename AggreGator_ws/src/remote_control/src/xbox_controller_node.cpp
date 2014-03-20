#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "remote_control/motorMSG.h"
#include <std_msgs/builtin_int16.h>
#include "ros/time.h"
#include "ros/duration.h"

/*
 * Variables
 */
int running_avg = 0;
double left_motors(0.0), right_motors(0.0);
double bucket_motor(0.0), linear_actuator(0.0);
unsigned short int bucket_motor_dir(1), linear_actuator_dir(1);

//Timing variables
ros::Time last_time, current_time;
ros::Duration send_time(0.1);       //time in seconds between sends

//Ros publishers and subscribers
ros::Publisher wheel_motor_pub;
ros::Publisher linear_actuator_pub;
ros::Publisher bucket_motor_pub;

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
    RIGHT_STICK
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
 * Write the motor msg out
 * Rights the current command out
 * IN: none
 * Out: motorMSG to motor_rc topic
 */
void WriteMotorValue()
{
    //Update time
    current_time = ros::Time::now();
    if(current_time - last_time > send_time)
    {
        //reset time
        last_time = current_time;
        //reset running avg
        running_avg = 0;

        //Format wheel motor data
        remote_control::motorMSG wheel_msg;
        int left = left_motors * 32767;       //Scale to signed 16 bit int
        int right = right_motors * 32767;     //Scale to signed 16 bit int
        wheel_msg.LF_motorVal = left;
        wheel_msg.LR_motorVal = left;
        wheel_msg.RR_motorVal = right;
        wheel_msg.RF_motorVal = right;

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
         *          value += 1
         *          value /= 2
         *          value *= 32767
         *
         *      Make positive or negative based on booleans:
         *          value *= direction
         *          direction = 1 or -1 based on LB and RB
         */
        int actuator = (linear_actuator + 1) * 16384 * linear_actuator_dir;
        int bucket = (bucket_motor + 1) * 16384 * bucket_motor_dir;

        std_msgs::Int16 actuator_msg, bucket_msg;
        actuator_msg.data = actuator;               //Float -> int
        bucket_msg.data = bucket;                   //Float -> int

        //Send msg
        wheel_motor_pub.publish(wheel_msg);
        linear_actuator_pub.publish(actuator_msg);
        bucket_motor_pub.publish(bucket_msg);
    }
}

/*
 * Average motor input
 * Takes a runing average of the remote control input
 * ((count - 1) * last_value + current_value) / count
 * the count must be kept externaly
 * IN: left motor setpoint, right motor setpoint
 * Out: Sets global variables to the average
 */
void AvgMotorInput(float left, float right, float bucket, float actuator)
{
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

void XboxCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //Check for button presses
    if(joy->buttons[LB])
        linear_actuator_dir *= -1;

    if(joy->buttons[RB])
        bucket_motor_dir *= -1;

    AvgMotorInput(joy->axes[UD_LEFT], joy->axes[UD_RIGHT], joy->axes[LT], joy->axes[RT]);
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
    double temp;
    n.param<double>("/remote_control_send_freq", temp, 0.1);
    send_time.fromSec(1 / temp);
    ROS_INFO("Setting send period to %f, %f", send_time.toSec(), temp);

    //Set up subscriber, listens to joy topic, buffer only 10 messages, us XboxCallback
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, XboxCallback);

    //Set up publisher on motor_rc, buffer up to 10 msgs
    wheel_motor_pub = n.advertise<remote_control::motorMSG>("wheel_motor_rc", 1000);

    //Set up publisher on linear_actuator_rc
    linear_actuator_pub = n.advertise<std_msgs::Int16>("linear_actuator_rc", 1000);

    //Set up publisher on bucket_motor_rc
    bucket_motor_pub = n.advertise<std_msgs::Int16>("bucket_motor_rc", 1000);

    //Initilize time
    last_time = ros::Time::now();
    current_time = last_time;


    /*
     * Main loop
     */

    while (ros::ok())
    {
        WriteMotorValue();
        ros::spinOnce();
    }
}
