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

ros::Time last_time, current_time;
const ros::Duration send_time(0.1);       //time in seconds between sends

ros::Publisher motor_pub;
ros::Publisher linear_actuator_pub;
ros::Publisher bucket_motor_pub;


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

        //Format data
        remote_control::motorMSG msg;
        int left = left_motors * 32767;       //Scale to signed 16 bit int
        int right = right_motors * 32767;     //Scale to signed 16 bit int
        msg.LF_motorVal = left;
        msg.LR_motorVal = left;
        msg.RR_motorVal = right;
        msg.RF_motorVal = right;

        //Send msg
        motor_pub.publish(msg);
        //ROS_INFO("Published a motor msg L:%i_R:%i", msg.LF_motorVal, msg.RR_motorVal);
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
void AvgMotorInput(float left, float right)
{
    if(running_avg)
    {
        left_motors = ((running_avg - 1) * left_motors + left) / running_avg;
        right_motors = ((running_avg - 1) * right_motors + right) / running_avg;
    }
    else
    {
        left_motors = left;
        right_motors = right;
    }
    //Increment running_avg
    running_avg++;
}

void XboxCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    AvgMotorInput(joy->axes[UD_LEFT], joy->axes[UD_RIGHT]);

    //ROS_INFO("0:%f_1:%f_2:%f_3:%f_4:%f_5:%f_6:%f_7:%f_8:%f", joy->axes[0], joy->axes[1], joy->axes[2], joy->axes[3], joy->axes[4], joy->axes[5], joy->axes[6], joy->axes[7]);
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

    //Set the send in Hz
    ros::Rate loop_rate(10);

    //Set up subscriber, listens to joy topic, buffer only 10 messages, us XboxCallback
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, XboxCallback);


    //Set up publisher on motor_rc, buffer up to 10 msgs
    motor_pub = n.advertise<remote_control::motorMSG>("motor_rc", 10);

    //Set up publisher on linear_actuator_rc
    linear_actuator_pub = n.advertise<std_msgs::Int16>("linear_actuator_rc", 10);

    //Set up publisher on bucket_motor_rc
    bucket_motor_pub = n.advertise<std_msgs::Int16>("bucket_motor_rc", 10);

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
