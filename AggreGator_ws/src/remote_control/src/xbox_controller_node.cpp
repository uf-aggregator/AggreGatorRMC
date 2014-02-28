#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
//#include <std_msgs/String.h>
//#include <sstream>

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
    LR_RIGHT,
    UD_RIGHT,
    RT,
    LT,
    LR_DPAD,        //I'm not sure this is actually the DPAD
    UD_DPAD
};

void XboxCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    ROS_INFO("%f", joy->axes[LR_LEFT]);
}

int main(int argc, char** argv)
{
    //Initilize the remote control node
    ros::init(argc, argv, "remote_control_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Set the send in Hz
    ros::Rate loop_rate(10);

    //Set up subscriber
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, XboxCallback);

    while (ros::ok())
    {

        ros::spin();
    }
}
