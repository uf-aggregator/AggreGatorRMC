#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char** argv)
{
    //Initilize the remote control node
    ros::init(argc, argv, "remote_control_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    /*
     * Advertise on the remote_control topic
     * returns a publisher
     * Parm1 = ???
     * Parm2 = num of messages to buffer
     */
    ros::Publisher remote_control_pub = n.advertise<std_msgs::String>("remote_control", 1000);

    //Set the send in Hz
    ros::Rate loop_rate(10);

    // debug stuff
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "Hello AggreGator " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());


        remote_control_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
}
