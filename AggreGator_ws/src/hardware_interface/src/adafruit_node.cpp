#include "ros/ros.h"
#include "AdaFruit.h"
int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the adafruit node
    ros::init(argc, argv, "adafruit_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    /*
     * Main loop
     */
    while (ros::ok())
    {
        ros::spinOnce();
    }
}
