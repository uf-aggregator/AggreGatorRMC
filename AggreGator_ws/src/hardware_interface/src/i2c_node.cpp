#include "ros/ros.h"

#include <i2c.h>


int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the i2c node
    ros::init(argc, argv, "i2c_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize the I2C bus
    init_i2c();

    /*
     * Main loop
     */
    while (ros::ok())
    {

        ros::spinOnce();
    }
}
