#include "ros/ros.h"

#include "hardware_interface/WriteI2CRegister.h"
#include "hardware_interface/ReadI2CRegister.h"

void PublishElectronicPower()
{

}

int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the ina226 node
    ros::init(argc, argv, "ina226_node");

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
