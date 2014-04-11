/*
 * Power monitoring node
 * Takes data from the ADC and converts voltage and current mesurments
 *      into power and keeps track of total power used
 */

#include "ros/ros.h"

#include "hardware_interface/RawPowerData.h"

//Global variables
ros::Subscriber raw_power_sub;

//Converts raw data from ADC into power
void ConvertToPower(const hardware_interface::RawPowerData& data)
{
    ROS_INFO("Raw data [Voltage: %i, Current: %i]", data.voltage, data.current);
}

/*
 * Main loop
 */
int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the adc node
    ros::init(argc, argv, "power_monitoring_node");

    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize publishers, subscribers, and services
    raw_power_sub = n.subscribe("raw_power", 1000, ConvertToPower);

    /*
     * Main loop
     */
    while (ros::ok())
    {
        ros::spinOnce();
    }
}
