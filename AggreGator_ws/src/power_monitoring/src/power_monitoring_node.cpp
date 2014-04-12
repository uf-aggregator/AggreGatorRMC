/*
 * Power monitoring node
 * Takes data from the ADC and converts voltage and current mesurments
 *      into power and keeps track of total power used
 */

#include "ros/ros.h"

#include "hardware_interface/RawMotorPowerData.h"
#include "hardware_interface/ElectronicPowerData.h"

//Global variables
ros::Subscriber raw_motor_power_sub;
ros::Subscriber electronic_power_sub;

//Converts raw data from ADC into power
void ConvertRawMotorToPower(const hardware_interface::RawMotorPowerData& data)
{
    ROS_INFO("Raw data [Voltage: %i, Current: %i]", data.voltage, data.current);
}

//Tracks the electronic power battery
void TrackElectronicPower(const hardware_interface::ElectronicPowerData& data)
{

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
    raw_motor_power_sub = n.subscribe("raw_motor_power", 1000, ConvertRawMotorToPower);
    electronic_power_sub = n.subscribe("electronic_power", 1000, TrackElectronicPower);

    /*
     * Main loop
     */
    while (ros::ok())
    {
        ros::spinOnce();
    }
}
