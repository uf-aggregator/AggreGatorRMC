/*
 * Power monitoring node
 * Takes data from the ADC and converts voltage and current mesurments
 *      into power and keeps track of total power used
 */

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "hardware_interface/RawMotorPowerData.h"
#include "hardware_interface/ElectronicPowerData.h"

using namespace std;

//Global variables
ros::Subscriber raw_motor_power_sub;
ros::Subscriber electronic_power_sub;

vector<float> powerReadings; //Vector to store power readings

ros::Time last_time(0), current_time; //Timing variables
ros::Duration update_rate(300); //time in seconds between power monitoring updates

//Converts raw data from ADC into power
void ConvertRawMotorToPower(const hardware_interface::RawMotorPowerData& data)
{
    ROS_INFO("Raw data [Voltage: %i, Current: %i]", data.voltage, data.current);
}

//Tracks the power used by the electronics
void TrackElectronicPower(const hardware_interface::ElectronicPowerData& data)
{
	float calculatedPower = data.powerLSB*data.power; //Calculates power based on the digital value from the INA226 and the bit-to-Watt conversion ratio calculated in the ina226 node
	
	powerReadings.push_back(calculatedPower); //Add reading to vector
	
	
}

//Adds together all power readings to get total power usage, resets the powerReadings vector with the updated value and saves the total power reading to a file for logging purposes
void UpdateElectronicPowerUsage()
{
	for(int i = 1; i < powerReadings.size(); i++)
		powerReadings[0] += powerReadings[i];
		
	powerReadings.resize(1); //resizes vector with new sum value
	
	ofstream powerFile;
	powerFile.open("PowerUsageLog.txt", ios::out | ios::app);
	if(powerFile.is_open())
	{
		powerFile << current_time << "\t";
		powerFile << powerReadings[0] << "W\n";
		powerFile.close();
	}
	else
		ROS_ERROR("Power log can't be opened.");
	
	
}
int main(int argc, char** argv)
{
    /*
     * Initilization
     */

    //Initilize the adc node
    ros::init(argc, argv, "power_monitoring_node");
	
	//Setup file for power logging
	ofstream powerFile;
	powerFile.open("PowerUsageLog.txt", ios::out | ios::trunc);
	if(powerFile.is_open())
	{
		powerFile << "UF NASA Robotic Mining Team, Power Logged during a Competition Run. \n";
		powerFile.close();
	}
	else
		ROS_ERROR("Cannot open file for power logging.!");
		
		
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
	current_time = ros::Time::now(); //updated time
		
	if(current_time - last_time > update_rate)
	{
		last_time = current_time;
		UpdateElectronicPowerUsage();
	}
        ros::spinOnce();
    }
	
	return 0;
}
