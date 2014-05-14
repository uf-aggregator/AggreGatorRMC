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

ros::Time last_time(0), current_time, start_time; //Timing variables
ros::Duration update_rate(60); //time in seconds between power monitoring updates

//Converts raw data from ADC into power
void ConvertRawMotorToPower(const hardware_interface::RawMotorPowerData& data)
{
  //  ROS_INFO("Raw data [Voltage: %i, Current: %i]", data.voltage, data.current);
}

//Tracks the power used by the electronics
void TrackElectronicPower(const hardware_interface::ElectronicPowerData& data)
{
	float calculatedPower = data.powerLSB*data.power; //Calculates power based on the digital value from the INA226 and the bit-to-Watt conversion ratio calculated in the ina226 node
	//ROS_INFO("%f",data.powerLSB);
	//ROS_INFO("%i",data.power);
	powerReadings.push_back(calculatedPower); //Add reading to vector
	
	//ROS_INFO("actual power: %f",calculatedPower);
	
}

//Adds together all power readings to get total power usage, resets the powerReadings vector with the updated value and saves the total power reading to a file for logging purposes
void UpdateElectronicPowerUsage()
{
	for(int i = 1; i < powerReadings.size(); i++)
		powerReadings[0] += powerReadings[i];
		
	powerReadings.resize(1); //resizes vector with new sum value
	
	ofstream powerFile;
	powerFile.open("/home/odroid/PowerUsageLog.txt", ios::out | ios::app);
	if(powerFile.is_open())
	{
		powerFile << current_time - start_time << "\t";
		powerFile << powerReadings[0] << " W\n";
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

    //Initilize the power monitoring node
    ros::init(argc, argv, "power_monitoring_node");
	
	//Setup file for power logging
	ofstream powerFile;
	powerFile.open("/home/odroid/PowerUsageLog.txt", ios::out | ios::trunc);
	if(powerFile.is_open())
	{	
		powerFile << "UF AggreGator Mining Team, Electronics Power Logged during a Competition Run. \n";
		powerFile << "This file shows total power used up to the indicated time. \n";
		powerFile << "Value on the left is time in seconds since the system was running. \n";
		powerFile << "Value on the right is the total power used up to the indicated time duration. \n";
		powerFile.close();
	}
	else
		ROS_ERROR("Cannot open file for power logging!");
		
		
    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize publishers, subscribers, and services
    raw_motor_power_sub = n.subscribe("raw_motor_power", 1000, ConvertRawMotorToPower);
    electronic_power_sub = n.subscribe("electronic_power", 1000, TrackElectronicPower);
    
    start_time = ros::Time::now(); //Start time
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
