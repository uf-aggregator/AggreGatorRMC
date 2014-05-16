/*
 * Power monitoring node
 * Takes data from the ADC and converts voltage and current mesurments
 *      into power and keeps track of total power used
 */

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include "hardware_interface/RawMotorPowerData.h"
#include "hardware_interface/ElectronicPowerData.h"

using namespace std;

//Global variables
ros::Subscriber raw_motor_power_sub;
ros::Subscriber electronic_power_sub;

ros::Time last_time(0), current_time, start_time, last_callback(0); //Timing variables
ros::Duration update_rate(15); //time in seconds between power monitoring updates

//hardcode battery capacity
int TOTAL_JOULES = 133200;

//global battery percentage value
float multi_energy = 0.00;		//Stores energy over multiple runs
float current_energy = 0.00; 		//store energy over one run

/*Write percentage to file*/
bool writePercentage(float percentage){
	ofstream batteryFile("/home/odroid/BatteryPercentage.txt", ios::app | ios::out);
	if(batteryFile.is_open())
	{
		batteryFile << percentage;
		batteryFile.close();
		return true;
	}
	else {
		ROS_ERROR("Battery log can't be opened.");
		return false;
	}
}

/*Read percentage from file and then set global multi_energy to that value*/
bool readPercentage(){
	string percentage;
	ifstream batteryFile("/home/odroid/BatteryPercentage.txt", ios::in);
	if(batteryFile.is_open()){
		while(getline(batteryFile, percentage)){
			multi_energy = (float)atof(percentage.c_str());
		}//endwhile	
		batteryFile.close();
		return true;
	} else {
		ROS_WARN("Battery log can't be opened assuming 0.");
		return false;
	}
}



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
	
	current_time = ros::Time::now();
	
	float percent = (calculatedPower * (current_time - last_callback).toSec()) * 100 / TOTAL_JOULES;
	current_energy += percent;
	multi_energy += percent;
	
	last_callback = current_time;
	
	//ROS_INFO("actual power: %f",calculatedPower);
	
}

//Adds together all power readings to get total power usage, resets the powerReadings vector with the updated value and saves the total power reading to a file for logging purposes
void UpdateElectronicPowerUsage()
{
	
	ROS_INFO("Power used (this run : multiple runs) = %.2f%% : %.2f%%", current_energy, multi_energy);
	writePercentage(multi_energy);
	
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
	powerFile.open("~/PowerUsageLog.txt", ios::out | ios::trunc);
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
    last_time = start_time;
    last_callback = start_time;

    //Initilize power
    readPercentage();

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
