/*
 * Power monitoring node
 * Takes data from the ADC and converts voltage and current mesurments
 *      into power and keeps track of total power used
 */

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include "common_files/RawMotorPowerData.h"
#include "common_files/ElectronicPowerData.h"

using namespace std;

//Global variables
ros::Subscriber raw_motor_power_sub;
ros::Subscriber electronic_power_sub;

ros::Time last_time(0), current_time, start_time, last_callback(0); //Timing variables
ros::Duration update_rate(15); //time in seconds between power monitoring updates

//hardcode battery capacity
int TOTAL_JOULES = 133200;

//global battery percentage value
float multi_energy = 0.00;		//Stores energy over multiple runs, percentage
float current_energy = 0.00; 		//store energy over one run, percentage
float remaining_energy = 100.00;	//Remaining energy left in battery, percentage

/*Write percentage to file*/
bool writePercentage(float percentage_1, float percentage_2){
	ofstream batteryFile("../../../../BatteryPercentage.txt", ios::trunc | ios::out);
	if(batteryFile.is_open())
	{	
		batteryFile << "UF AggreGator, Percent of Electronics Battery Used. \n";
		batteryFile << "Single Run: " << percentage_1 << "\t"; 
		batteryFile << "Overall Total: " << percentage_2;
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
	fstream batteryFile("../../../../BatteryPercentage.txt", ios::in);
	if(batteryFile.is_open()){
		batteryFile.seekg(88,ios::beg); //Set stream pointer to read correct float value
		while(getline(batteryFile, percentage)){
			multi_energy = (float)atof(percentage.c_str());
			remaining_energy = 100.00 - multi_energy;
			//ROS_INFO("multi_energy %f",multi_energy);
		}//endwhile	
		batteryFile.close();
		return true;
	} else {
		ROS_WARN("Battery log can't be opened, assuming 0.");
		return false;
	}
}



//Converts raw data from ADC into power
void ConvertRawMotorToPower(const common_files::RawMotorPowerData& data)
{
  //  ROS_INFO("Raw data [Voltage: %i, Current: %i]", data.voltage, data.current);
}

//Tracks the power used by the electronics
void TrackElectronicPower(const common_files::ElectronicPowerData& data)
{
	float calculatedPower = data.power; //Calculates power based on the digital value from the INA226 and the bit-to-Watt conversion ratio. Calculated in the ina226 node
	//ROS_INFO("%.2f kwH", (data.voltage * data.current/1000));
	current_time = ros::Time::now();
	
	float percent = (calculatedPower * (current_time - last_callback).toSec()) * 100 / TOTAL_JOULES;
	current_energy += percent;
	multi_energy += percent;
	remaining_energy = 100 - multi_energy;
	if(remaining_energy <= 20)
		ROS_ERROR("Low Battery! Consider charging electronics battery.");
	last_callback = current_time;
	//ROS_INFO("actual power: %f",calculatedPower);
	
}

//Outputs power data for electronics and writes data to a file for logging purposes
void UpdateElectronicPowerUsage()
{
	ROS_INFO("Power used (this run : multiple runs) = %.2f%% : %.2f%%", current_energy, multi_energy);
	ROS_INFO("Remaining Power = %.2f%% ", remaining_energy);
	writePercentage(current_energy,multi_energy);	
}

int main(int argc, char** argv)
{
    /*
     * Initialization
     */

    //Initilize the power monitoring node
    ros::init(argc, argv, "power_monitoring_node");
		
	
    //Node handler this is how you work with ROS
    ros::NodeHandle n;

    //Initilize publishers, subscribers, and services
    raw_motor_power_sub = n.subscribe("raw_motor_power", 1000, ConvertRawMotorToPower);
    electronic_power_sub = n.subscribe("electronic_power", 1000, TrackElectronicPower);
    
    start_time = ros::Time::now(); //Start time
    last_time = start_time;
    last_callback = start_time;

    //Initialize power
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
