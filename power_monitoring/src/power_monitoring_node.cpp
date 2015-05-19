/*
 * Power monitoring node
 * Takes data from the ADC and converts voltage and current mesurments
 *      into power and keeps track of total power used
 */

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include "std_msgs/Float32.h"

//Global variables
ros::Subscriber overall_cs;
ros::Time last_time(0), curr_time(0);
ros::Duration update_rate(5); //time in seconds between power monitoring updates

using namespace std;

//name of log file
const string log_file = "~/AggreGator_ws/src/AggreGatorRMC/power_monitoring/power_log.txt";

//hardcode battery capacity
const float TOTAL_JOULES = 133200.0;

//instantaneous power
float inst_power = 0.0;

//joules used in previous and current run
float prev_coulombs = 0.0;
float curr_coulombs = 0.0;


bool writeCoulombs(float total_coulombs){
	ofstream batteryFile("/home/odroid/AggreGator_ws/src/AggreGatorRMC/power_monitoring/power_log.txt", fstream::trunc | fstream::out); //trunc will clear the file while opening
	if(batteryFile.is_open())
	{	
		//TODO: Write accumulated power
		batteryFile << total_coulombs;
		batteryFile.close();
		return true;
	}
	else {
		ROS_ERROR("Battery log can't be opened.");
		return false;
	}
}

bool readCoulombs(){
	string temp_str;
	fstream batteryFile("/home/odroid/AggreGator_ws/src/AggreGatorRMC/power_monitoring/power_log.txt", ios::in);
	if(batteryFile.is_open()){
		getline(batteryFile, temp_str);
		prev_coulombs = (float)atof(temp_str.c_str());
		batteryFile.close();
		return true;
	} else {
		prev_coulombs = 0.0;
		ROS_WARN("Battery log can't be opened, assuming no power used.");
		return false;
	}
}

void currentCallback(const std_msgs::Float32::ConstPtr& current){
	last_time = curr_time;
	curr_time = ros::Time::now(); 
	if(last_time.toSec() != 0.0){
		inst_power = current->data*22.0;	
		float inst_coulombs = current->data*(curr_time.toSec() - last_time.toSec());
		//ROS_INFO("inst_coulombs = %f", inst_coulombs);
		//ROS_INFO("curr coulombs, before addition: %f", curr_coulombs);
		curr_coulombs = curr_coulombs + inst_coulombs;
		//ROS_INFO("curr_coulombs, after addition:  %f", curr_coulombs);
	} //else, ignore the first value
}

void logCallback(const ros::TimerEvent&){
	ROS_INFO("Coulombs used this run: %f", curr_coulombs);
        ROS_INFO("Coulombs used overall: %f", (curr_coulombs + prev_coulombs));
	ROS_INFO("Watt*hours used this run: %f", curr_coulombs*22/3600);
	ROS_INFO("Watt*hours used overall: %f", curr_coulombs*22/3600);
	ROS_INFO("Current power usage (watts): %f", inst_power);
        writeCoulombs(curr_coulombs + prev_coulombs);
}

int main(int argc, char** argv)
{
    //Initilize the power monitoring node
    ros::init(argc, argv, "power_monitoring_node");
    ros::NodeHandle n;

    //Initilize publishers, subscribers, and services
    overall_cs = n.subscribe("overall_current", 1000, currentCallback);
    ros::Timer log_timer = n.createTimer(ros::Duration(1), logCallback);	    
    //Initialize power
    readCoulombs();

    /*
     * Main loop
     */
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
