/*
	angle node
	subscribes to mpu_node to determine heading of robot


*/

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "common_files/Gyro.h"
#include <ros/time.h>
#include <vector>

ros::Subscriber gyro;
ros::Time reading_time(0);
ros::Time previous_time(0);
ros::Time offset_calculation(0);

std::vector<double> samplesForOffset;
int calculatingOffset = 0;

double offset = 0.0;
double orientation = 0.0;



void commandCallback(const std_msgs::Int8::ConstPtr& command){
	if(command->data == 0){
		//use the last_laser as the snapshot for this command
		ROS_INFO("Zeroing");
		orientation = 0.0;	
		reading_time.fromSec(0.0);
		previous_time.fromSec(0.0);
	}else{
		//calculate the offset over a period of 4 seconds
		samplesForOffset.clear();
		calculatingOffset = 1;
	}
}

void gyroCallBack(const common_files::Gyro::ConstPtr& gyro_reading){
	if(calculatingOffset == 1){
		samplesForOffset.push_back(gyro_reading->x);
		if(samplesForOffset.size() == 400){
			calculatingOffset = 0;
			double sample_total = 0.0;
			for(int i = 0; i < samplesForOffset.size(); i++){
				sample_total = sample_total + samplesForOffset.at(i);
			}
			offset = sample_total/400.0;
			ROS_INFO("Offset calculation: %f degrees", offset);
		}
		return;
	}	

	if(previous_time.toSec() == 0.0){
		previous_time = ros::Time::now();
		reading_time = ros::Time::now();		
	}else{
		previous_time = reading_time;
		reading_time = ros::Time::now();
		double time_difference = reading_time.toSec() - previous_time.toSec();
		double angle = time_difference * (gyro_reading->x - offset);
		orientation = orientation + angle;
	}
	
}

void rosoutCallback(const ros::TimerEvent&){
	if(calculatingOffset == 1){
		ROS_INFO("Calculating offset");	
	}else{
		ROS_INFO("Orientation: %f", orientation);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "angle_node");
	ros::NodeHandle nh;
	
	gyro = nh.subscribe("gyro", 1, gyroCallBack);
	ros::Subscriber command_topic = nh.subscribe("gyro_command", 1, commandCallback);	

	ros::Timer ros_out_timer = nh.createTimer(ros::Duration(.25), rosoutCallback);

	while(ros::ok()){
		ros::spinOnce();	
	}
}
