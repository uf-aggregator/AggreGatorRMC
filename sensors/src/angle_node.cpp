/*
	angle node
	subscribes to mpu_node to determine heading of robot
	
*/

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include "common_files/Gyro.h"
#include <ros/time.h>
#include <vector>

ros::Subscriber gyro;
ros::Subscriber command_topic;

ros::Publisher pub_angle;

ros::Time reading_time(0);
ros::Time previous_time(0);
ros::Time offset_calculation(0);

std::vector<double> samplesForOffset;
int calculatingOffset = 0;

double offset = 0.0;
double orientation = 0.0;



void commandCallback(const std_msgs::Int8::ConstPtr& command){
	if(command->data == 0){
		//Reset the orientation of the robot
		//This would typically be performed before a turn
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
	if(calculatingOffset == 1){	//if offset calculation command was received
		//save this sample in a vector of samples
		samplesForOffset.push_back(gyro_reading->x);
		if(samplesForOffset.size() == 400){ //at 100 Hz, this is four seconds
			//clear the calculatingOffset flag when finished
			//and then calculate the average of th samples
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
		//if this is the first sample after a reset, ignore it but save time
		previous_time = ros::Time::now();
		reading_time = ros::Time::now();		
	}else{
		//if this is a normal sample, integrate it!
		previous_time = reading_time;
		reading_time = ros::Time::now();
		//integration of degrees per second will give heading in degrees
		//this is a basic riemman sum, n SUMMATION i=0: f(t_of_(i)) * (t_of_(i) - t_of_(i-1))
		//every time we get a mpu callback, i increases.  But we always work with the current i. 
		//see en.wikipedia.org/wiki/Riemmann_sum
				//width is delta_t
				//height is dps
				//area is delta_t * dps
		//first calculate t_of_(i) - t_of_(i-1), or delta_t
		double time_difference = reading_time.toSec() - previous_time.toSec(); //in seconds
		//now calculate f(t_of_(i)) * (t_of_(i) - t_of_(i-1)), or area of rectangle
		double angle = time_difference * (gyro_reading->x - offset);  //in degrees
		orientation = orientation + angle; //SUMMMATION 
	}
	
}

void rosoutCallback(const ros::TimerEvent&){
	if(calculatingOffset == 1){
		ROS_INFO("Calculating offset");	
	}else{
		ROS_INFO("Orientation: %f", orientation);
		int argc; char** argv;
		ros::init(argc, argv, "publish_orientation_angle");
		
		ros::NodeHandle nh;
		pub_angle = nh.advertise<std_msgs::Float32>("orientation_angle", 1);
		
		//publish orientation to orientation_angle topic
		std_msgs::Float32 angle;
		angle.data = orientation;
		pub_angle.publish(angle);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "angle_node");
	ros::NodeHandle nh;
	
	gyro = nh.subscribe("gyro", 1, gyroCallBack);
	command_topic = nh.subscribe("gyro_command", 1, commandCallback);	

	ros::Timer ros_out_timer = nh.createTimer(ros::Duration(.25), rosoutCallback);

	while(ros::ok()){
		ros::spinOnce();	
	}
}
