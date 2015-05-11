/*
	angle node
	subscribes to mpu_node to determine heading of robot

*/

#include <ros/ros.h>

ros::Subscriber gyro;
ros::Time reading_time(0);
ros::Time previous_time(0);

double orientation = 0.0;

void commandCallBack(const std_msgs::Int8::ConstPtr& command){
	if(command->data == 0){
		//use the last_laser as the snapshot for this command
		ROS_INFO("Zeroing");
		orientation = 0.0;	
	}else{
		//ignore the snapshot
		ROS_INFO("Removing snapshot");
	}
}

void gyroCallBack(const common_files::Gyro::ConstPtr& gyro_reading){
	if(previous_time.toSec() == 0.0){
		previous_time = ros::Time::now();
		reading_time = ros::Time::now();		
	}else{
		last_gyro = *gyro_reading;
		previous_time = reading_time;
		reading_time = ros::Time::now();
		double time_difference = reading_time.toSec() - previous_time.toSec();
		double angle = time_difference * last_gyro.x;
		orientation = orientation + angle;
	}
	
}

void rosoutCallback(const ros::TimerEvent&){
	ROS_INFO("Orientation: %f", orientation);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "angle_node.cpp");
	ros::NodeHandle nh;
	
	gyro = nh.subscribe("gyro", 1, gyroCallBack);

	ros::Timer ros_out_timer = n.createTimer(ros::Duration(.25), rosoutCallback);

	while(ros::ok()){
		ros::spinOnce();	
	}
}
