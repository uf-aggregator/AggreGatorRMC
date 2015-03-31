
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "common_files/Motor.h"
#include "common_files/WriteI2C.h"

common_files::Motor last_motor_msg;
ros::Time last_motor_time;
ros::Time last_i2c_pub_time;

ros::Subscriber motor_sub;

void callBack (const common_files::Motor motor_msg){
	last_motor_msg = motor_msg;
	last_motor_time = ros::Time::now();
}

common_files::WriteI2C generate_msg(){
	//this function always uses the last motor msg
	common_files::WriteI2C i2c_msg;
	i2c_msg.addr = 1;
	/* Note about direction byte
		Left Forward, Right Forward: 0
		Left Forward, Right Backward: 1
		Left Backward, Right Forward: 2
		Left Backward, Right Backward: 3
	*/
	if(last_motor_msg.left >= 0 && last_motor_msg.right >= 0){
		//Left Forward, Right Forward
		i2c_msg.data.push_back(0);
	}
	if(last_motor_msg.left >= 0 && last_motor_msg.right < 0){
		//Left Forward, Right Backward
		i2c_msg.data.push_back(1);
	}
	if(last_motor_msg.left < 0 && last_motor_msg.right >= 0){
		//Left Backward, Right Forward
		i2c_msg.data.push_back(2);
	}
	if(last_motor_msg.left < 0 && last_motor_msg.right < 0){
		//Left Backward, Right Backward
		i2c_msg.data.push_back(3);
	}
	char left = (char) (abs(last_motor_msg.left*255.0));
	char right = (char) (abs(last_motor_msg.right*255.0));
	ROS_INFO("left: %d, right: %d", left, right);
	i2c_msg.data.push_back( left );
	i2c_msg.data.push_back( right );
	return i2c_msg;
	
}

common_files::WriteI2C generate_stop_msg(){
	common_files::WriteI2C i2c_msg;
	i2c_msg.addr = 1;
	i2c_msg.data.push_back(0);
	i2c_msg.data.push_back(0);
	i2c_msg.data.push_back(0);
	return i2c_msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_node"); 
	ros::NodeHandle n;
	
	//Subscribe to motor value messages
	motor_sub = n.subscribe("motor_vals", 10, callBack);

	//Publish values  on I2C
	ros::Publisher i2c_pub = n.advertise<common_files::WriteI2C>("write_i2c", 1000);

	ros::Duration motor_update_rate(.25); 
	ros::Time curr_time = ros::Time::now();
	
	while(ros::ok()){
		curr_time = ros::Time::now();
		if(curr_time - last_i2c_pub_time > motor_update_rate){
			//it's time to update the motors
			//check if we've received a motor_msg recently
			if(curr_time - last_motor_time > motor_update_rate){
				//the last motor msg has expired
				//send a message to stop the motors
				common_files::WriteI2C i2c_msg = generate_stop_msg();
				i2c_pub.publish(i2c_msg);
				last_i2c_pub_time = ros::Time::now();
			}else{
				//the last motor msg is recent enough to send
				common_files::WriteI2C i2c_msg = generate_msg();
				i2c_pub.publish(i2c_msg);
				last_i2c_pub_time = ros::Time::now();
			}
		}
		ros::spinOnce();
	}
}
