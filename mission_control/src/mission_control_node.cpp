/*************************
 * PENDING REMOVAL - this program is potentially unneeded
 * as we could just run everything from MissionControl
 *I can't think of much reason why I had this level of abstraction
 *************************/

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mission_control/mission_control.h"
#include <common_files/Motor.h>

using namespace std;

ros::Publisher pub;

//Timing variables
ros::Time last_time(0), current_time;
ros::Duration update_rate(5.0);  //turn for 1 second

int main(int argc, char **argv){
	ros::init(argc, argv, "mission_control_node");
	ros::NodeHandle nh;
        pub = nh.advertise<common_files::Motor>("motor_rc",1000);

	MissionControl *mc = new MissionControl();
	mc->StartSenseAct();
/*	
	int count = 0;
	current_time = ros::Time::now();
	last_time = current_time;
	while(ros::ok() && current_time - last_time < update_rate){
		current_time = ros::Time::now();
		last_time = current_time;
		common_files::Motor motor_msg;
        	motor_msg.leftFront_motorVal = 1.0;
        	motor_msg.leftRear_motorVal = 1.0;
        	motor_msg.rightFront_motorVal = -1.0;
        	motor_msg.rightRear_motorVal = -1.0;
        	ROS_INFO("case MOVE: %f, %f", motor_msg.leftFront_motorVal, motor_msg.rightFront_motorVal);
                pub.publish(motor_msg);
		ros::spinOnce();
	}

	while(ros::ok()){
		common_files::Motor motor_msg;
	        motor_msg.leftFront_motorVal = 0.0;
	        motor_msg.leftRear_motorVal = 0.0;
	        motor_msg.rightFront_motorVal = 0.0;
	        motor_msg.rightRear_motorVal = 0.0;
	        ROS_INFO("case MOVE: %f, %f", motor_msg.leftFront_motorVal, motor_msg.rightFront_motorVal);
	        pub.publish(motor_msg);
		ros::spinOnce();
	}
*/
	return 0;
}
