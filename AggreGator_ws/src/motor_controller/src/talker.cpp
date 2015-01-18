/*
 * talker.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: Daniel Kelly
 */
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "motor_controller/AdaCmd.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub =n.advertise<motor_controller::AdaCmd>("adaFruit",1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		motor_controller::AdaCmd msg;

		msg.device = motor_controller::AdaCmd::wheelMotors;

		int motorData[] = {234,678,23,454};

		for(int i = 0; i < 4; i++)
		{
	    		msg.value.push_back(motorData[i]);
		
		}

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();

		//++count;
		
			
	}

	return 0;

}



