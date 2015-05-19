#include "navigation_behavior.h"
#include "motor_utility.h"
#include <ros/ros.h>


/* GLOBAL VARIABLES =====================================*/
const ros::Duration move_time(1.5);
const float moveSpd = 1.0;


/* STATIC METHODS =====================================*/
int NavigationBehavior::moveStraight(bool front){
	ros::Time startTime = ros::Time::now();
	
	while(startTime - ros::Time::now() < move_time){
		motor_utility::write(moveSpd, moveSpd);
	}

	return 0;
}

int NavigationBehavior::turnLeft(bool front){
	ros::Time startTime = ros::Time::now();
	
	while(startTime - ros::Time::now() < move_time){
		motor_utility::write(-moveSpd, moveSpd);
	}

	return 0;
}

int NavigationBehavior::turnRight(bool front){
	ros::Time startTime = ros::Time::now();
	
	while(startTime - ros::Time::now() < move_time){
		motor_utility::write(moveSpd, -moveSpd);
	}

	return 0;
}

int NavigationBehavior::backUp(bool front){
	ros::Time startTime = ros::Time::now();
	
	while(startTime - ros::Time::now() < move_time){
		motor_utility::write(-moveSpd, -moveSpd);
	}

	return 0;
}
