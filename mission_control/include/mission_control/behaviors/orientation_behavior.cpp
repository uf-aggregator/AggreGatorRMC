#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "behaviors.h"
#include "common_files/Coordinates.h"

#define INVALID_VAL 32768.0

/* INITIALIZE STATIC MEMBERS ======================*/
Direction OrientationBehavior::lastSeen = NONE;
float OrientationBehavior::x1 = -1.0;
float OrientationBehavior::y1 = -1.0;
float OrientationBehavior::x2 = -1.0;
float OrientationBehavior::y2 = -1.0;
float OrientationBehavior::width = 0.0;
float OrientationBehavior::height = 0.0;
float OrientationBehavior::angle = INVALID_VAL;


/* CALLBACKS ======================*/
void OrientationBehavior::orientAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
	angle = msg->data;
}


/* OPERATIONAL METHODS ======================*/
// turns based on the LED static variables
void OrientationBehavior::turn(){
	float mX = abs( x1 + x2 )/2.0;
	float centerX = width/2.0;
	
	//determine how far off the midpoint of the coordinates are from the center
	float offset = abs(mX - centerX)/(centerX);
	
	//set the left and right motor values to be written
	float left_value = offset;
	float right_value = offset;

	//determine the direction to turn and mod values as necessary
	if(mX >= centerX) {
		lastSeen = RIGHT;
		right_value *= -1.0;
	}
	else {
		lastSeen = LEFT;
		left_value *= -1.0;
	}

	//write the values
	motor_utility::write(left_value, right_value);
}

//turns based on a given amount of degrees using /gyro topic
void OrientationBehavior::turn(int degrees){
	int argc; char **argv;
	ros::init(argc, argv, "turning_using_gyro_topic");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("orientation_angle", 1, orientAngleCallback);
	while(angle == INVALID_VAL)
		ros::spinOnce();

	bool negative = degrees < 0;
	degrees = negative ? degrees * -1.0 : degrees;
	float fixed = angle;
	float delta = 0.0;

	while(delta <= degrees){
		delta = angle - fixed;
		delta = std::abs(delta);

		if(negative) motor_utility::write(-50.0, 50.0);
		else motor_utility::write(50.0, -50.0);
	}

	motor_utility::stop();
}

//updates only the current angle
void OrientationBehavior::updateAngle(){
	int argc; char **argv;
	ros::init(argc, argv, "updating_angle");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("orientation_angle", 1, orientAngleCallback);
	do {
		ros::spinOnce();
	} while(angle == INVALID_VAL);
}

//uses the video processing service to find the position of the red LEDs
void OrientationBehavior::find(){
	int argc; char **argv;
	ros::init(argc, argv, "calling_video_processing_service");
	ros::NodeHandle nh;
	char const* serviceNm = "get_target_coordinates_in_meters";
	ros::ServiceClient client = nh.serviceClient<common_files::Coordinates>(serviceNm);
	common_files::Coordinates coords;

	int currIndex = 0;
	const int SAFETY = 10;
	bool found = false;

	//try to get valid response from the service
	while(!found && ++currIndex < 10){
		if(client.call(coords)){
			x1 = coords.response.output[0];
			y1 = coords.response.output[1];
			x2 = coords.response.output[2];
			y2 = coords.response.output[3];
			width = coords.response.width;
			height = coords.response.height;

			//if we have legit coordinates, break the loop
			if( (int)x1 != -1 && (int)y1 != -1 && (int)x2 != -1 && (int)y2 != -1 )
				found = true;
		}	
	}
}

//orients the robot so LEDs are in the center of the camera
void OrientationBehavior::orientToLEDs(){
	float mX = abs( x1 + x2 )/2.0;
	float centerX = width/2.0;
	
	//determine how far off the midpoint of the coordinates are from the center
	float offset = abs(mX - centerX)/(centerX);

	int currIndex = 0;
	const int SAFETY = 25;

	//turn until the offset is 5% from the center and within iteration limit
	while( offset <= 0.05 && ++currIndex < SAFETY ){
		turn();

		//recalculate the offset
		mX = abs( x1 + x2 )/2.0;
		centerX = width/2.0;
		offset = abs(mX - centerX)/(centerX);
	}

	motor_utility::stop();
}

