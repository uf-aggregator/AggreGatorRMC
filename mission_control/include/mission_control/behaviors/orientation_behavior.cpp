#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "behaviors.h"
#include "common_files/Coordinates.h"

OrientationBehavior::OrientationBehavior(){
	lastSeen = NONE;
	x1 = -1.0;
	y1 = -1.0;
	x2 = -1.0;
	y2 = -1.0;
	width = 0.0;
	height = 0.0;
}

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

void OrientationBehavior::orient(){
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
}



