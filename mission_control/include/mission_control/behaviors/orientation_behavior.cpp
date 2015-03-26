#include <ros/ros.h>
#include "behaviors.h"

SeekTargetBehavior::SeekTargetBehavior(){
	lastSeen = -1;
}

void SeekTargetBehavior::turnLeft(){

}

void SeekTargetBehavior::turnRight(){

}

float* SeekTargetBehavior::find(){
	int argc; char **argv;

	ros::init(argc, argv, "calling_video_processing_service");
	ros::NodeHandle nh;
	char* serviceNm = "get_target_coordinates_in_meters";
	ros::ServiceClient client = nh.serviceClient<common_files::Coordinates>(serviceNm);
	common_files::Coordinates coords;

	if(client.call(coords)){
		srv.response.output[0];
		srv.response.output[1];
		srv.response.width;
		srv.response.height;
	}
	else {
		//return redo coordinates
	}
}

void SeekTargetBehavior::test(){

}



