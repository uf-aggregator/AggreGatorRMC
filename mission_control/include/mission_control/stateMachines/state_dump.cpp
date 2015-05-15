#include "state_dump.h"

DumpState::DumpState(States state){
	//for initializing data members for the particular state
	switch(state){
		case DUMP:
			ROS_INFO("Starting DUMP state...");
			break;
	}
}

int DumpState::dump() {
	ros::Time startTime = ros::Time::now();
	ros::Duration raise_duration(10.0); //in seconds

	//raise the bucket
	while( startTime - ros::Time::now() < raise_duration ) { //loop for 5 seconds
		motor_utility::write_to_bucket(1, 0.0); //dumpLift, dumpVal
	}

	//lift to dump
	startTime = ros::Time::now();
	ros::Duration dump_duration(60.0);
	while( startTime - ros::Time::now() < dump_duration ) { //loop for 45 seconds
		motor_utility::write_to_bucket(0.0, 8);	
	}
	

	//level and lower the bucket
	startTime = ros::Time::now();
	while( startTime - ros::Time::now() < raise_duration ) { //loop 
		motor_utility::write_to_bucket(-1, -8);
	}

	motor_utility::write_to_bucket(0.0, 0.0);

	return 0;
}