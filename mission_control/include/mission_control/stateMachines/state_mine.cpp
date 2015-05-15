#include "state_mine.h"

MineState::MineState(States state){
	//for initializing data members for the particular state
	switch(state){
		case MINE:
			ROS_INFO("Starting MINE state...");
			break;
	}
}

int MineState::mine() {
	ros::Time startTime = ros::Time::now();
	ros::Duration lower_duration(5.0); //in seconds

	//lower the ladder
	while( startTime - ros::Time::now() < lower_duration ) { //loop for 5 seconds
		motor_utility::write_to_ladder(1, 0.0);
	}

	//begin the ladder convolutions
	startTime = ros::Time::now();
	ros::Duration mine_duration(45.0);
	while( startTime - ros::Time::now() < mine_duration ) { //loop for 45 seconds
		motor_utility::write_to_ladder(0.0, 8);	
	}	
	
	//raise the ladder
	startTime = ros::Time::now();
	while( startTime - ros::Time::now() < lower_duration ) { //loop 
		motor_utility::write_to_ladder(-1, 0.0);
	}

	//stop mining ops
	motor_utility::write_to_ladder(0.0, 0.0);	

	return 0;
}