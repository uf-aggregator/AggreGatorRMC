#include <ros/ros.h>
#include "state_navigation.h"

#define TOO_CLOSE 20 //cm


/* INIT STATIC VARIABLES =====================================*/
common_files::IRDistances NavigationState::ir_distances;

/* CONSTRUCTORS =====================================*/
NavigationState::NavigationState(States state){
	//for initializing data members for the particular state
	switch(state){
		case NAVIGATE:
			ROS_INFO("Starting NAVIGATE state...");
			break;

		case GO_HOME:
			ROS_INFO("Starting GO_HOME state...");
			break;

		case GO_MINE:
			ROS_INFO("Starting GO_MINE state...");
			break; 
	}
}

/* CALLBACKS =====================================*/
void NavigationState::readIrDistancesCallback(const common_files::IRDistances::ConstPtr& msg) {
	ir_distances.IR_FL = msg->IR_FL;
	ir_distances.IR_FR = msg->IR_FR;
	ir_distances.IR_BL = msg->IR_BL;
	ir_distances.IR_BR = msg->IR_BR;
}

/* METHODS =====================================*/

/* checks ir_distances and returns whether there's an obstacle in front */
int NavigationState::obstaclesAt(bool left, bool right) {
	if(left && right){ //obstacles on both sides
		return 2;
	}
	else if(left && !right ){ //obstacle on left side
		return 1;
	}
	else if(!left && right){ //obstacle on right side
		return -1;
	}
	
	return 0; //no obstacles
}

//checks for obstacles on the front IRs
int NavigationState::checkIRs(bool front) {
	int leftIrDist = front ? ir_distances.IR_FL.cm : ir_distances.IR_BL.cm;
	int rightIrDist = front ? ir_distances.IR_FR.cm : ir_distances.IR_BR.cm;
	bool leftObstacle = leftIrDist <= TOO_CLOSE ;
	bool rightObstacle = rightIrDist <= TOO_CLOSE ;
	
	return obstaclesAt(leftObstacle, rightObstacle);
}


/* WRAPPER METHODS =====================================*/
int NavigationState::navigateToDump() {
	navigateTo(false);
}

int NavigationState::navigateToMine() {
	navigateTo(true);
}

int NavigationState::navigateTo(bool mine){
	int argc; char** argv; 
	ros::init(argc, argv, "navigate_to_mine");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("IrDistances", 1, readIrDistancesCallback);
	ros::spinOnce();
	int index = 0;

	while(++index < 15 && ros::ok()){ //find a real condition, currently placeholding
		//read in the "front" ir distances and move accordingly
		switch(checkIRs(mine)){
			case -1:
				break;
	
			case 0:
				break;
	
			case 1:
				break;
	
			case 2:
				break;
		}

		//next iteration, read new data
		ros::spinOnce();	
	}
	
	return 0;	
}
