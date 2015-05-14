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
	return 0;
}