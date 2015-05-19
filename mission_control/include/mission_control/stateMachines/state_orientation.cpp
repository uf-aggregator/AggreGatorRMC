#include "state_orientation.h"

/* CONSTRUCTORS =====================================*/
OrientationState::OrientationState(States state){
	//for initializing data members for the particular state
	switch(state){
		case ORIENTATION_START:
			std::cout << "ORIENTATION_START" << std::endl;
			break;

		case ORIENTATION_DUMP:
			std::cout << "ORIENTATION_DUMP" << std::endl;
			break;
	}
}


/* METHODS =====================================*/
int OrientationState::orientToMine(){
	OrientationBehavior::updateAngle();
	OrientationBehavior::orientByCentroid();
	return 0;
}

int OrientationState::orientToDump(){
	OrientationBehavior::updateAngle();
	return 0;
}
